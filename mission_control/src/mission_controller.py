# Mission controller node
# Viene lanciato all'avvio del sistema e si occupa di scegliere la missione e portarla a avanti.
# In particolare controlla in una specifica cartella se ci sono missioni da eseguire se presenti avvia la missione più recente se non sono presenti resta in attesa di una missione
# Legge i waiponts dal file GPX e li invia uno ad uno al mission executor
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.0
# Data: 10/09/2025
# Note: Questo script richiede la libreria 'gpxpy'. Installala tramite 'pip install gpxpy'.

#!/usr/bin/env python3

import rospy
import gpxpy
import os
import glob
import actionlib
from collections import deque
import time
import json
from typing import NamedTuple
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int32, Int16, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.srv import GetPlan
import tf2_ros
import tf2_geometry_msgs
import utm
import math
from actionlib_msgs.msg import GoalStatusArray, GoalID
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from diagnostic import UsvDiagnostic
from mission_ext_interface import missionExtInterface
from rosgraph import Master, MasterException


DEBUG = True
MISSION_DIRECTORY_PATH = "/home/davide/Scrivania/MISSION_CONTROLLER/mission_control"
MISSIONS_JSON_FILE = os.path.join(MISSION_DIRECTORY_PATH, "missions_history.json")
DRONE_ID = 1
# --- Timeout di Sistema ---
CMD_VEL_TIMEOUT = 30 # Tempo in secondi (5 minuti) dopo il quale se non ricevo comandi di velocità torno in stato 0: inattivo in attesa
INACTIVITY_TIMEOUT = 900 # Secondi (15 minuti) dopo il quale se non ricevo comandi di velocità torno in stato 0: inattivo in attesa
MB_SERVER_TIMEOUT = 5 # Secondi
PENDING_TIMEOUT = 120
MAX_MBS_RETRY_CNT = 3
# --- Frame di Riferimento ---
# FRAME_ID_MAP: Il frame globale di move_base (solitamente 'map')
FRAME_ID_MAP = "map" 
# FRAME_ID_UTM: Il frame UTM che navsat_transform_node usa come riferimento globale
FRAME_ID_UTM = "utm"
# FRAME_BASE_ID: il frame di base del robot (solitamente 'base_link')
FRAME_BASE_ID = "base_link"

TO_SEC = 1000000000

ROSTIMEOUT = 10 # secondi

mappa_degli_stati_usv = {
    0: "In attesa",
    1: "Avvio nav. autonoma",
    2: "Navigazione autonoma",
    3: "Navigazione da remoto",
    4: "Errore minore", 
    5: "Errore critico"
    }

mappa_move_base_status = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
   -1: "UNKNOWN"
}

mappa_errori = {
    0: "sensore fuori uso",
    1: "motore fuori uso",
    2: "errore planner",
    3: "errore del localizzatore"
}

mappa_errori_planner = {
    0: "Errore del navigation server",
    1: "Errore di calcolo percorso"
}

mappa_errore_sensori = {
    0: "IMU fuori uso",
    1: "GPS fuori uso",
    2: "Bussola fuori uso",
    3: "Impossibile calcolare l'odometria"
}

# Struttura che contiene info sulle missioni eseguite
class MissionInfo(NamedTuple):
    mission_id: str
    mission_status: str
    # il resto sono idee suggerite da copilot, capiremo in futuro se implementarle
    total_waypoints: int
    waypoint_reached: int
    total_distance: float
    distance_traveled: float
    mission_duration: float # secondi da convertire in ore minuti

mb_cnt = tf_cnt = 1
retry_cnt = [mb_cnt, tf_cnt]

class MissionController:
    def __init__(self):
        # Variabili interne
        self.mission_files = None
        self.waypoints = None
        self.mission_directory = MISSION_DIRECTORY_PATH
        self.usv_status = mappa_degli_stati_usv[0] # stato iniziale inattivo in attesa
        self.navigation_status = GoalStatusArray()
        self.tf_buffer = tf2_ros.Buffer()
        self.move_base_client = None
        self.mission_index = 0
        self.preemption_counter = 0
        self.new_mission_id = None
        self.errori_minori = []
        self.planner_ripristinato = False
        self.missions = []
        self.load_missions_from_json()  # Carica le missioni dal file JSON all'avvio
        self.waypoints_reached = 0
        self.can_jump = False
        self.lunghezza_segmenti = []
        self.prev_stamp = time.time() # Timestamp
        self.durata_missione = 0.0
        self.prev_pos = (0.0, 0.0)
        self.vel_media = deque(maxlen=10)
        # Variabili di stato
        self.is_valid_mission = False
        self.is_remote_control = False
        self.nav_system_active = [False, False] # 0: move_base, 1: tf
        self.retry_cnt = retry_cnt
        self.cc_cmd_vel = Twist()
        self.move_base_cmd_vel = Twist()
        self.minor_error = False
        self.major_error = False
        self.system_status = Int16
        # Altre classi
        self.usvDiagnostic = UsvDiagnostic()
        self.gndStationIf = missionExtInterface(MISSION_DIRECTORY_PATH, DRONE_ID)
        # Publisher e Subscriber
        #self.pub_waypoint = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_status = rospy.Publisher('/usv_status', Int32, queue_size=10)
        self.pub_vel_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.gps_goal_2convert = rospy.Publisher('/initial_navsat_fix', NavSatFix, queue_size=1)
        self.pub_abort_current_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)        
        self.heartbeat = rospy.Publisher('/mission_controller/heartbeat', std_msgs.msg.Header, queue_size=10)
        self.sub_cc_cmd_vel = rospy.Subscriber('/CC/cmd_vel', Twist, self.cmd_vel_cc_callback) # spostato
        self.sub_move_base_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_move_base_callback)
        #self.sub_goal_reached = rospy.Subscriber('/move_base/status', GoalStatusArray, self.usv_status_callback)
        self.goal_feedback = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.usv_feedback_callback)
        self.result = None 
        self.sub_diagnostic_status = rospy.Subscriber('/usv_status_monitor/diagnostic/status', Int16, self.diagnostic_status_callback)
        self.sub_remote_request = rospy.Subscriber('/cc/remote_cmd', Bool, self.remote_cmd_callback) # spostato
        self.last_cmd_vel_time = None
        self.last_cmd_vel_time_move_base = None
        self.pending_time = None
        rospy.init_node('mission_controller', anonymous=True)
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.run) # Avvia la macchina a stati in un thread separato

    

    # Funzioni interne
    def start_control_node():
        rospy.loginfo("Mission controller avviato.")
        rospy.spin() # Gestisco i callback ROS

    def jump_to_state(self, state):
        """
        Permette di rieseguire il loop prima che scada nuovamente il timer.
        """
        if state in mappa_degli_stati_usv:
            self.usv_status = mappa_degli_stati_usv[state]
            self.can_jump = True
        else:
            rospy.logerr(f"Stato {state} non valido.")
    
    # Funzioni per il caricamento delle missioni da file GPX
    def search_for_mission(self, mission_directory):
        """
        Cerca nella cartella specificata se ci sono file GPX.
        """
        file_gpx = []
        if not os.path.isdir(mission_directory):
            rospy.logerr(f"Directory {mission_directory} non trovata.")
            return None
        
        file_gpx = glob.glob(os.path.join(mission_directory, '*.gpx'))
        
        return file_gpx
    
    def check_last_mission(self, mission_files):
        """
        Controlla tra i file GPX trovati quale è il più recente.
        """
        if not mission_files:
            rospy.loginfo("Nessun file GPX trovato.")
            return None
        
        latest_file = max(mission_files, key=os.path.getctime)
        return latest_file
    
    def load_mission(self, mission_path):
        """
        Carica la missione dal file GPX, andando ad inserire la lista di waypoint espressi come lat/lon in una lista.
        """
        try:
            rospy.loginfo(f"Caricamento della missione da {mission_path}...")
            with open(mission_path, 'r') as file:
                gpx_content = file.read()
                gpx = gpxpy.parse(gpx_content)
                self.waypoints = [
                    (point.latitude, point.longitude) 
                    for track in gpx.tracks 
                    for segment in track.segments 
                    for point in segment.points
                    ]
                if gpx.tracks and gpx.tracks[0].name:
                    self.new_mission_id = gpx.tracks[0].name
                elif gpx.name:  # alcuni file GPX hanno il nome in <gpx><name>
                    self.new_mission_id = gpx.name
                else:
                    # fallback: usa il nome del file senza estensione
                    import os
                    self.new_mission_id = os.path.splitext(os.path.basename(mission_path))[0]
                
                if DEBUG:
                    rospy.loginfo(f"Missione caricata correttamente con {len(self.waypoints)} waypoints.")
                return len(self.waypoints)
            
        except Exception as e:
            rospy.logerr(f"Errore nel caricamento della missione: {e}")
            return  0

    def vel_multiplexer(self):
        # Funzione per multiplexare i comandi di velocità
        # Parto dalle condizioni più importanti cioè la scelta tra controllo remoto e navigazione autonoma
        if self.usv_status in [mappa_degli_stati_usv[1], mappa_degli_stati_usv[2]]:

            if self.is_remote_control:
                if self.last_cmd_vel_time and self.last_cmd_vel_time - rospy.get_time() > CMD_VEL_TIMEOUT:
                    return Twist() # Torno velocità zero se non ricevo comandi da più di CMD_VEL_TIMEOUT secondi
                else:
                    return self.cc_cmd_vel
            else:
                if self.last_cmd_vel_time_move_base and self.last_cmd_vel_time_move_base - rospy.get_time() > CMD_VEL_TIMEOUT:
                    return Twist()
                else:
                    return self.move_base_cmd_vel
        else:
            return Twist() # Torno velocità zero se non sono in stato di navigazione o controllo remoto

    # ------------- CALBACK PER I RESULT DI move_base ----------------
    def cmd_vel_move_base_callback(self, msg_twist):
        """ 
        Callback per il topic /move_base/cmd_vel.
        """
        self.move_base_cmd_vel = msg_twist
        if rospy.core.is_initialized():
            self.last_cmd_vel_time_move_base = rospy.get_time()

    def usv_status_callback(self, msg):
        last_time = msg.header.stamp
        if last_time :
            self.planner_active = True
        
    def usv_feedback_callback(self, msg):
        pass

    def nav_result_callback(self, msg):
        if not msg:
            return        
        latest_status = mappa_move_base_status[msg.status_list[-1].status]
        if latest_status == mappa_move_base_status[3]: # SUCCEEDED
            self.navigation_status = mappa_move_base_status[3]
        elif latest_status == mappa_move_base_status[4]: # ABORTED
            self.navigation_status = mappa_move_base_status[4]
        elif latest_status == mappa_move_base_status[1]: #ACTIVE
            self.navigation_status = mappa_move_base_status[1]
        elif latest_status == mappa_move_base_status[2]: # PREEMPTED
            self.navigation_status = mappa_move_base_status[2]
        elif latest_status == mappa_move_base_status[0]: # PENDING
            self.navigation_status = mappa_move_base_status[0]
        elif latest_status == mappa_move_base_status[5]: # REJECTED
            self.navigation_status = mappa_move_base_status[5]
        else:
            pass
 
    def check_system_status(self, system_status):
        """
        Interpreta lo stato del"""    
        sensor_fail =[False, False] # GPS, IMU
        if self.usvDiagnostic.status.sensor_presence:
            # ----------------- GPS ------------------
            if self.usvDiagnostic.status.gps_presence:
                if self.usvDiagnostic.status.is_gpspos:
                    sensor_fail[0] = False
                else:
                    sensor_fail[0] = True
                    rospy.logwarn(f"GPS fail: impossibile calcolare la soluzione")

                if self.usvDiagnostic.status.is_gpsvel:
                    rospy.loginfo(f"GPS: qualità della soluzione per la velocità: {self.usvDiagnostic.status.gps_vel_type}") # Al momento tralasciato perché non ci da la Speed Over Groung SOG

                if self.usvDiagnostic.status.is_gps_hdt:
                    self.gndStationIf.set_COG(self.usvDiagnostic.status.COG)
                else:
                    rospy.logwarn(f"GPS fail: COG da GPS non disponibile, COG calcolata dal planner.")
                
                rospy.loginfo(f"GPS: qualità del segnale GPS: {self.usvDiagnostic.status.gps_type}.")
            else:
                sensor_fail[0]= True
                rospy.logerr("GPS: GPS interno al sensore Ellipse non è disponibile.")

            # ---------------- IMU -------------------
            if self.usvDiagnostic.status.imu_presence:
                if self.usvDiagnostic.status.is_imu:
                    sensor_fail[1] = False
                else:
                    sensor_fail[1]= True
                    rospy.logwarn(f"IMU fail: soluzione non affidabile.")
            else:
                self.minor_error = True
                rospy.logerr("La IMU interna al sensore Ellipse non è disponibile.")

            # Setto il minor error
            if not sensor_fail[0] or not sensor_fail[1]:
                self.minor_error = True
            elif sensor_fail[0] and sensor_fail[1]:
                self.minor_error = False

        else:
            self.minor_error = True
            rospy.logerr("Sensore Ellipse non disponibile.")


    def calculatePose(waypoint1, waypoint2):
        """
        Calcola l'orientamento in quaternioni tra due punti convertiti in coordinate cartesiane
        """
        delta_x = waypoint2[0] - waypoint1[0]
        delta_y = waypoint2[1] - waypoint1[1]

        yaw = math.atan2(delta_y, delta_x)

        # Conversione in quaternioni
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw) # qx, qy, qz, qw

    def convert_gps_to_map(self, lat, long, elev):
        """
        Converte le coordinate GPS (Lat/Lon) nel frame cartesiano 'map'
        sfruttando la trasformata utm -> map pubblicata da navsat_transform_node.
        """
        # --- Passo A: Conversione GPS (Lat/Lon) in coordinate UTM cartesiane ---
        # utm.from_latlon restituisce (easting, northing, zone_number, zone_letter)
        easting, northing, _, _ = utm.from_latlon(lat, long)
        
        rospy.loginfo(f"Goal GPS convertito in UTM: X={easting:.2f}, Y={northing:.2f}")

        self.listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform("map", "odom", rospy.Time(0))

        # --- Passo B: Creazione del PoseStamped (nel frame UTM) ---
        # La posizione UTM è il nostro goal nel frame globale (non il frame locale 'map')
        pose_utm = PoseStamped()
        pose_utm.header.stamp = rospy.Time.now()
        pose_utm.header.frame_id = FRAME_ID_UTM
        
        pose_utm.pose.position.x = easting - trans[0]
        pose_utm.pose.position.y = northing - trans[1]

        # Orientamento (neutro, rivolto in avanti sull'asse X UTM)
        pose_utm.pose.orientation.w = 1.0

        # --- Passo C: Trasformazione da UTM a MAP (usando la TF) ---
        try:
            # Trasforma il goal dal frame 'utm' al frame 'map'
            pose_map = self.tf_buffer.transform(pose_utm, FRAME_ID_MAP, rospy.Duration(1.0))
            
            return pose_map
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Errore durante la trasformazione utm -> map: {e}")
            return None

    def send_converted_goal(self, waypoint):
        """
        Invia tramite ActionLib i goal di navigazione a move_base.
        """
        pose_map = self.convert_gps_to_map(waypoint[0], waypoint[1], waypoint[2])
        
        if pose_map is None:
            rospy.logerr("Impossibile inviare il goal: conversione fallita.")
            return

        # 1. Creazione del MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = pose_map
        
        X_map = pose_map.pose.position.x
        Y_map = pose_map.pose.position.y
        
        # 2. Invio del Goal
        rospy.loginfo(f"Invio del Goal ActionLib: X={X_map:.2f} m, Y={Y_map:.2f} m nel frame '{FRAME_ID_MAP}'")
        self.move_base_client.send_goal(goal)
        
        # Opzionale: attendi il risultato
        # self.move_base_client.wait_for_result()
        # if self.move_base_client.get_state() == actionlib.SimpleClientGoalState.SUCCEEDED:
        #     rospy.loginfo("Goal GPS raggiunto!")
        # else:
        #     rospy.logwarn("Goal GPS fallito.")
        
        rospy.signal_shutdown("Goal inviato.")
  
    # Funzioni per il salvataggio di una missione e l'aggiornamento di una missione interrotta. 
    def updateMissionInfo(self, index, new_mission_status):
        mission_to_update = self.missions[index]
        new_mission = mission_to_update._replace(
            mission_status=new_mission_status,
            waypoint_reached=self.mission_index-1,
            distance_traveled=mission_to_update.total_distance - self.distanza_rimanente()
        )
        self.missions = new_mission
        rospy.loginfo("Missione aggiornata")

    def load_missions_from_json(self):
        """
        Carica le informazioni delle missioni dal file JSON.
        """
        try:
            if os.path.isfile(MISSIONS_JSON_FILE):
                with open(MISSIONS_JSON_FILE, 'r') as f:
                    missions_data = json.load(f)
                    self.missions = []
                    for mission_dict in missions_data:
                        mission = MissionInfo(
                            mission_id=mission_dict['mission_id'],
                            mission_status=mission_dict['mission_status'],
                            total_waypoints=mission_dict['total_waypoints'],
                            waypoint_reached=mission_dict['waypoint_reached'],
                            total_distance=mission_dict['total_distance'],
                            distance_traveled=mission_dict['distance_traveled'],
                            mission_duration=mission_dict['mission_duration']
                        )
                        self.missions.append(mission)
                    rospy.loginfo(f"Caricate {len(self.missions)} missioni dal file JSON.")
            else:
                rospy.loginfo("File di storia missioni non trovato. Inizio con lista vuota.")
                self.missions = []
        except Exception as e:
            rospy.logerr(f"Errore nel caricamento delle missioni dal JSON: {e}")
            self.missions = []

    def save_missions_to_json(self):
        """
        Salva le informazioni delle missioni nel file JSON.
        """
        try:
            missions_data = []
            for mission in self.missions:
                mission_dict = {
                    'mission_id': mission.mission_id,
                    'mission_status': mission.mission_status,
                    'total_waypoints': mission.total_waypoints,
                    'waypoint_reached': mission.waypoint_reached,
                    'total_distance': mission.total_distance,
                    'distance_traveled': mission.distance_traveled,
                    'mission_duration': mission.mission_duration
                }
                missions_data.append(mission_dict)
            
            # Crea la directory se non esiste
            os.makedirs(os.path.dirname(MISSIONS_JSON_FILE), exist_ok=True)
            
            with open(MISSIONS_JSON_FILE, 'w') as f:
                json.dump(missions_data, f, indent=4)
            rospy.loginfo(f"Missioni salvate nel file JSON: {MISSIONS_JSON_FILE}")
        except Exception as e:
            rospy.logerr(f"Errore nel salvataggio delle missioni nel JSON: {e}")

    def saveMissionInfo(self, mission_status="completed"):
        """
        Salva localmente le informazione delle missioni eseguite e di quelle interrotte (abortite o sospese) in formato JSON.
        """
        lunghezza_totale = sum(self.lunghezza_segmenti) + math.sqrt((self.get_usv_pose()[0]-self.waypoints[0][0])**2 + 
                                                                    (self.get_usv_pose()[1]-self.waypoints[0][1])**2
                                                                    )
        mission = MissionInfo(
            mission_id=self.new_mission_id,
            mission_status=mission_status,
            total_waypoints=len(self.waypoints),
            waypoint_reached=self.mission_index-1, # L'ultimo waypoint raggiunto
            total_distance=lunghezza_totale,
            distance_traveled=0.0,
            mission_duration=0.0 # Da implementare
        )
        # Controllo se la missione è già presente
        if self.new_mission_id in [mission.mission_id for mission in self.missions]:
            # Controllo se lo stato è cambiato
            for idx, m in enumerate(self.missions):
                if m.mission_id == self.new_mission_id:
                    if m.mission_status != "completed":
                        self.missions.pop(idx)
                        self.missions.append(mission)
                        rospy.loginfo(f"Aggiornato lo stato della missione {self.new_mission_id} a {mission_status}.")
                    else:
                        rospy.loginfo(f"La missione {self.new_mission_id} è già registrata con lo stato {mission_status}.")
        else:
            self.missions.append(mission)
            rospy.loginfo(f"Registrata nuova missione {self.new_mission_id} con stato {mission_status}.")
        
        # Salva le missioni nel file JSON
        self.save_missions_to_json()

    def get_usv_pose(self):
        """
        Calcola la posizione attuale dell'USV nel frame map.
        """
        try:
            # Ottieni la trasformazione da "map" a "base_link"
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            q = trans.transform.rotation
            # Se vuoi anche l'orientamento in yaw
            import tf_conversions
            euler = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = euler[2]

            return (x, y, yaw)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Trasformazione map→base_link non disponibile, ritento...")

    def get_usv_pos_global(self, x_map, y_map):
        """
        Calcola la posizione attuale dell'USV nel formato WGS84 lat/long, data la posizione nel fram locale
        """
        # Forse bisognerebbe aggiungere anche il calcolo della rotta/direzione
        zone_number = 32
        zone_letter = 'T'
        # Posizione nel frame locale 'map'
        #(x,y, yaw) = self.get_usv_pose()

        # Posizione nel frame UTM
        x_utm = x_map + self.origin[0]
        y_utm = y_map + self.origin[1]

        # Posizione in lat, long
        lat, long = utm.to_latlon(x_utm, y_utm, zone_number, zone_letter)

        return lat, long

    def distanza_rimanente(self, pos_attuale):
        """
        Calcola la distanza rimanente alla fine della missione sommando le distanze tra i waypoint rimanenti.
        """
        if len(self.lunghezza_segmenti) > 1:
            if not self.waypoints or self.mission_index > len(self.waypoints):
                return 0.0
            else: 
                distanza = 0.0
                for i in range(self.mission_index-1, len(self.waypoints)-1):
                    distanza += self.lunghezza_segmenti[i]
                distanza += self.dist(pos_attuale, self.waypoints[self.mission_index-1])
                return distanza
        else:
            return 0.0

    def calc_posa(self, x, y, frame=FRAME_ID_MAP):
        """
        Crea un PoseStamped con frame specificato.
        """
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def calc_lunghezza(self, path):
        """
        Calcola la lunghezza di una missione dato un path generato da /move_base/make_plan
        """
        if not path or len(path.poses) < 2:
            return 0.0
        lunghezza = 0.0
        for i in range(len(path.poses)-1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            lunghezza += math.sqrt((p2.x -p1.x)**2 + (p2.y - p1.y)**2)
        return lunghezza
    
    def get_path_between(self, start_pose, goal_pose):
        """
        Richiede al servizio /move_base/make_plan un path tra due punti successivi.
        Restituisce il path se trovato, altrimenti None.
        """
        rospy.loginfo("In attesa del servizio make_plan...")
        # try:
        #     rospy.wait_for_service('move_base/make_plane', timeout=5)
        # except rospy.ROSException as e:
        #     rospy.logerr(f"Servizio make_plan non disponibile: {e}")
        #     return
        try:
            make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
            plan = make_plan(start_pose, goal_pose, 0.5)
            return plan.plan
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore durante la chiamata del servizio make_plan: {e}")
            return None
        
    def lunghezza_missione(self, pos_attuale):
        """
        Calcola la lunghezza totale di una missione sommando le lunghezze della traiettoria calcolata dal planner tra i vari waypoint.
        """

        if len(self.waypoints) < 2:
            return 0.0
        
        lunghezza_totale = 0.0

        for i in range(self.mission_index, len(self.waypoints)-1): # In seguito il -1 potrebbe servire
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i+1]

            start_point = self.calc_posa(wp1[0], wp1[1], FRAME_ID_MAP)
            end_point = self.calc_posa(wp2[0], wp2[1], FRAME_ID_MAP)

            path = self.get_path_between(start_point, end_point)
            if path:
                lunghezza = self.calc_lunghezza(path)
                self.lunghezza_segmenti.append(lunghezza)
                rospy.loginfo(f"Segmento {i+1}: {lunghezza:.2f} m")
                lunghezza_totale += lunghezza
            else:
                rospy.logwarn(f"Impossibile calcolare il path tra i waypoint {i} e {i+1}")
        lunghezza_totale += self.dist(pos_attuale, self.waypoints[self.mission_index-1])
        rospy.loginfo(f"Lunghezza totale stimata della missione: {lunghezza_totale:.2f} m")
        return lunghezza_totale

    # Rappresenta il ciclo principale del mission controller    
    def run(self, event):
        try:
            self.traformazione = self.tf_buffer.lookup_transform(FRAME_ID_MAP, FRAME_BASE_ID, rospy.Time(0), timeout=rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Trasformazione map->base_link non disponibile.")

        self.can_jump = True
        while self.can_jump and not rospy.is_shutdown():
            self.can_jump = False
            # Eseguo la diagnostica
            self.system_status = self.usvDiagnostic.get_usv_status()
            self.check_system_status()
            # Eseguo il ciclo di controllo
            self.state_machine()
            # Pubblico/Eseguo la telemetria
            self.gndStationIf.pub_telemetry(self.usv_status, self.get_usv_pos_global())

    def state_machine(self):
        
        self.heartbeat.publish(std_msgs.msg.Header(stamp=rospy.Time.now()))
        # Macchina a stati del mission controller
        # Stato 0: Inattivo in attesa
        if self.usv_status == mappa_degli_stati_usv[0]:
            rospy.loginfo("STATO: inattivo in attesa")
            # Leggo stati
            if self.is_remote_control:
                self.jump_to_state(2) # Passa a 'controllo remoto'
                
            elif self.is_valid_mission and not self.is_remote_control:
                self.jump_to_state(5) # Passa a 'avvio nav. autonoma'
                
            else:                    
                # Cerca una missione
                self.mission_files = self.search_for_mission(self.mission_directory)
                if self.mission_files:
                    # Se trova delle missioni carica la più recente
                    last_mission = self.check_last_mission(self.mission_files)
                    if last_mission:
                        if self.load_mission(last_mission):
                            if self.new_mission_id in [mission.mission_id for mission in self.missions]:
                                try:
                                    active_mission = next(
                                        mission for mission in self.missions if mission.mission_id == self.new_mission_id
                                    )
                                    if active_mission.mission_status == "completed":
                                        self.is_valid_mission = False
                                        rospy.loginfo("Missione già eseguita. Attendere il caricamento di una nuova missione.")
                                    else:
                                        self.is_valid_mission = True
                                        # self.planner_ripristinato = True
                                        # Qui sarebbe corretto inserire un caricamento della missione interrotta
                                        rospy.loginfo("Riprendo la missione precedentemente iniziata e abortita.")
                                        
                                except StopIteration:
                                    rospy.loginfo(f"La Missione {self.new_mission_id} non è ancora iniziata.")
                                    self.is_valid_mission = True
                            else:
                                self.is_valid_mission = True
                                rospy.loginfo("Nuova missione disponibile.")
                                self.mission_index = 0
                        else:
                            self.is_valid_mission = False
                            rospy.logerr("Errore nel caricamento della missione.")
                    else:
                        self.is_valid_mission = False
                        rospy.loginfo("Nessuna missione trovata.")
                else:
                    self.is_valid_mission = False
                    rospy.loginfo("Nessuna missione trovata.")
            self.pub_status.publish(0) 
            
        # Stato 2: In navigazione
        elif self.usv_status == mappa_degli_stati_usv[2]:
            
            rospy.loginfo("STATO: in navigazione autonoma")
            if self.is_remote_control:
                self.pub_abort_current_goal.publish(GoalID()) # Comando di abort
                self.saveMissionInfo("suspended")
                self.jump_to_state(2) # Passa a 2: 'controllo remoto'
                
            elif not self.is_remote_control and not self.is_valid_mission:
                self.jump_to_state(0) # Torno in stato 0: inattivo in attesa
                
            elif self.minor_error:
                self.jump_to_state(3) # Passa a 3: 'errore minore'
                
            elif self.major_error:
                self.jump_to_state(4) # Passa a 4: 'errore critico'
                
            else:
                # Inizio della missione/navigazione
                if self.mission_index == 0:
                    rospy.loginfo("Inizio della missione...")
                    waypoint = self.waypoints[self.mission_index]
                    # self.send_converted_goal(self.waypoints[self.mission_index])
                    # ------------------------------------------------
                    # ------- Per prova invio diretto del goal -------
                    # ------------------------------------------------
                    goal = MoveBaseGoal()
                    goal.target_pose.header = Header()
                    goal.target_pose.header.seq = self.mission_index
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.pose.position.x = waypoint[0]
                    goal.target_pose.pose.position.y = waypoint[1]
                    goal.target_pose.pose.position.z = 0.0
                    goal.target_pose.pose.orientation.w = 1.0
                    goal.target_pose.pose.orientation.x = 0.0
                    goal.target_pose.pose.orientation.y = 0.0
                    goal.target_pose.pose.orientation.z = 0.0
                    # ------------------------------------------------
                    rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                    self.move_base_client.send_goal(goal)
                    self.mission_index += 1

                elif self.mission_index <= len(self.waypoints):
                    # if SUCCEEDED
                    if self.navigation_status == mappa_move_base_status[3]: 
                        rospy.loginfo(f"Prossimo waypoint {self.mission_index}")
                        if self.mission_index < len(self.waypoints):
                            rospy.loginfo("Procedo al waypoint successivo.")
                            waypoint = self.waypoints[self.mission_index]
                            goal = MoveBaseGoal()
                            goal.target_pose.header = Header()
                            goal.target_pose.header.seq = self.mission_index
                            goal.target_pose.header.stamp = rospy.Time.now()
                            goal.target_pose.header.frame_id = "map"
                            goal.target_pose.pose.position.x = waypoint[0]
                            goal.target_pose.pose.position.y = waypoint[1]
                            goal.target_pose.pose.position.z = 0.0
                            goal.target_pose.pose.orientation.w = 1.0
                            goal.target_pose.pose.orientation.x = 0.0
                            goal.target_pose.pose.orientation.y = 0.0
                            goal.target_pose.pose.orientation.z = 0.0
                            rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                            self.move_base_client.send_goal(goal)
                            self.mission_index += 1 # Indece del prossimo waypoint successivo a quello appena inviato.
                            self.waypoints_reached += 1
                            # waypoint = self.waypoints[self.mission_index]
                            # self.send_converted_goal(self.waypoints[self.mission_index]) # f"{waypoint[0]},{waypoint[1]}")
                        else:
                            rospy.loginfo("Ultimo waypoint raggiunto. Missione completata.")
                            self.is_valid_mission = False # Torno in stato 0: 'inattivo in attesa'
                            self.mission_index = 0
                            self.saveMissionInfo("completed")
                    
                    # if ABORTED
                    elif self.navigation_status == mappa_move_base_status[4]: 
                        # Se il planner non trova un percorso valido
                        # Se il planner non riesce a generare cmd_vel validi
                        # Se i recovery behavior non riescono a risolvere il problema
                        if self.planner_ripristinato:
                            # Controllo che il waypoint non raggiungibile non sia l'utlimo
                            if self.index < len(self.waypoints):
                                self.planner_ripristinato = False
                                rospy.loginfo("Procedo al waypoint successivo.")
                                waypoint = self.waypoints[self.mission_index]
                                goal = MoveBaseGoal()
                                goal.target_pose.header = Header()
                                goal.target_pose.header.seq = self.mission_index
                                goal.target_pose.header.stamp = rospy.Time.now()
                                goal.target_pose.header.frame_id = "map"
                                goal.target_pose.pose.position.x = waypoint[0]
                                goal.target_pose.pose.position.y = waypoint[1]
                                goal.target_pose.pose.position.z = 0.0
                                goal.target_pose.pose.orientation.w = 1.0
                                goal.target_pose.pose.orientation.x = 0.0
                                goal.target_pose.pose.orientation.y = 0.0
                                goal.target_pose.pose.orientation.z = 0.0
                                rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                                self.move_base_client.send_goal(goal)
                                self.mission_index += 1
                            else:
                                rospy.loginfo(f"Impossibile raggiungere l'ultimo waypoint, missione terminta negativamente.")
                                self.is_mission_valid = False
                                self.mission_index = 0
                        else:
                            rospy.logerr(f"Errore: impossibile raggiungere il Waypoint {self.mission_index}. Stato del planner: {self.navigation_status}")
                            self.minor_error = True
                            self.errori_minori.append(mappa_errori_minori[2]) # Errore planner
                            self.pub_abort_current_goal.publish(GoalID()) # Comando di abort
                            self.saveMissionInfo("aborted")
                            #rospy.logerr("Errore: il planner ha abortito la missione. Passo in stato errore minore")
                            # self.minor_error = True

                    # if ACTIVE
                    elif self.navigation_status == mappa_move_base_status[1]: 
                        rospy.loginfo(f"Prossimo waypoint {self.mission_index}. Planner: {self.navigation_status}.")
                        # Attendo, nel caso, che sia il planner a segnalare un suo errore da trasformare in un errore minore
                                                    
                    # if PREEMPTED
                    elif self.navigation_status == mappa_move_base_status[2]:
                        # Riprovo a inviare il waypoint
                        rospy.loginfo("Goal preempted.")
                        # Controllo se la posizione attuale è vicina al waypoint
                        # Se è vicina lo considero raggiunto e procedo al successivo
                        # Altrimenti lo invio nuovamente
                        pose = self.get_usv_pose()
                        # Calcolo la distanza da tutti i waypoint della missione
                        distanze = []
                        for wp in self.waypoints:
                            distanza = math.sqrt((pose[0]-wp[0])**2 + (pose[1]-wp[1])**2)
                            distanze.append(distanza)

                        # Trovo l'indice del waypoint più vicino
                        wp_piu_vicino = distanze.index(min(distanze))
                        waypoint = self.waypoints[wp_piu_vicino] # waypoint più vicino
                        # --------------------------------------------------
                        goal = MoveBaseGoal()
                        goal.target_pose.header = Header()
                        goal.target_pose.header.seq = self.mission_index
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.pose.position.x = self.waypoints[wp_piu_vicino][0]
                        goal.target_pose.pose.position.y = self.waypoints[wp_piu_vicino][1]
                        goal.target_pose.pose.position.z = 0.0
                        goal.target_pose.pose.orientation.w = 1.0
                        goal.target_pose.pose.orientation.x = 0.0
                        goal.target_pose.pose.orientation.y = 0.0
                        goal.target_pose.pose.orientation.z = 0.0
                        # ------------------------------------------------
                        rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                        self.move_base_client.send_goal(goal)
                        self.mission_index = wp_piu_vicino + 1 # Aggiorno l'indice del waypoint successivo
                                                                                
                    # if PENDING
                    elif self.navigation_status == mappa_move_base_status[0]: # PENDING
                        rospy.loginfo("Goal pending: in attesa che il goal precedente venga terminato...")
                        if self.pending_time is None:
                            self.pending_time = rospy.get_time()

                        if self.pending_time and (rospy.get_time() - self.pending_time) > PENDING_TIMEOUT: # Se il planner è in pending da più di due minuti
                            # Devo controllare se il planner è bloccato
                            # Devo controllare che i sensori siano attivi
                            # Devo controllare che il target sia raggiungibile 
                            rospy.logerr("Errore: goal bloccato verrà eliminato e inviato nuovamente.")
                            self.pending_time = None 
                            self.move_base_client.cancel_goal() # Cancella il goal pending, l'ultimo inviato
                            rospy.sleep(0.5)
                            rospy.loginfo(f"In attesa di cancellazione del goal {self.mission_index-1}...")
                            goal = MoveBaseGoal()
                            goal.target_pose.header = Header()
                            goal.target_pose.header.seq = self.mission_index
                            goal.target_pose.header.stamp = rospy.Time.now()
                            goal.target_pose.header.frame_id = "map"
                            goal.target_pose.pose.position.x = self.waypoints[self.mission_index-1][0]
                            goal.target_pose.pose.position.y = self.waypoints[self.mission_index-1][1]
                            goal.target_pose.pose.position.z = 0.0
                            goal.target_pose.pose.orientation.w = 1.0
                            goal.target_pose.pose.orientation.x = 0.0
                            goal.target_pose.pose.orientation.y = 0.0
                            goal.target_pose.pose.orientation.z = 0.0
                            # ------------------------------------------------
                            rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                            self.move_base_client.send_goal(goal)
                            self.mission_index = wp_piu_vicino + 1
                        else:
                            # Resto in attesa che il planner calcoli la rotta
                            pass
                    
                    # if REJECTED
                    elif self.navigation_status == mappa_move_base_status[5]:
                        # Come per le altre condizioni di errore si va in errore minore e li si decide cosa fare
                        rospy.logerr("Errore: il planner ha rifiutato la missione. Passo in stato errore minore")
                        self.minor_error = True

                    elif not self.waypoints or self.mission_index >= len(self.waypoints):
                        rospy.loginfo("Missione completata") # Decideremo in seguito se la missione è completata o se non c'è nessuna missione.
                        self.is_valid_mission = False # Torno in stato 0: inattivo in attesa
                
            self.pub_status.publish(1)
            
        # Stato 3: Controllo remoto
        elif self.usv_status == mappa_degli_stati_usv[3]:
            rospy.loginfo("STATO: controllo remoto")
            if not self.is_remote_control:
                self.last_cmd_vel_time = None
                self.jump_to_state(0) # Torno in stato 0: inattivo in attesa
                
            if self.last_cmd_vel_time is None:
                self.last_cmd_vel_time = rospy.get_time()
            else:
                # Devo prendere il tempo dell'ultimo comando ricevuto
                cmd_vel = self.vel_multiplexer()
                # Pubblico il comando di velocità
                if self.last_cmd_vel_time is None:
                    self.last_cmd_vel_time = rospy.get_time()

                if self.last_cmd_vel_time and ((rospy.get_time() - self.last_cmd_vel_time) > CMD_VEL_TIMEOUT):
                    rospy.loginfo("Timeout comandi di controllo remoto. Torno in stato inattivo in attesa.")
                    self.last_cmd_vel_time = None
                    self.is_remote_control = False
                    self.pub_vel_cmd.publish(cmd_vel)
                    
                elif self.last_cmd_vel_time and ((rospy.get_time() - self.last_cmd_vel_time) < CMD_VEL_TIMEOUT):
                    self.pub_vel_cmd.publish(cmd_vel)
                    rospy.loginfo(f"In controllo remote: {cmd_vel}")             
            self.pub_status.publish(2)

        # Stato 3: Errore minore: qui decido in quale altra modalità passare:
        # caso 1: Input degradato
        #           (teoricamente) potrei andare in uno stato di navigazione autonoma degradata in cui il sistema naviga lo stesso solo se le informazioni richieste 
        #           risultano presenti es. sensori alternativi o di back-up. Naviga con cautela velocità di manovra ridotta ridotta
        # caso 2: Input assente
        #           (teoricamente) non ho un sensore di riserva o di back-up o intero sistema non va più, es non so più la posizione. In questo caso, richiedo l'intervento
        #           dell'operatore. Vado in modalità manuale.
        # Caso 3: Autonomia insufficiente o il sistema non riesce a raggiungere il target
        #           (teoricamente) vado in modalità rientro di emergenza in cui il USV torna al primo waypoint o in un punto sicuro noto. Ovviamente solo se può navigare da solo.
        # Caso 4: Errore critico
        #           (teoricamente) Il USV non riesce a navigare (motori o timone in avaria) e va in modalità errore critico, manda un allarme per richiedere intervento esterno. 
        #           
        #           
        elif self.usv_status == mappa_degli_stati_usv[4]:
            rospy.loginfo("STATO: errore minore. Identificazione dell'errore.")
            if not self.minor_error:
                self.jump_to_state(0) # Torno in stato 0: idle (riparte il meccanismo di missione)
                
            elif self.major_error:
                self.jump_to_state(4) # Passo in 4: 'errore critico'
                
            elif self.is_remote_control:
                self.jump_to_state(2) # Passa a 2: 'controllo remoto'
                
            else:
                if "errore planner" in self.errori_minori:
                    rospy.loginfo("Errore del planner: passo in modalità controllo remoto.")
                    self.is_remote_control = True
                    self.minor_error = False
                    # self.planner_ripristinato = True
                    self.is_valid_mission = False
                    self.errori_minori.remove("errore planner")
                    
                elif "sensore fuori uso" in self.errori_minori:
                    rospy.loginfo("Errore del sensore ellipse: passo in modalità controllo remoto.")
                    self.is_remote_control = True
                    self.minor_error = False
                    self.errori_minori.remove("sensore fuori uso")
                    
                elif "motore fuori uso" in self.errori_minori:
                    self.major_error = True

                elif self.navigation_status == mappa_move_base_status[4]:
                    rospy.loginfo("Errore del local planner: Non ci sono errori che impediscano la navigazione. La missione continuerà dal waypoint successivo.")
                    self.minor_error = False
                    self.planner_ripristinato = True

                else:
                    rospy.loginfo("Errore falso: il sistema di monitoraggio del veicolo non ha rilevato alcun malfunzionamento. La missione può riprendere.")
                    self.minor_error = False

                rospy.logerr("Fermata di emergenza! Identificazione dell'errore.")
                # Qui potrei implementare una logica per tentare di risolvere il problema o attendere l'intervento umano
                # Torno in stato 0: inattivo in attesa, per ora se c'è una fermata di emergenza devo intervenire manualmente
                # Fermo l'USV
                self.pub_vel_cmd.publish(Twist()) # Il vel multiplexer dovrebbe già fare in modo che in stato di errore minore non escano comandi di velocità
                # Performo check di sistema
                #self.sistem_check()
            self.pub_status.publish(3)
        
        # Stato 5: Errore critico
        elif self.usv_status == mappa_degli_stati_usv[5]:
            rospy.loginfo("STATO: errore critico")
            if self.major_error:
                self.pub_status.publish(4)
                rospy.logerr("Errore critico! USV non risponde. Intervento richiesto.")
                
            elif not self.major_error and self.minor_error:
                self.jump_to_state(3)
                
            else:
                self.jump_to_state(0)
                
        # Navigazione in avvio: prima di passare alla navigazione vera e propria devo verificare che sia tutto ok
        # In particolare deve essere attivo: il planner, la trosformata della posa (tf) e in futuro dovrò controllare che il l'obstacle avoidance sia attivo 
        elif self.usv_status == mappa_degli_stati_usv[1]:
            # avvio l'actionclient per move_base
            rospy.loginfo("STATO: avvio planner")
            if self.is_remote_control:
                rospy.loginfo("Passaggio in modalità controllo remoto in seguito a ricezione comando.")
                self.jump_to_state(2)
                
            elif all(self.nav_system_active):
                rospy.loginfo("Planner attivo: passagio in navigazione autonoma. Calcolo del percorso...")
                self.saveMissionInfo("iniziata")
                self.jump_to_state(1)
                
            elif not self.nav_system_active[0] and self.minor_error:
                if self.is_valid_mission:
                    self.jump_to_state(3)
                    rospy.logerr("Il planner non funzionante, passare in modalità controllo rempoto.")
                    
                else:
                    self.jump_to_state(0)
                    rospy.logwarn("Condizione strana. Si torna in idle.")
                    
            elif not self.nav_system_active[1] and self.minor_error:
                if self.is_valid_mission:
                    self.jump_to_state(3)
                    rospy.logerr("Il sitema di trasformazioni coordinate non funzionante, passare in modalità controllo rempoto.")
                    
                else:
                    self.jump_to_state(0)
                    rospy.logwarn("Condizione strana. Si torna in idle.")
            
            else:
                # Controllo lo stato del planner e della tf
                if self.move_base_client is None:
                    self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                if not self.nav_system_active[0]:    
                    rospy.loginfo(f"Tentativo {self.retry_cnt[0]} di connessione al server di move_base attendere...")
                    mb_connected = self.move_base_client.wait_for_server(rospy.Duration(MB_SERVER_TIMEOUT)) 
                    if mb_connected:
                        self.nav_system_active[0] = True
                        self.retry_cnt[0] = 1
                        rospy.loginfo("Server move_base connesso.")
                    else:
                        self.nav_system_active[0] = False
                        if self.retry_cnt[0] > MAX_MBS_RETRY_CNT:
                            self.minor_error = True
                        self.retry_cnt[0] += 1
                        rospy.logwarn("Timeout: move_base non risponde.")                 
                # Attivazione fittizia per testare il passaggio dei waypoint
                self.nav_system_active[1] = True
                # Attendiamo la trasformata che navsat_transform_node usa per ancorare
                # l'origine UTM al frame 'map' (che è il nostro [0,0])
                # if not self.nav_system_active[1]:
                #     rospy.loginfo(f"Tentativo {self.retry_cnt[1]} di connessione a tf attendere...")
                #     try:
                #         self.tf_buffer.lookup_transform(FRAME_ID_MAP, FRAME_ID_UTM, rospy.Time(0), rospy.Duration(5.0))
                #         self.nav_system_active[1] = True
                #         self.retry_cnt[1] = 1
                #         rospy.loginfo("Trasformata utm -> odom trovata.")
                #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                #         self.nav_system_active[1] = False
                #         if self.retry_cnt[1] > MAX_MBS_RETRY_CNT:
                #             self.minor_error = True
                #         self.retry_cnt[1] += 1
                #         rospy.logwarn("Timeout: tf non risponde.")              
                self.result = rospy.Subscriber('/move_base/status', MoveBaseActionResult, self.nav_result_callback)

        elif self.usv_status == mappa_degli_stati_usv[6]: # Docking
            if self.is_remote_control:
                self.pub_abort_current_goal.publish(GoalID()) # Comando di abort
                self.jump_to_state(3) # Passa a 3: 'controllo remoto'
            else:
                goal = MoveBaseGoal()
                goal.target_pose.header = Header()
                goal.target_pose.header.seq = 0
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.pose.position.x = 0.0 # Presumibilmente le coordinate del punto di docking saranno note
                goal.target_pose.pose.position.y = 0.0 # Presumibilmente le coordinate del punto di docking saranno note
                goal.target_pose.pose.position.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0
                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.0
                # ------------------------------------------------
                rospy.loginfo(f"Invio del Goal ActionLib: X={waypoint[0]}, Y={waypoint[1]} m nel frame '{FRAME_ID_MAP}'")
                self.move_base_client.send_goal(goal)

            
            rospy.loginfo("STATO: docking in corso")

        # Stato non gestito
        else:
            rospy.logwarn(f"Stato non gestito: {self.usv_status}")
            self.usv_status = mappa_degli_stati_usv[0] # Torno in stato di idle
            self.jump_to_state(0)
   

def is_roscore_running_master_check():
        """Controlla se roscore è attivo tentando di connettersi al Master ROS."""
        try:
            # Tenta di creare un'istanza del Master.
            # Questo fallirà se roscore non è avviato o non è raggiungibile.
            master = Master('/rospy_master_check')
            # Tenta una semplice chiamata al Master (es. getUri).
            master.getUri()
            return True
        except MasterException as e:
            # La MasterException viene sollevata se non riesce a connettersi al Master.
            # rospy.logerr(f"roscore non è attivo. Errore: {e}")
            return False
        except Exception as e:
            # Gestisce altri possibili errori di connessione/rete.
            # rospy.logerr(f"Errore inaspettato durante il controllo di roscore: {e}")
            return False

def get_ASCII_art():
    return r"""
               .--.              .--.
             .'_\/_'._        _.'_\/_'._ 
            '. /\ /\ .'      '. /\ /\ .'
              "|| ||"          "|| ||"
               || ||            || ||
          /\   || ||  /\    /\  || ||   /\
         /  \  || || /  \  /  \ || ||  /  \
        /====\ ||_||/====\/====\||_|| /====\
       /  []  \====/  []  \  []  \====/  [] \
      /________\__/________\_____/__\________\
       /  /  \  \  /  /  \  \  /  /  \  \  /  \
      /__/____\__\/__/____\__\/__/____\__\/__\_\
         \_/  \_/    \_/  \_/    \_/  \_/  \_/
             \_/        \_/        \_/
              / \  .-----.  / \
             / _ \/  .-.  \/ _ \
            | (_) | (   ) | (_) |
             \___/   `-'   \___/
               |             |
               |  \     /    |
                \  `---'    /
                 `.______.'
                    |  |
                 ___|  |___
                /          \
               /    ❤ ❤    \
              /              \
             /                \
    """
def get_ASCII_art_sitep():
    return r"""
__          __    _____  _____  _______  ______  _____  
\ \        / /   / ____||_   _||__   __||  ____||  __ \ 
 \ \  /\  / /   | (___    | |     | |   | |__   | |__) |
  \ \/  \/ /     \___ \   | |     | |   |  __|  |  __ / 
   \  /\  /      ____) | _| |_    | |   | |____ | |  
    \/  \/      |_____/ |_____|   |_|   |______||_|  
"""

# MAIN
if __name__ == '__main__':
    waitCounter = 0
    ROSMaster = False
    while not is_roscore_running_master_check() and waitCounter < ROSTIMEOUT:
            print("In attesa del ROS core...")
            waitCounter += 1
            time.sleep(1)

    if waitCounter < ROSTIMEOUT:
        mission_controller = MissionController()
        mission_controller.start_control_node()
    else:
        print("Timeout scaduto: il ROS core non risponde.") 
        print("Programma terminato, exit code: 0")
        print("                                   ")
        print(get_ASCII_art_sitep())
    # try:
    #     mission_controller.run()
    # except rospy.ROSInterruptException:
    #     pass
