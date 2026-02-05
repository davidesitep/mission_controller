# Mission External Interface Class
# In questa classe è implementata l'interfaccia tra il nodo mission_controller e l'esterno.
# Sono registrati i subscriber e le relative callback per interpretare i comandi ricevuti da una stazione di terra,
# i publisher per la pubblicazione dei dati di stato e telemetria del veicolo USV.
# Inoltre mette a disposizione delle funzioni per il calcolo di alcuni dati telemetrici, come SOG/COG, distanza dal goal, durata missione.
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.0
# Data: 15/10/2025
# Note: Questo script richiede la libreria 'gpxpy'. Installala tramite 'pip install gpxpy'.

#!/usr/bin/env python3

# IMPORTANTE: ricordarsi di aggiungere la lettura del topic della camera per streammare o decidere di leggerlo direttamente dal gateway.

import rospy
import gpxpy
import time
from collections import deque
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Int32
import math

DEBUG = False # Imposta a True per abilitare messaggi e percorsi di debug
MS_TO_KN = 1.9438
MEAN_SAMPLE = 5
USV_MEAN_VEL = 1.02 # m/s corrispondente a 2kn

class coda(deque):
        def __init__(self, lenmax):
            self.lenmax = lenmax
            self.stack = deque(maxlen=self.lenmax)

        def push(self, value):
            if len(self.stack) < self.lenmax:
                self.stack.append(value)
            else:
                self.stack.popleft()
                self.stack.append(value)

        def mean_val(self):
            return sum(self.stack)/self.lenmax

class missionExtInterface():
    def __init__(self, mission_file_path, drone_id):
        self.mission_file_path = mission_file_path
        self.prev_stamp = time.time()
        self.istant_vel = 0.0
        self.mean_vel = 0.0
        self.vel_stack = coda(MEAN_SAMPLE)
        self.hdg_stack = coda(MEAN_SAMPLE)
        self.prev_stamp = [None, None]
        # self.local_position = [None, None] # Latitudine e longitudine
        # self.global_position = [None, None] # Latitudine e longitudine
        # self.usv_status = None
        self.last_cmd_vel_time = None
        self.sub_cc_cmd_vel = rospy.Subscriber('/CC/cmd_vel', Twist, self.cmd_vel_cc_callback)
        self.sub_remote_request = rospy.Subscriber('/cc/remote_cmd', Bool, self.control_required_callback)
        self.sub_mission_file = rospy.Subscriber('/mission_file', String, self.mission_file_callback)
        self.pub_vel = rospy.Publisher(f'/drone{drone_id}/vel', Twist, queue_size=10)
        self.pub_lat = rospy.Publisher(f'/drone{drone_id}/lat', Int32, queue_size=10)
        self.pub_long = rospy.Publisher(f'/drone{drone_id}/long', Int32, queue_size=10)
        self.pub_hdg = rospy.Publisher(f'/drone{drone_id}/heading', Int32, queue_size=10)
        self.pub_status = rospy.Publisher(f'/drone{drone_id}/status', String, queue_size=10)

    # ------------ METODI DI CALLBACK ------------        
    def mission_file_callback(self, msg):
        """
        Callback che permette di ricevere e salvare ad un percorso specificato del file system il file di missione in formato .GPX.
        """
        rospy.loginfo("Ricevuto GPX, parsing in corso...")
        try:
            gpx = gpxpy.parse(msg.data)
            file_path = self.mission_file_path + gpx.tracks[0].name if gpx.tracks else 'N/A'
            file_path += '.gpx' if file_path != 'N/A' else 'mission_file.gpx'
            
            if DEBUG:
                for track in gpx.tracks:
                    for segment in track.segments:
                        for point in segment.points:
                            rospy.loginfo(f"Punto: {point.latitude}, {point.longitude}, {point.elevation}")
            else:
                with open(file_path, 'w') as f:
                    f.write(msg.data)

                rospy.loginfo(f"File di missione salvato in {file_path}")

        except Exception as e:
            rospy.logerr(f"Errore nel parsing del GPX: {e}")

    def cmd_vel_cc_callback(self, msg_twist):
        """
        Callback per la ricezione dei comandi di velocità (lungitudinale e di virata) dell'USV.
        Salva il timestamp dell'ultimo comando ricevuto.
        """
        # Callback per il topic /cmd_vel per proveniente dal controllo remoto
        self.cc_cmd_vel = msg_twist
        # Se ho eseguito init_node altrimenti get_time non funziona
        if rospy.core.is_initialized():
            self.last_cmd_vel_time = rospy.get_time() # Necessario per implementare un timeout in caso di assenza dei comandi da remoto.

    def control_required_callback(self, msg):
        """
        Callback per intercettare il messaggio di richiesta di controllo remoto.
        """
        self.is_remote_control = msg.data
    # ----------------------------------------------------------------

    # Di fatto il body della classe, è quel metodo che sarà chiamato esternamente
    def pub_telemetry(self, usv_status ,now_pos):
        """
        Pubblica i dati di telemetria dopo aver calcolato quello che manca come SOG (velocità) e COG (rotta), è la funziona chiamta esternamente alla classe.
        """
        now_stamp = time.time()
        delta_time = now_stamp - self.prev_stamp
        if usv_status == "in navigazione":
            self.elapsed_time += delta_time
        else:
            self.elapsed_time = 0.0
            
        self.get_mean_hdg(now_pos)
        self.get_mean_vel(now_pos, delta_time)

        self.prev_stamp = now_stamp

        # Pubblico gli i dati verso la stazione di terra
        self.pub_vel.publish(self.mean_vel)
        self.pub_lat.publish(self.prev_stamp[0])
        self.pub_long.publish(self.prev_stamp[1])
        self.pub_hdg.publish(self.mean_hdg)
        self.pub_status.publish(usv_status)

    # Servono a ricevere le informazioni di posizione nella classe 
    # def set_local_pose(self, local_position): # position: latitudine e longitudine
    #     """
    #     Calcola la posizione attuale dell'USV nel frame map.
    #     """
    #     self.local_position = local_position
        
    # def set_global_pos(self, global_position):
    #     """
    #     Calcola la posizione attuale dell'USV nel formato WGS84 lat/long, data la posizione nel fram locale
    #     """
    #     self.global_position = global_position

    # Funzioni per il calcolo del SOG
    def get_istant_vel(self, prev_pos, now_pos, delta_t=1):
        """
        Calcola la velocità istantanea del drone. Intervallo di pochi istanti
        """
        delta_s = self.dist(prev_pos, now_pos) 

        if delta_s == 0 or delta_t == 0:
            self.istant_vel = (0.0,0.0)
            return self.istant_vel
        
        vel_ms = (delta_s/ delta_t)
        vel_kn = vel_ms * MS_TO_KN
        self.istant_vel = (vel_ms, vel_kn)

        self.vel_stack.push(vel_ms)

        return self.istant_vel # m/s e Kn (nodi)
    
    def get_mean_vel(self, prev_pos, now_pos, delta_time=1):
        """
        Velocità media su un numero di campioni di velocità istantanee MEAN_SAMPLE
        """
        self.get_istant_vel(prev_pos, now_pos, delta_time)
        self.mean_vel = self.vel_stack.mean_val()
    
    # Funzioni per il calcolo del COG
    def get_istant_hdg(self, prev_pos, now_pos):
        """
        Calcola la direzione istantanea del veicolo in gradi in coordinate cartesiane. Intervallo di pochi istanti.
        θ=arctan2(sin(Δλ)⋅cos(ϕ2​),cos(ϕ1​)⋅sin(ϕ2​)−sin(ϕ1​)⋅cos(ϕ2​)⋅cos(Δλ))
        """

        lat1 = math.radians(prev_pos[0])
        lon1 = math.radians(prev_pos[1])
        lat2 = math.radians(now_pos[0])
        lon2 = math.radians(now_pos[1])

        delta_lon = lon2 - lon1

        # Calcolo azimuth
        num = math.sin(delta_lon) * math.cos(lat2)

        denom = math.cos(lat1) * math.sin(lat2) - math.cos(lat2) * math.cos(delta_lon)

        heading_rad = math.atan2(num, denom)

        heading_deg = math.degrees(heading_rad)
        
        # Normalizzazione a [0, 360] gradi (il Nord è 0/360, Est è 90)
        # math.degrees() restituisce un valore tra -180 e 180.
        heading = (heading_deg + 360)%360
        self.hdg_stack.push(heading)

        return heading
    
    def get_mean_hdg(self, prev_pos, now_pos):
        """
        Calcola la direzione su un intervallo temporale di MEAN_SAMPLE secondi.
        """
        self.get_istant_hdg(prev_pos, now_pos)
        self.mean_hdg = self.hdg_stack.mean_val()

    def get_istant_hdg_cart(self, prev_pos, now_pos):
        """
        Calcola la direzione istantanea del veicolo in gradi in coordinate cartesiane. Intervallo di pochi istanti.
        """
        heading = 0.0
       
        a = now_pos[0] - prev_pos[0]
        b = now_pos[1] - prev_pos[1]
        heading = math.atan2(a,b)

        return heading
       
    # Altre informazioni di telemetria utili
    def set_distance_to_goal(self, distance):
        self.distance_to_goal = distance

    def set_distance_to_finish(self, total_distance):
        self.distance_to_finish = total_distance

    def get_time_to_completion(self):
        self.time_to_completion = self.distance_to_finish/self.mean_vel 

    def get_time_to_completion(self):
        self.time_to_completion = self.distance_to_goal/self.mean_vel 

    # Da spostare???
    def dist(self, point2, point1):
        """
        Calcola la distanza tra due punti
        """
        if hasattr(point1, 'x') and hasattr(point2, 'x'):
            x1, y1 = point1.x, point1.y
            x2, y2 = point2.x, point2.y
        elif isinstance(point1, (list, tuple)) and len(point1) >= 2:
            x1, y1 = point1[0], point1[1]
            x2, y2 = point2[0], point2[1]
        else:
            raise TypeError("Formato punto non supportato. Usa [x, y] o oggetto con .x, .y")
        
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    