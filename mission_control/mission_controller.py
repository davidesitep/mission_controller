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
from std_msgs.msg import String
from std_msgs.msg import Int32, Int16
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult


DEBUG = True
MISSION_DIRECTORY_PATH = "/home/davide/Scrivania/MTCOM/mission_control"
CMD_VEL_TIMEOUT = 300 # Tempo in secondi (5 minuti) dopo il quale se non ricevo comandi di velocità torno in stato 0: inattivo in attesa

mappa_degli_stati_usv = {
    0: "inattivo in attesa",
    1: "in navigazione",
    2: "controllo remoto",
    3: "errore minore", # il planner non è riuscito a calcolare una rotta per raggiungere il waypoint
    4: "errore critico" # usv non risponde
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
    2: "errore planner"
}

mappa_errore_sensori = {
    0: "IMU fuori uso",
    1: "GPS fuori uso",
    2: "Bussola fuori uso",
    3: "Impossibile calcolare l'odometria"
}

class MissionController:
    def __init__(self):
        # Variabili interne
        self.mission_files = None
        self.waypoints = None
        self.mission_directory = MISSION_DIRECTORY_PATH
        self.usv_status = mappa_degli_stati_usv[0] # stato iniziale inattivo in attesa
        self.navigation_status = GoalStatusArray()
        self.mission_index = 0
        self.preemption_counter = 0
        # Variabili di stato
        self.is_valid_mission = False
        self.is_remote_control = False
        self.cc_cmd_vel = Twist()
        self.move_base_cmd_vel = Twist()
        self.minor_error = False
        self.major_error = False
        self.system_status = Int16
        # Publisher e Subscriber
        self.pub_waypoint = rospy.Publisher('/mission_executor/waipoint', String, String, queue_size=10)
        self.pub_status = rospy.Publisher('/usv_status', Int32, queue_size=10)
        self.pub_vel_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_cc_cmd_vel = rospy.Subscriber('/cc/cmd_vel', Twist, self.cmd_vel_cc_callback)
        self.sub_move_base_cmd_vel = rospy.Subscriber('/move_base/cmd_vel', Twist, self.cmd_vel_move_base_callback)
        self.sub_goal_reached = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_reached_callback)
        self.goal_feedback = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.goal_feedback_callback)
        self.sub_diagnostic_status = rospy.Subscriber('/diagnostic/status', Int16, self.diagnostic_status_callback)
        self.result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.last_cmd_vel_time = None
        self.last_cmd_vel_time_move_base = None
        self.pending_time = None
        rospy.init_node('mission_controller', anonymous=True)
        self.run()
        rospy.spin()

    # Funzioni interne
    def jump_to_state(self, state):
        if state in mappa_degli_stati_usv:
            self.usv_status = state
        else:
            rospy.logerr(f"Stato {mappa_degli_stati_usv[state]} non valido.")

    def search_for_mission(self, mission_directory):

        #Cerca nella cartella specificata se ci sono file GPX
        file_gpx = []
        if not os.path.isdir(mission_directory):
            rospy.logerr(f"Directory {mission_directory} non trovata.")
            return None
        
        file_gpx = glob.glob(os.path.join(mission_directory, '*.gpx'))
        
        return file_gpx
    
    def check_last_mission(self, mission_files):
        #Controlla tra i file GPX trovati quale è il più recente
        if not mission_files:
            rospy.loginfo("Nessun file GPX trovato.")
            return None
        
        latest_file = max(mission_files, key=os.path.getctime)
        return latest_file
    
    def load_mission(self, mission_path):
        # Carica la missione dal file GPX
        try:
            rospy.loginfo(f"Caricamento della missione da {mission_path}...")
            with open(mission_path, 'r') as file:
                gpx_content = file.read()
                gpx = gpxpy.parse(gpx_content)
                self.waypoints = [(point.latitude, point.longitude) for track in gpx.tracks for segment in track.segments for point in segment.points]
                if DEBUG:
                    rospy.loginfo(f"Missione caricata correttamente con {len(self.waypoints)} waypoints.")
                return len(self.waypoints)
            
        except Exception as e:
            rospy.logerr(f"Errore nel caricamento della missione: {e}")
            return  0
        
    def cmd_vel_cc_callback(self, msg_twist):
        # Callback per il topic /cmd_vel per proveniente dal controllo remoto
        self.cc_cmd_vel = msg_twist
        self.last_cmd_vel_time = rospy.get_time()

    def cmd_vel_move_base_callback(self, msg_twist):
        # Callback per il topic /move_base/cmd_vel
        self.move_base_cmd_vel = msg_twist
        self.last_cmd_vel_time_move_base = rospy.get_time()

    def diagnostic_status_callback(self, msg):
        self.system_status = msg.data


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

    # Callback per il topic /move_base/status per rilevare lo stato delle navigazione
    def goal_reached_callback(self, msg):
        if not msg.status_list:
            return        
        latest_status = msg.status_list[-1].status
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

    def goal_feedback_callback(self, msg):
        pass
    def result_callback(self, msg):
        pass
    
    # Funzione che controlla lo stato della sensoristica di bordo
    def decode_system_status(self, system_status):
        if self.is_set(system_status.data, 0): # Imu: in imeout
            pass
        elif self.is_set(system_status.data, 1): # Imu: accelerazioni sospette
            pass
        elif self.is_set(system_status.data, 2): # Imu: no data
            pass
        elif self.is_set(system_status.data, 3): # GPS: in timeout
            pass
        elif self.is_set(system_status.data, 4): # GPS: no data
            pass
        elif self.is_set(system_status.data, 5): # GPS: few satellite
            pass
        elif self.is_set(system_status.data, 6): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 7): # Motor: fix type non valido
            pass
        elif self.is_set(system_status.data, 8): # Motor: fix type non valido
            pass
        elif self.is_set(system_status.data, 9): # Motor: fix type non valido
            pass
        elif self.is_set(system_status.data, 10): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 11): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 12): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 13): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 14): # GPS: fix type non valido
            pass
        elif self.is_set(system_status.data, 15): # GPS: fix type non valido
            pass

    def is_set(val, i):
        if (val >> i) & 0x01:
            return True
        else:
            return False
        
    # Rappresenta il ciclo principale del mission controller    
    def run(self):
        rospy.loginfo("Mission controller avviato.")

        # Macchina a stati del mission controller
        while not rospy.is_shutdown():
                      
            # Stato 0: Inattivo in attesa
            if self.usv_status == mappa_degli_stati_usv[0]:
                # Leggo stati
                if self.is_remote_control:
                    self.jump_to_state(2) # Passa a 'controllo remoto'
                    continue
                elif self.is_valid_mission and not self.is_remote_control:
                    self.jump_to_state(1) # Passa a 'in navigazione'
                    continue
                else:                    
                    # Cerca una missione
                    self.mission_files = self.search_for_mission(self.mission_directory)
                    if self.mission_files:
                        # Se trova delle missioni carica la più recente
                        last_mission = self.check_last_mission(self.mission_files)
                        if last_mission:
                            if self.load_mission(last_mission):
                                self.is_valid_mission = True
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
                rospy.loginfo("IDLE: in attesa di una missione o di comandi di controllo remoto...")

            # Stato 1: In navigazione
            elif self.usv_status == mappa_degli_stati_usv[1]:
                
                if self.is_remote_control:
                    self.jump_to_state(2) # Passa a 2: 'controllo remoto'
                    continue
                elif not self.is_remote_control and not self.is_valid_mission:
                    self.jump_to_state(0) # Torno in stato 0: inattivo in attesa
                    continue
                elif self.minor_error:
                    self.jump_to_state(3) # Passa a 3: 'errore minore'
                    continue
                elif self.major_error:
                    self.jump_to_state(4) # Passa a 4: 'errore critico'
                    continue
                else:
                    # Inizio della missione/navigazione
                    if self.mission_index == 0:
                        rospy.loginfo("Inizio della missione...")
                        waypoint = self.waypoints[self.mission_index]
                        self.pub_waypoint.publish(f"{waypoint[0]},{waypoint[1]}")
                    elif self.mission_index < len(self.waypoints):
                        # if SUCCEEDED
                        if self.navigation_status == mappa_move_base_status[3]: 
                            rospy.loginfo("Waypoint raggiunto.")
                            if self.mission_index < len(self.waypoints) -1:
                                rospy.loginfo("Procedo al waypoint successivo.")
                                self.mission_index += 1
                                waypoint = self.waypoints[self.mission_index]
                                self.pub_waypoint.publish(f"{waypoint[0]},{waypoint[1]}")
                            else:
                                rospy.loginfo("Ultimo waypoint raggiunto. Missione completata.")
                                self.is_valid_mission = True # Torno in stato 0: 'inattivo in attesa'
                                self.mission_index = 0
                        
                        # if ABORTED
                        elif self.navigation_status == mappa_move_base_status[4]: 
                            # Se il planner non trova un percorso valido
                            # Se il planner non riesce a generare cmd_vel validi
                            # Se i recovery behavior non riescono a risolvere il problema
                            rospy.logerr("Errore: il planner ha abortito la missione. Passo in stato errore minore")
                            self.minor_error = True

                        # if ACTIVE
                        elif self.navigation_status == mappa_move_base_status[1]: 
                            rospy.loginfo("In navigazione verso il waypoint...")
                            # Resto in attesa che il planner raggiunga il target
                            # Attendo, nel caso, che sia il planner a segnalare un suo errore da trasformare in un errore minore
                            pass
                            
                        # if PREEMPTED
                        elif self.navigation_status == mappa_move_base_status[2]:
                            # Riprovo a inviare il waypoint
                            rospy.loginfo("Riprovo a inviare il waypoint...")
                            if self.preemption_counter < 3:
                                waypoint = self.waypoints[self.mission_index]
                                self.pub_waypoint.publish(f"{waypoint[0]},{waypoint[1]}")
                                self.preemption_counter += 1
                            else:
                                rospy.logerr("Errore: il planner ha preemptato la missione troppe volte. Passo in stato di fermata di emergenza.")
                                self.preemption_counter = 0
                                self.jump_to_state(3) # Passa a 3: 'errore minore'
                                continue
                        
                        # if PENDING
                        elif self.navigation_status == mappa_move_base_status[0]: # PENDING
                            rospy.loginfo("In attesa che il planner calcolila rotta...")
                            if self.pending_time is None:
                                self.pending_time = rospy.get_time()

                            if self.pending_time and (rospy.get_time() - self.pending_time) > 300: # Se il planner è in pending da più di due minuti
                                # Devo controllare se il planner è bloccato
                                # Devo controllare che i sensori siano attivi
                                # Devo controllare che il target sia raggiungibile 
                                rospy.logerr("Errore: il planner non riesce a calcolare una rotta. Passo in stato di fermata di emergenza.")
                                self.pending_time = None 
                                self.jump_to_state(3) # Passa a 3: 'errore minore'
                                continue
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
                rospy.loginfo("In navigazione...")

            # Stato 2: Controllo remoto
            elif self.usv_status == mappa_degli_stati_usv[2]:
                
                if not self.is_remote_control:
                    self.last_cmd_vel_time = None
                    self.jump_to_state(0) # Torno in stato 0: inattivo in attesa
                    continue

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
                        self.pub_vel_cmd.publish(cmd_vel)
                        self.jump_to_state(0) # Torno in stato 0: inattivo in attesa
                        continue
                    elif self.last_cmd_vel_time and ((rospy.get_time() - self.last_cmd_vel_time) < CMD_VEL_TIMEOUT):
                        self.pub_vel_cmd.publish(cmd_vel)
                        rospy.loginfo("In controllo remoto...")             
                self.pub_status.publish(2)
                rospy.loginfo("In controllo remoto...")

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
            elif self.usv_status == mappa_degli_stati_usv[3]:
                if not self.minor_error:
                    self.jump_to_state(1) # Torno in stato 1: in navigazione
                    continue
                elif self.major_error:
                    self.jump_to_state(4) # Passo in 4: 'errore critico'
                    continue
                else:
                    rospy.logerr("Fermata di emergenza! Identificazione dell'errore.")
                    # Qui potrei implementare una logica per tentare di risolvere il problema o attendere l'intervento umano
                    # Torno in stato 0: inattivo in attesa, per ora se c'è una fermata di emergenza devo intervenire manualmente
                    # Fermo l'USV
                    self.pub_vel_cmd.publish(Twist()) # Il vel multiplexer dovrebbe già fare in modo che in stato di errore minore non escano comandi di velocità
                    # Performo check di sistema
                    self.sistem_check()
                self.pub_status.publish(3)
            
            # Stato 4: Errore critico
            elif self.usv_status == mappa_degli_stati_usv[4]:
                
                if self.major_error:
                    self.pub_status.publish(4)
                    rospy.logerr("Errore critico! USV non risponde. Intervento richiesto.")
                    continue

                elif not self.major_error and self.minor_error:
                    self.jump_to_state(3)
                    continue
                else:
                    self.jump_to_state(1)
                    continue
                

            # Stato non gestito
            else:
                rospy.logwarn(f"Stato non gestito: {self.usv_status}")
                self.usv_status = mappa_degli_stati_usv[0] # Torno in stato di idle
            
            rospy.sleep(1)
        

# MAIN
if __name__ == '__main__':
    mission_controller = MissionController()
    # try:
    #     mission_controller.run()
    # except rospy.ROSInterruptException:
    #     pass