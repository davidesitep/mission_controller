# Watchdog node
# Viene lanciato all'avvio del sistema e si occupa del monitoraggio del mission_controller.
# Verifica la presenza di vita da parte del mission_controller e in caso di mancata ricezione del segnale di "heartbeat"
# per un certo intervallo di tempo, riavvia il nodo mission_controller.
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.0
# Data: 05/02/2026

#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from threading import Timer

TIMEOUT = 15.0  # Tempo in secondi per considerare il mission_controller non responsivo, alcune operaazioni potrebbero richiedere più tempo, quindi è importante scegliere un timeout adeguato
WATCHDOG_RATE = 2.0 # in secondi 0.5Hz
MAX_MC_RESTARTS = 3  # Numero massimo di tentativi di riavvio del mission_controller prima di segnalare un errore critico

allert_map = {
    0: "Funzionante",
    1: "Allarme 1: Nessun heartbeat ricevuto",
    2: "Allarme 2: Mission Controller non responsivo, tentativo di riavvio in corso...",
    3: "Allarme 3: Errore critico, impossibile riavviare il Mission Controller"
}

class Watchdog: 
    def __init__(self):
        # Timeout massimo consentito tra due heartbeat
        self.timeout = TIMEOUT
        # Salva l'ultimo timestamp ricevuto dal heartbeat
        self.last_heartbeat_time = None
        # Livello di allerta
        self.alert_level = 0
        # Conta i tentativi di riavvio del mission_controller
        self.restart_attempts = 0

        # Si sottoscrive al topic heartbeat del mission_controller
        self.heartbeat_sub = rospy.Subscriber('/mission_controller/heartbeat', Header, self.heartbeat_callback)

        # Publica comandi di emergenza
        # Publisher comandi emergenza
        self.pub_emergency_stop = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_watchdog_status = rospy.Publisher('/watchdog/status', String, queue_size=1)
        self.pub_abort_current_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
        # Avvia un timer periodico che controlla la differenza tra il tempo attuale e l'ultimo heartbeat
        self.periodic_timer = Timer(rospy.Duration(TIMEOUT).to_sec(), self.check_heartbeat)
     
    def heartbeat_callback(self, msg):
        # Callback chiamata ogni volta che riceve un messaggio di heartbeat
        # Salva il timestamp ricevuto dal messaggio Header
        self.last_heartbeat_time = msg.stamp
        self.alert_level = 0

    def check_heartbeat(self):
        # Funzione periodica che controlla se il mission_controller è vivo
        # Se non è stato mai ricevuto un heartbeat, logga e riavvia
        if self.last_heartbeat_time is None:
            return # Primo avvio
        


        # Calcola il tempo attuale in secondi
        time_since_heartbeat = (rospy.Time.now() - self.last_heartbeat_time).to_sec()

        if  time_since_heartbeat > (self.timeout*2):
            self.alert_level = 2
            self.raise_alert(time_since_heartbeat, self.alert_level)
            if self.restart_attempts < MAX_MC_RESTARTS:
                self.attempt_recovery()
                self.restart_attempts += 1
            else:
                rospy.logerr("Impossibile riavviare il Mission Controller numero massimo di tentativi raggiunto.")
                rospy.logwarn("Verifica dello stato del ROS Maser...")
                self.alert_level = 3
                self.raise_alert(time_since_heartbeat, self.alert_level)
                self.emergency_stop()

        elif time_since_heartbeat > self.timeout:
            self.alert_level = 1
            self.raise_alert(time_since_heartbeat, self.alert_level)
        
    def raise_alert(self, time_since_heartbeat, alert_level):
        rospy.logwarn(f"Mission Controller non risponde da {time_since_heartbeat:.1f}s, livello di allerta {alert_level}: {allert_map[alert_level]}")
        # self.pub_watchdog_status.publish("WARNING")
    
    def attempt_recovery(self):
        rospy.logwarn("Tentativo restart Mission Controller...")
        os.system("rosnode kill /mission_controller")
        os.system("roslaunch mission_control mission_controller.launch")
    
    def emergency_stop(self):
        rospy.logfatal("EMERGENCY STOP - Mission Controller non recuperabile")
        # Ferma motori
        self.pub_abort_current_goal.publish(GoalID()) # Comando di abort
        self.pub_emergency_stop.publish(Twist())
        # Pubblica stato critico
        self.pub_watchdog_status.publish("CRITICAL")
                

if __name__ == '__main__':
    try:
        watchdog = Watchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass