# mission file handler for USV_CAT
# Permette di ricevere e salvare file di missione in formato .GPX in un percorso specificato
# Autore: Davide Domeneghetti
# Email: d.domeneghetti@sitepitalia.it
# Versione: 1.0
# Data: 10/09/2025
# Note: Questo script richiede la libreria 'gpxpy'. Installala tramite 'pip install gpxpy'.

#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String
import gpxpy

MISSION_FILE_PATH = "/home/davide/Scrivania/MTCOM/mission_control/" # Percorso dove salvare il file di missione
DEBUG = False # Imposta a True per abilitare messaggi e percorsi di debug


class MissionFileHanlder:

    def __init__(self, mission_file_path=MISSION_FILE_PATH):
        self.mission_file_path = mission_file_path
        rospy.init_node('mission_file_handler', anonymous=True)
        rospy.Subscriber('/mission_file', String, self.callback)
        rospy.loginfo("Mission file handler avviato, in attesa di file GPX...")
        rospy.spin()

    def callback(self, msg):
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


if __name__ == '__main__':
    try:
        MissionFileHanlder()
    except rospy.ROSInterruptException:
        pass