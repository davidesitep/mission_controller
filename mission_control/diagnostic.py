# USV status monitor
# Controllo lo stato dei sensori, dell'odometria e dei motori al fine di capire se ci sono errori di sistema 
# Autore: Davide Domeneghetti
# Email: d.domeneghetti@sitepitalia.it
# Versione: 1.0
# Data: 16/09/2025
# Note: Questo script richiede il pkg 'sbg_driver'. Installalo tramite sudo apt-get install ros-noetic-sbg-driver.

#!/usr/bin/env python3

import rospy
from typing import NamedTuple
from std_msgs.msg import Int16
from std_msgs.msg import Header
from custom_msgs.msg import SystemStatus
from sbg_driver.msg import (
    SbgImuData, SbgGpsPos, SbgEkfEuler,
    SbgEkfNav, SbgGpsVel, SbgGpsHdt,
    SbgMag, SbgAirData, SbgShipMotion,
    SbgStatusGeneral  
)

mappa_stati_error = {
    0: 'NO_ERRORE',
    1: 'SENSORE_DEGRADATO',
    3: 'SENSORE_ASSENTE',
    4: 'EMERGENCY_STOP'
}

class system_status_struct(NamedTuple):
    sensor_presence: bool
    imu_presence: bool
    gps_presence: bool
    is_gpspos: bool
    gps_type: int
    is_gpsvel: bool
    gps_vel_type: int
    is_gps_hdt: bool
    is_magnetometer: bool
    is_imu: bool
    ekf_status: int


# Classe dedicata al monitoraggio dello stato dell'USV
class USV_monitor:

    def __init__(self):
        # Timeout e soglie
        self.imu_timeout = rospy.get_param("~imu_timeout", 1.0)
        self.gps_timeout = rospy.get_param("~gps_timeout", 2.0)
        self.max_accel = rospy.get_param("~max_accel", 50.0)
        self.min_satellites = rospy.get_param("~min_satellites", 6)

        # Stato interno diagnostic node
        self.diagnostic_stato_gps = None
        self.diagnostic_stato_imu = None
        self.diagnostic_stato_ekf = None
        self.diagnostic_stato_gpsvel = None
        self.diagnostic_stato_gpshdt = None
        self.diagnostic_stato_mag = None
        self.diagnostic_stato_shipm = None
        self.diagnostic_stato_generale = None
        self.diagnostic_stato_motore = None
        self._status = system_status_struct(True, True, True, True, 0, True, 0, True, True, True, 0)

        # Subscribers
        sub_imu = rospy.Subscriber("/sbg/imu_data", SbgImuData, self.imu_callback)
        sub_gps_pos = rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.gps_callback)
        sub_ekf_euler = rospy.Subscriber("/sbg/ekf_euler", SbgEkfEuler, self.ekf_callback)
        sub_gps_vel = rospy.Subscriber("/sbg/gps_vel", SbgGpsVel, self.gps_vel_callback)
        # sub_gps_hdt = rospy.Subscriber("/sbg/gps_hdt", SbgGpsHdt, self.gpd_hdt_callback)
        sub_mag = rospy.Subscriber("/sbg/mag", SbgMag, self.mag_callback)
        # sub_air = rospy.Subscriber("/sbg/air_data", SbgAirData, self.air_callback)
        # sub_ship = rospy.Subscriber("/sbg/ship_motion", SbgShipMotion, self.ship_callback)
        sub_gen_status = rospy.Subscriber("/sbg/status_general", SbgStatusGeneral, self.general_status_callback)

        # Publisher
        self.status_pub = rospy.Publisher("/usv_status_monitor/diagnostic/status", SystemStatus, queue_size=10)
    
    # Calcolo stato del sensore ellipse
    def general_status_callback(self, msg):
        main_power = not(msg.main_power)
        imu_power = not(msg.imu_power)
        gps_power = not(msg.gps_power)

        if main_power:
            self.diagnostic_stato_generale = 0 # System off
        elif not main_power and imu_power:
            self.diagnostic_stato_generale = 1 # Imu off
        elif not main_power and gps_power:
            self.diagnostic_stato_generale = 2 # GPS off
        elif imu_power and gps_power:
            self.diagnostic_stato_generale = 3 # Both sensor off
        else:
            self.diagnostic_stato_generale = 4 # Sensor OK

    # Calcolo stato della IMU
    def imu_callback(self, msg):
        if not msg.status.imu_com:
            self.diagnostic_stato_imu = 0 # No Communication
        elif msg.status.imu_com and not msg.status.imu_status:
            self.diagnostic_stato_imu = 1 # No Calibration (poor)
        elif msg.status.imu_com and not msg.status.imu_accels_in_range:
            self.diagnostic_stato_imu = 2 # Unexpected linear acceleration (poor)
        elif msg.status.imu_com and not msg.status.imu_gyros_in_range:
            self.diagnostic_stato_imu = 3 # Unexpected angular acceleration (poor)
        elif not msg.status.imu_accels_in_range and not msg.status.imu_gyros_in_range:
            self.diagnostic_stato_imu = 4 # Unexpected acceleration (very poor)
        else:
            self.diagnostic_stato_imu = 5 # Imu OK

    # Calcolo stato del GPS
    def gps_callback(self, msg):
        
        if msg.status.status != 0:
            self.diagnostic_stato_gps = 0 # Internal error
        else:
                    
            if msg.status.type >= 6:
                self.diagnostic_stato_gps = 1 # Optimum
            elif msg.status.type in [3,5]:
                self.diagnostic_stato_gps = 2 # Good
            elif msg.status.type in [2,4]:
                self.diagnostic_stato_gps = 3 # Poor
            elif msg.status.type in [0,1]:
                self.diagnostic_stato_gps = 4 # None
            else:
                self.diagnostic_stato_gps = 4 # None
                    
    # Calcolo stato del EKF
    def ekf_callback(self, msg):
        fault_count = 0
        if not msg.status.attitude_valid:
            fault_count += 1
        if not msg.status.heading_valid:
            fault_count += 1
        if not msg.status.velocity_valid:
            fault_count += 1
        if not msg.status.position_valid:
            fault_count += 1

        if fault_count == 0:
            self.diagnostic_stato_ekf = 0 # Ekf ok
        elif fault_count == 1:
            self.diagnostic_stato_ekf = 1 # Ekf sufficient
        elif fault_count == 2:
            self.diagnostic_stato_ekf = 2 # Ekf poor
        elif fault_count == 3:
            self.diagnostic_stato_ekf = 3 # Ekf None

    # Calcolo stato gps vel
    def gps_vel_callback(self, msg):

        if msg.status.vel_status != 0:
            self.diagnostic_stato_gpsvel = 0 # Internal error
        else:
                    
            if msg.status.vel_type == 0:
                self.diagnostic_stato_gps = 1 # No valid velocity (none)
            elif msg.status.vel_type == 1:
                self.diagnostic_stato_gps = 2 # Unknown solution (very poor)
            elif msg.status.vel_type == 2:
                self.diagnostic_stato_gps = 3 # Doppler velocity (good)
            elif msg.status.vel_type == 4:
                self.diagnostic_stato_gps = 4 # Differential velocity (very good)
            else:
                self.diagnostic_stato_gps = 1 # None

    # Calcolo stato gps hdt
    def gps_hdt_callback(self, msg):
        if msg.status == 0:
            self.diagnostic_stato_gpshdt = 0 # Solution computed
        else:
            self.diagnostic_stato_gpshdt = 1 # Internal error

    # Calcolo stato magnetometro
    def mag_callback(self, msg):
        if not msg.status.mags_in_range:
            self.diagnostic_stato_mag = 0 # Mag not in range
        elif not msg.status.accels_in_range:
            self.diagnostic_stato_mag = 1 # Mag acceleration not in range
        elif not msg.status.calibration:
            self.diagnostic_stato_mag = 2 # Mag not calibrated
        else:
            self.diagnostic_stato_mag = 3 # Mag ok

    # Qui decido in che stato porre il sistema in base alle condizioni dei vari sensori
    def check_status(self):
        now = rospy.get_time()
        status = Int16()
        # ---------------- Sensor off --------------------
        if self.diagnostic_stato_generale in [0,3]:
            self._status.sensor_presence = False # Sensore spento: la navigazione può continuare solo in modalità guida remota 
        # ---------------- Imu off --------------------
        elif self.diagnostic_stato_generale == 1: # Imu off
            self._status.imu_presence = False
            self._status.gps_presence = True
            # Se non c'è la Imu devo capire cosa posso fare con il GPS
            if self.diagnostic_stato_gpsvel in [3,4]: # Velocità calcolata
                self._status.is_gpsvel = True
            else:
                self._status.is_gpsvel = False # Non è possibile calcolare la velocità del mezzo dal GPS
            
            if self.diagnostic_stato_gpshdt == 0: # hdt da Gps ok
                self._status.is_gps_hdt = True
            else:
                self._status.is_gps_hdt = False
            
            if self.diagnostic_stato_gps in [1,2,3]:
                self._status.is_gpspos = True
            else:
                self._status.is_gpspos = False
            
            self._status.gps_type = self.diagnostic_stato_gps
            self._status.gps_vel_type = self.diagnostic_stato_gpsvel

        # ---------------- Gps off --------------------
        elif self.diagnostic_stato_generale == 2:
            self._status.gps_presence = False
            self._status.imu_presence = True
            # Se gps off devo controllare solo imu e magnetometro sono in dead reckoning
            if self.diagnostic_stato_imu:
                self._status.is_imu = True
            else:
                self.is_imu = False
            
            if self.diagnostic_stato_mag in [2,3]:
                self._status.is_magnetometer = True
            else:
                self._status = False
            
        # ---------------- Both Senson on --------------------
        elif self.diagnostic_stato_generale == 4: # Sensor On
            self._status.gps_presence = True
            self._status.imu_presence = True

            if self.diagnostic_stato_gpsvel in [3,4]: # Velocità calcolata
                self._status.is_gpsvel = True
            else:
                self._status.is_gpsvel = False # Non è possibile calcolare la velocità del mezzo dal GPS
            
            if self.diagnostic_stato_gpshdt == 0: # hdt da Gps ok
                self._status.is_gps_hdt = True
            else:
                self._status.is_gps_hdt = False
            
            if self.diagnostic_stato_gps in [1,2,3]:
                self._status.is_gpspos = True
            else:
                self._status.is_gpspos = False
            
            if self.diagnostic_stato_imu:
                self._status.is_imu = True
            else:
                self.is_imu = False
            
            if self.diagnostic_stato_mag in [2,3]:
                self._status.is_magnetometer = True
            else:
                self._status = False
            
            self._status.gps_type = self.diagnostic_stato_gps
            self._status.gps_vel_type = self.diagnostic_stato_gpsvel
            
        return self._status
        
    def set_bit(self, val, i):
        val = val | (1 << i)
        return val
    
    def reset_bit(self, val, i):
        val = val & (~(1 << i))
        return val
    
    def publish_system_status(self, status_msg):
        new_status_msg = SystemStatus()
        # new_status_msg.header = Header(stamp=rospy.Time.now(), frame_id="nav_sensors_monitor")
        new_status_msg.sensor_presence = status_msg.sensor_presence
        new_status_msg.imu_presence = status_msg.imu_presence
        new_status_msg.gps_presence = status_msg.gps_presence
        new_status_msg.is_gpspos = status_msg.is_gpspos
        new_status_msg.gps_type = status_msg.gps_type
        new_status_msg.is_gpsvel = status_msg.is_gpsvel
        new_status_msg.gps_vel_type = status_msg.gps_vel_type
        new_status_msg.is_gps_hdt = status_msg.is_gps_hdt
        new_status_msg.is_magnetometer = status_msg.is_magnetometer
        new_status_msg.is_imu = status_msg.is_imu
        new_status_msg.ekf_status = status_msg.ekf_status
        self.status_pub.publish(new_status_msg)

    def spin(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            status_msg = self.check_status()
            self.publish_system_status(status_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("usv_status_monitor")
    monitor = USV_monitor()
    monitor.spin()


