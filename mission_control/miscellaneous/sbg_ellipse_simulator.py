# Sbg Ellipse simulator
# Simula le informazioni fornite dal sensore in particolare posizione, assetto, rotta, velocità e informazione eulerian kalman filter

#!/usr/bin/env python3
import rospy
import random
import math
from std_msgs.msg import Header
from sbg_driver.msg import (
    SbgImuData, SbgGpsPos, SbgEkfEuler,
    SbgEkfNav, SbgGpsVel, SbgGpsHdt,
    SbgMag, SbgAirData, SbgShipMotion,
    SbgStatusGeneral  
)
import PySimpleGUI as sg
sg.home()

# Variabili di stato GPS
gps_type = {
    0: "NO_SOLUTION",
    1: "UNKNOWN_TYPE",
    2: "SINGLE",
    3: "PSRDIFF", # DGPS
    4: "SBAS",
    5: "OMNISTAR",
    6: "RTK_FLOAT", # Floating RTK ambiguity solution (20 cms RTK).
    7: "RTK_INT", # Integer RTK ambiguity solution (2 cms RTK).
    8: "PPP_FLOAT", # Precise Point Positioning with float ambiguities
    9: "PPP_INT", # Precise Point Positioning with fixed ambiguities
   10: "FIXED" # Fixed location solution position
}

status_gps = {
    0: 'SOL_COMPUTED',	# A valid solution has been computed.
    1: 'INSUFFICIENT_OBS', # Not enough valid SV to compute a solution.
    2: 'INTERNAL_ERROR', # An internal error has occurred.
    3: 'HEIGHT_LIMIT' #	The height limit has been exceeded.
}

# Variabili di stato IMU
imu_status = True
imu_com = True
imu_accels_in_range = True
imu_gyros_in_range = True

# Variabili di stato EKF
ekf_solution_mode = {
    0: "UNINITIALIZED", # The Kalman filter is not initialized and the returned data are all invalid.
    1: "VERTICAL_GYRO", # The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely.
    2: "AHRS",          # A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely.
    3: "NAV_VELOCITY",  # The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation.
    4: "NAV_POSITION"   # Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided. 
}

# True if Attitude data is reliable (Roll/Pitch error < 0,5 deg)
attitude_valid = True
# True if Heading data is reliable (Heading error < 1 deg)
heading_valid = True
# True if Velocity data is reliable (velocity error < 1.5 m/s)
velocity_valid = True
# True if Position data is reliable (Position error < 10m)
position_valid = True
# True if sensor alignment and calibration parameters are valid
align_valid = True

# Variabili di stato velocità
vel_status = {
    0: "SOL_COMPUTED", # A valid solution has been computed.
    1: "INSUFFICIENT_OBS", # Not enough valid SV to compute a solution.
    2: "INTERNAL_ERROR", # An internal error has occurred.
    3: "LIMIT" # Velocity limit exceeded.
}
vel_type = {
    0: "VEL_NO_SOLUTION", # No valid velocity solution available.
    1: "VEL_UNKNOWN_TYPE", # An unknown solution type has been computed.
    2: "VEL_DOPPLER",	#		A Doppler velocity has been computed.
    3: "VEL_DIFFERENTIAL" # A velocity has been computed between two positions.
}

# Variabile di stato di Heading
# Bit 0-5: enum:
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR		An internal error has occurred.
# 3 HEIGHT_LIMIT		The height limit has been exceeded.
# Bit 6: mask:
# 1 BASELINE_VALID      The baseline length field is filled and valid.
heading_status = int()

# Variabili di stato Magnetometro
# True if magnetometer is not saturated
mags_in_range = True
# True if accelerometer is not saturated
accels_in_range = True
# True if magnetometer seems to be calibrated
calibration = True

# General status bitmask and enums
main_power = True
imu_power = True
gps_power = True
status_general = [main_power, imu_power, gps_power]




def sbg_ellipse_simulator():
    rospy.init_node("sbg_ellipse_simulator")
    
    pub_imu = rospy.Publisher("/sbg/imu_data", SbgImuData, queue_size=10)
    pub_gps_pos = rospy.Publisher("/sbg/gps_pos", SbgGpsPos, queue_size=10)
    pub_ekf_euler = rospy.Publisher("/sbg/ekf_euler", SbgEkfEuler, queue_size=10)
    pub_gps_vel = rospy.Publisher("/sbg/gps_vel", SbgGpsVel, queue_size=10)
    pub_gps_hdt = rospy.Publisher("/sbg/gps_hdt", SbgGpsHdt, queue_size=10)
    pub_mag = rospy.Publisher("/sbg/mag", SbgMag, queue_size=10)
    pub_air = rospy.Publisher("/sbg/air_data", SbgAirData, queue_size=10)
    pub_ship = rospy.Publisher("/sbg/ship_motion", SbgShipMotion, queue_size=10)
    pub_gen_status = rospy.Publisher("/sbg/status_general", SbgStatusGeneral, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # Punto iniziale (Roma come esempio)
    lat = 41.9028
    lon = 12.4964
    alt = 20.0
    heading = 90.0

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # -------------- General Status -------------
        gen_status = SbgStatusGeneral()
        gen_status.main_power = main_power
        gen_status.imu_power = imu_power
        gen_status.gps_power = gps_power
        pub_gen_status.publish(gen_status)
        # -------------------------------------------

        # ---------------- IMU ----------------
        imu = SbgImuData()
        imu.header = Header(stamp=now, frame_id="imu_link")
        imu.accel.x = random.uniform(-0.1, 0.1)
        imu.accel.y = random.uniform(-0.1, 0.1)
        imu.accel.z = 9.81 + random.uniform(-0.05, 0.05)
        imu.gyro.x = random.uniform(-0.01, 0.01)
        imu.gyro.y = random.uniform(-0.01, 0.01)
        imu.gyro.z = random.uniform(-0.01, 0.01)
        # ------------- IMU Status ------------
        imu.imu_status.imu_accels_in_range = True
        imu.imu_status.imu_gyros_in_range = True
        imu.imu_status.imu_com = True
        imu.imu_status.imu_status = True
        pub_imu.publish(imu)
        # -------------------------------------

        # ---------------- GPS Pos ----------------
        gps = SbgGpsPos()
        gps.header = Header(stamp=now, frame_id="gps_link")
        gps.latitude = lat + random.uniform(-1e-6, 1e-6)
        gps.longitude = lon + random.uniform(-1e-6, 1e-6)
        gps.altitude = alt + random.uniform(-0.1, 0.1)
        gps.num_sv_used = 10
        gps.status.status = status_gps[0]
        gps.status.type = gps_type[6]
        pub_gps_pos.publish(gps)

        # ---------------- EKF Euler ----------------
        ekf = SbgEkfEuler()
        ekf.header = Header(stamp=now, frame_id="ekf_link")
        ekf.angle.x = random.uniform(-2.0, 2.0) * math.pi/180
        ekf.angle.y = random.uniform(-2.0, 2.0) * math.pi/180
        heading += 0.01
        ekf.angle.z = math.radians(heading % 360)
        ekf.status.solution_mode = ekf_solution_mode[4]
        # --------------- EKF Status -----------------
        ekf.status.position_valid = position_valid
        ekf.status.attitude_valid = attitude_valid
        ekf.status.heading_valid = heading_valid
        ekf.status.velocity_valid = velocity_valid
        pub_ekf_euler.publish(ekf)
        # --------------------------------------------

        # ---------------- GPS Vel ----------------
        vel = SbgGpsVel()
        vel.header = Header(stamp=now, frame_id="gps_link")
        vel.velocity.x = random.uniform(-0.2, 0.2)
        vel.velocity.y = random.uniform(-0.2, 0.2)
        vel.velocity.z = random.uniform(-0.1, 0.1)
        # ------------ GPS Vel Status -------------
        vel.status.vel_type = vel_type[3]
        vel.status.vel_status = vel_status[0]
        pub_gps_vel.publish(vel)
        # -----------------------------------------

        # ---------------- GPS Heading ----------------
        hdt = SbgGpsHdt()
        hdt.header = Header(stamp=now, frame_id="gps_link")
        hdt.true_heading = (heading + random.uniform(-1, 1)) % 360
        hdt.status = 1
        pub_gps_hdt.publish(hdt)

        # ---------------- Magnetometer ----------------
        mag = SbgMag()
        mag.header = Header(stamp=now, frame_id="mag_link")
        mag.mag.x = random.uniform(-50, 50)
        mag.mag.y = random.uniform(-50, 50)
        mag.mag.z = random.uniform(-50, 50)
        # ----------------- Mag Status ------------------
        mag.status.accels_in_range = accels_in_range
        mag.status.mags_in_range = mags_in_range
        mag.status.calibration = calibration
        pub_mag.publish(mag)
        # -----------------------------------------------

        # ---------------- Air Data ----------------
        air = SbgAirData()
        air.header = Header(stamp=now, frame_id="air_link")
        air.pressure_abs = 1013.25 + random.uniform(-5, 5)
        air.air_temperature = 20.0 + random.uniform(-2, 2)
        pub_air.publish(air)

        # ---------------- Ship Motion ----------------
        ship = SbgShipMotion()
        ship.header = Header(stamp=now, frame_id="ship_link")
        ship.ship_motion.x = 0.0
        ship.ship_motion.y = random.uniform(-1.0, 1.0)
        ship.ship_motion.z = 0.0
        ship.heave_period = random.uniform(1.0, 10.0)
        pub_ship.publish(ship)

        rate.sleep()


if __name__ == "__main__":
    try:
        sbg_ellipse_simulator()
    except rospy.ROSInterruptException:
        pass

