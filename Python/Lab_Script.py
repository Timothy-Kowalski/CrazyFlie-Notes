import csv
import sys
import logging

from threading import Event
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import pandas as pd
import matplotlib.pyplot as plt

# Set the URI of the Crazyflie (adjust according to hardware)
uri = "radio://0/80/2M/E7E7E7E7E1"

#Change PIDs here

#attitude error controller from IMU
attitude_roll_p = 6.0
attitude_roll_i = 3.0
attitude_roll_d = 0.0

attitude_pitch_p = 6.0
attitude_pitch_i = 3.0
attitude_pitch_d = 0.0

attitude_yaw_p = 6.0
attitude_yaw_i = 1.0
attitude_yaw_d = 0.35

#angular velocity error controller
rate_roll_p = 250.0
rate_roll_i = 500.0
rate_roll_d = 2.5

rate_pitch_p = 250.0
rate_pitch_i = 500.0
rate_pitch_d = 2.5 

rate_yaw_p = 120.0
rate_yaw_i = 16.7
rate_yaw_d = 0.0

#velocity error controller
vel_x_p = 25.0
vel_x_i = 1.0 
vel_x_d = 0.0

vel_y_p = 25.0
vel_y_i = 1.0 
vel_y_d = 0.0

vel_z_p = 25.0
vel_z_i = 15.0
vel_z_d = 0.0

#position error controller
pos_x_p = 2.0
pos_x_i = 0.0 
pos_x_d = 0.0

pos_y_p = 2.0
pos_y_i = 0.0 
pos_y_d = 0.0

pos_z_p = 2.0
pos_z_i = 0.5
pos_z_d = 0.0

#DEFAULT VALUES

#attitude error controller from IMU
attitude_def_roll_p = 6.0
attitude_def_roll_i = 3.0
attitude_def_roll_d = 0.0

attitude_def_pitch_p = 6.0
attitude_def_pitch_i = 3.0
attitude_def_pitch_d = 0.0

attitude_def_yaw_p = 6.0
attitude_def_yaw_i = 1.0
attitude_def_yaw_d = 0.35

#angular velocity error controller
rate_def_roll_p = 250.0
rate_def_roll_i = 500.0
rate_def_roll_d = 2.5

rate_def_pitch_p = 250.0
rate_def_pitch_i = 500.0
rate_def_pitch_d = 2.5

rate_def_yaw_p = 120.0
rate_def_yaw_i = 16.7
rate_def_yaw_d = 0.0

#velocity error controller
vel_def_x_p = 25.0
vel_def_x_i = 1.0
vel_def_x_d = 0.0

vel_def_y_p = 25.0
vel_def_y_i = 1.0
vel_def_y_d = 0.0

vel_def_z_p = 25.0
vel_def_z_i = 15.0
vel_def_z_d = 0.0

#position error controller
pos_def_x_p = 2.0
pos_def_x_i = 0.0
pos_def_x_d = 0.0

pos_def_y_p = 2.0
pos_def_y_i = 0.0
pos_def_y_d = 0.0

pos_def_z_p = 2.0
pos_def_z_i = 0.5
pos_def_z_d = 0.0

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Open the CSV file in write mode and prepare a CSV writer.
csv_filename = "crazyflie_log.csv"
csv_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(csv_file)
# Write header row (customize headers as needed)
csv_writer.writerow(["Timestamp", "Roll", "Pitch", "Yaw", "accel_x", "accel_y", "accel_z", "velocity_x", "velocity_y", "velocity_z", "position_x", "position_y", "position_z"])

def log_callback(timestamp, data, logconf):
    """
    This callback is called each time the Crazyflie sends new log data.
    Write the timestamp and state estimates to the CSV file.
    """

    relative_time_ms = timestamp / 1000.0
    # Retrieve values; adjust variable names if they differ.
    roll = data.get("stabilizer.roll", "")
    pitch = data.get("stabilizer.pitch", "")
    yaw = data.get("stabilizer.yaw", "")
    
    accel_x = data.get("acc.x", "")
    accel_y = data.get("acc.y", "")
    accel_z = data.get("acc.z", "")

    velocity_x = data.get("stateEstimate.vx", "")
    velocity_y = data.get("stateEstimate.vy", "")
    velocity_z = data.get("stateEstimate.vz", "")

    position_x = data.get("stateEstimate.x", "")
    position_y = data.get("stateEstimate.y", "")
    position_z = data.get("stateEstimate.z", "")

    
    # Write a row to the CSV file.
    csv_writer.writerow([relative_time_ms, roll, pitch, yaw, accel_x, accel_y, accel_z, velocity_x, velocity_y, velocity_z, position_x, position_y, position_z])

    # Optionally, print to the console.
    print(f"[{relative_time_ms}] Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

def create_imu_log_config():
# Create a log configuration that includes the state estimates
    lg_full = LogConfig(name="imu", period_in_ms=30)

    lg_full.add_variable("stabilizer.roll", "float")
    lg_full.add_variable("stabilizer.pitch", "float")
    lg_full.add_variable("stabilizer.yaw", "float")

    lg_full.add_variable("acc.x", "float")
    lg_full.add_variable("acc.y", "float")
    lg_full.add_variable("acc.z", "float")

    return lg_full

def create_velocity_position_log_config():
    lg_full = LogConfig(name="velocity_position", period_in_ms=30)

    lg_full.add_variable("stateEstimate.vx", "float")
    lg_full.add_variable("stateEstimate.vy", "float")
    lg_full.add_variable("stateEstimate.vz", "float")

    lg_full.add_variable("stateEstimate.x", "float")
    lg_full.add_variable("stateEstimate.y", "float")
    lg_full.add_variable("stateEstimate.z", "float")

    return lg_full

def set_PID(cf):

    # Wait for the parameter table (TOC) to be fully downloaded.
    # This is critical as the parameter system is populated asynchronously.
    time.sleep(1)

    # Set PID gains for attitude controller
    cf.param.set_value("pid_attitude.roll_kp", attitude_roll_p)
    cf.param.set_value("pid_attitude.roll_ki", attitude_roll_i)
    cf.param.set_value("pid_attitude.roll_kd", attitude_roll_d)
    
    cf.param.set_value("pid_attitude.pitch_kp", attitude_pitch_p)
    cf.param.set_value("pid_attitude.pitch_ki", attitude_pitch_i)
    cf.param.set_value("pid_attitude.pitch_kd", attitude_pitch_d)
    
    cf.param.set_value("pid_attitude.yaw_kp", attitude_yaw_p)
    cf.param.set_value("pid_attitude.yaw_ki", attitude_yaw_i)
    cf.param.set_value("pid_attitude.yaw_kd", attitude_yaw_d)
    
    # Set PID gains for rate controller
    cf.param.set_value("pid_rate.roll_kp", rate_roll_p)
    cf.param.set_value("pid_rate.roll_ki", rate_roll_i)
    cf.param.set_value("pid_rate.roll_kd", rate_roll_d)
    
    cf.param.set_value("pid_rate.pitch_kp", rate_pitch_p)
    cf.param.set_value("pid_rate.pitch_ki", rate_pitch_i)
    cf.param.set_value("pid_rate.pitch_kd", rate_pitch_d)
    
    cf.param.set_value("pid_rate.yaw_kp", rate_yaw_p)
    cf.param.set_value("pid_rate.yaw_ki", rate_yaw_i)
    cf.param.set_value("pid_rate.yaw_kd", rate_yaw_d)

    # Set PID gains for velocity controller
    cf.param.set_value("velCtlPid.vxKp", vel_x_p)
    cf.param.set_value("velCtlPid.vxKi", vel_x_i)
    cf.param.set_value("velCtlPid.vxKd", vel_x_d)
    
    cf.param.set_value("velCtlPid.vyKp", vel_y_p)
    cf.param.set_value("velCtlPid.vyKi", vel_y_i)
    cf.param.set_value("velCtlPid.vyKd", vel_y_d)
    
    cf.param.set_value("velCtlPid.vzKp", vel_z_p)
    cf.param.set_value("velCtlPid.vzKi", vel_z_i)
    cf.param.set_value("velCtlPid.vzKd", vel_z_d)

    # Set PID gains for position controller
    cf.param.set_value("posCtlPid.xKp", pos_x_p)
    cf.param.set_value("posCtlPid.xKi", pos_x_i)
    cf.param.set_value("posCtlPid.xKd", pos_x_d)
    
    cf.param.set_value("posCtlPid.yKp", pos_y_p)
    cf.param.set_value("posCtlPid.yKi", pos_y_i)
    cf.param.set_value("posCtlPid.yKd", pos_y_d)
    
    cf.param.set_value("posCtlPid.zKp", pos_z_p)
    cf.param.set_value("posCtlPid.zKi", pos_z_i)
    cf.param.set_value("posCtlPid.zKd", pos_z_d)
    
    # Optionally, wait a while to observe the effects.
    time.sleep(1)
    
    #Read back and print the values

    print("\nAttitude Roll PID:")
    print("P =", cf.param.get_value("pid_attitude.roll_kp"))
    print("I =", cf.param.get_value("pid_attitude.roll_ki"))
    print("D =", cf.param.get_value("pid_attitude.roll_kd"))
    
    print("\nRate Roll PID:")
    print("P =", cf.param.get_value("pid_rate.roll_kp"))
    print("I =", cf.param.get_value("pid_rate.roll_ki"))
    print("D =", cf.param.get_value("pid_rate.roll_kd"))

    print("\nVel x PID:")
    print("P =", cf.param.get_value("velCtlPid.vxKp"))
    print("I =", cf.param.get_value("velCtlPid.vxKi"))
    print("D =", cf.param.get_value("velCtlPid.vxKd"))

    print("\nPos x PID:")
    print("P =", cf.param.get_value("posCtlPid.xKp"))
    print("I =", cf.param.get_value("posCtlPid.xKi"))
    print("D =", cf.param.get_value("posCtlPid.xKd"))
    
    print("\nAll PID parameters set successfully.")

def set_default_PID(cf):
    # Wait for the parameter table (TOC) to be fully downloaded.
    # This is critical as the parameter system is populated asynchronously.
    time.sleep(1)

    # Set PID gains for attitude controller
    cf.param.set_value("pid_attitude.roll_kp", attitude_def_roll_p)
    cf.param.set_value("pid_attitude.roll_ki", attitude_def_roll_i)
    cf.param.set_value("pid_attitude.roll_kd", attitude_def_roll_d)
    
    cf.param.set_value("pid_attitude.pitch_kp", attitude_def_pitch_p)
    cf.param.set_value("pid_attitude.pitch_ki", attitude_def_pitch_i)
    cf.param.set_value("pid_attitude.pitch_kd", attitude_def_pitch_d)
    
    cf.param.set_value("pid_attitude.yaw_kp", attitude_def_yaw_p)
    cf.param.set_value("pid_attitude.yaw_ki", attitude_def_yaw_i)
    cf.param.set_value("pid_attitude.yaw_kd", attitude_def_yaw_d)
    
    # Set PID gains for rate controller
    cf.param.set_value("pid_rate.roll_kp", rate_def_roll_p)
    cf.param.set_value("pid_rate.roll_ki", rate_def_roll_i)
    cf.param.set_value("pid_rate.roll_kd", rate_def_roll_d)
    
    cf.param.set_value("pid_rate.pitch_kp", rate_def_pitch_p)
    cf.param.set_value("pid_rate.pitch_ki", rate_def_pitch_i)
    cf.param.set_value("pid_rate.pitch_kd", rate_def_pitch_d)
    
    cf.param.set_value("pid_rate.yaw_kp", rate_def_yaw_p)
    cf.param.set_value("pid_rate.yaw_ki", rate_def_yaw_i)
    cf.param.set_value("pid_rate.yaw_kd", rate_def_yaw_d)

    # Set PID gains for velocity controller
    cf.param.set_value("velCtlPid.vxKp", vel_def_x_p)
    cf.param.set_value("velCtlPid.vxKi", vel_def_x_i)
    cf.param.set_value("velCtlPid.vxKd", vel_def_x_d)
    
    cf.param.set_value("velCtlPid.vyKp", vel_def_y_p)
    cf.param.set_value("velCtlPid.vyKi", vel_def_y_i)
    cf.param.set_value("velCtlPid.vyKd", vel_def_y_d)
    
    cf.param.set_value("velCtlPid.vzKp", vel_def_z_p)
    cf.param.set_value("velCtlPid.vzKi", vel_def_z_i)
    cf.param.set_value("velCtlPid.vzKd", vel_def_z_d)

    # Set PID gains for position controller
    cf.param.set_value("posCtlPid.xKp", pos_def_x_p)
    cf.param.set_value("posCtlPid.xKi", pos_def_x_i)
    cf.param.set_value("posCtlPid.xKd", pos_def_x_d)
    
    cf.param.set_value("posCtlPid.yKp", pos_def_y_p)
    cf.param.set_value("posCtlPid.yKi", pos_def_y_i)
    cf.param.set_value("posCtlPid.yKd", pos_def_y_d)
    
    cf.param.set_value("posCtlPid.zKp", pos_def_z_p)
    cf.param.set_value("posCtlPid.zKi", pos_def_z_i)
    cf.param.set_value("posCtlPid.zKd", pos_def_z_d)
    
    # Optionally, wait a while to observe the effects.
    time.sleep(1)


def fly_crazyflie(scf):
    with MotionCommander(scf, default_height=0.1) as mc:
        mc.up(0.3)
        time.sleep(1)
        print('Rolling left 1 at 1m/s')
        mc.left(1, velocity=1)
        # Wait a bit
        time.sleep(0.5)
            
        print('Moving forward 1m')
        mc.forward(1, velocity=1)
        # Wait a bit
        time.sleep(0.5)

        print('Rolling right 1m at 1/s')
        mc.right(1, velocity=1)
        # Wait a bit
        time.sleep(0.5)

        print('Moving back 1m')
        mc.back(1, velocity=1)
        # Wait a bit
        time.sleep(0.5)

        print ('Circling right')
        mc.circle_right(0.75, 1, 1080)

def plot_data():
    # Read the CSV file into a DataFrame.
    df = pd.read_csv("crazyflie_log.csv")

    # Create a figure for plotting.
    plt.figure(figsize=(10, 6))

    # Plot each desired column vs Timestamp.
    plt.plot(df['Timestamp'], df['accel_x'], label="X Acceleration", color='blue', marker='o')
    plt.plot(df['Timestamp'], df['velocity_x'], label="X Velocity", color='green', marker='s')
    plt.plot(df['Timestamp'], df['position_x'], label="X Position", color='red', marker='^')

    # Set axis labels and title.
    plt.xlabel("Time (s)")
    plt.ylabel("Measurement")
    plt.title("Crazyflie Log Data: X-Axis Measurements")

    # Enable grid and legend.
    plt.grid(True)
    plt.legend()

    # Display the plot.
    plt.show()
    
    
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Create both log configurations.
    imu_conf = create_imu_log_config()
    vel_pos_conf = create_velocity_position_log_config()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:

        #set_def_PID(scf.cf)
        set_PID(scf.cf)
        
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        # Add and start the IMU log configuration.
        scf.cf.log.add_config(imu_conf)
        imu_conf.data_received_cb.add_callback(log_callback)

        # Add and start the velocity/position log configuration.
        scf.cf.log.add_config(vel_pos_conf)
        vel_pos_conf.data_received_cb.add_callback(log_callback)
        imu_conf.start()
        vel_pos_conf.start()
        
        fly_crazyflie(scf)
        time.sleep(0.5)

        imu_conf.stop()
        vel_pos_conf.stop()

        #plot_data()
        
        csv_file.close()
        print(f"Log saved to {csv_filename}")

        
