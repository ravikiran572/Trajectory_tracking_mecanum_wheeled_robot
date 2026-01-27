# -*- coding: utf-8 -*-
import os
import time
import math
import csv
import threading
from datetime import datetime
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator
from cflib.crazyflie.log import LogConfig

# === CONFIGURATION ===
URI1 = 'radio://0/80/2M/E7E7E7E703'  # Drone 1
URI2 = 'radio://0/80/2M/E7E7E7E704'  # Drone 2
URI3 = 'radio://0/80/2M/E7E7E7E706'  # Drone 3
LOG_DIR = r"C:\\Users\\abhin\\Desktop\\intern_fault_tolerant\\data_logs"
os.makedirs(LOG_DIR, exist_ok=True)

# === TRAJECTORY PARAMETERS ===
Z0 = 0.8       # base altitude
DURATION = 20  # total flight time in seconds
POINTS = 1320  # number of trajectory steps

# Shared state for coordination
shared_states = {
    "drone1": {"x": 0.0, "y": 0.0},
    "drone2": {"x": 0.0, "y": 0.0},
    "drone3": {"x": 0.0, "y": 0.0},
}

# === TAKEOFF & LANDING ===
def smooth_takeoff(cf, target_z, duration=3.0, steps=60):
    for i in range(steps):
        z = (i + 1) / steps * target_z
        cf.commander.send_position_setpoint(0, 0, z, 0)
        time.sleep(duration / steps)

def land(cf, x, y,from_z, duration=2.0, steps=40):
    for i in range(steps):
        z = from_z - (i + 1) / steps * from_z
        cf.commander.send_position_setpoint(x, y, max(z, 0), 0)
        time.sleep(duration / steps) 
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

# === LOGGING SETUP ===
def make_logger(logged_data, drone_name):
    latest = {}

    def log_cb_1(timestamp, data, logconf):
        latest.update({
            "timestamp": timestamp,
            "x": data["stateEstimate.x"],
            "y": data["stateEstimate.y"],
            "z": data["stateEstimate.z"],
            "roll": data["stateEstimate.roll"],
            "pitch": data["stateEstimate.pitch"],
            "yaw": data["stateEstimate.yaw"]
        })
        shared_states[drone_name]["x"] = latest["x"]
        shared_states[drone_name]["y"] = latest["y"]
        write_row()

    def log_cb_2(timestamp, data, logconf):
        latest.update(data)
        write_row()

    def write_row():
        try:
            row = [
                latest["timestamp"],
                latest["x"], latest["y"], latest["z"],
                latest["stateEstimate.vx"], latest["stateEstimate.vy"], latest["stateEstimate.vz"],
                latest["roll"], latest["pitch"], latest["yaw"],
                latest["gyro.x"], latest["gyro.y"], latest["gyro.z"],
                latest["motor.m1"], latest["motor.m2"], latest["motor.m3"], latest["motor.m4"]
            ]
            logged_data.append(row)
        except KeyError:
            pass

    return log_cb_1, log_cb_2

def start_logging(cf, cb1, cb2):
    log1 = LogConfig(name="Pose", period_in_ms=100)
    for var in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"]:
        log1.add_variable(var, "float")
    log1.data_received_cb.add_callback(cb1)
    cf.log.add_config(log1)
    log1.start()

    log2a = LogConfig(name="Velocity", period_in_ms=100)
    for var in ["stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"]:
        log2a.add_variable(var, "float")
    log2a.data_received_cb.add_callback(cb2)
    cf.log.add_config(log2a)
    log2a.start()

    log2b = LogConfig(name="Gyro", period_in_ms=100)
    for var in ["gyro.x", "gyro.y", "gyro.z"]:
        log2b.add_variable(var, "float")
    log2b.data_received_cb.add_callback(cb2)
    cf.log.add_config(log2b)
    log2b.start()

    log3 = LogConfig(name="Motors", period_in_ms=100)
    for var in ["motor.m1", "motor.m2", "motor.m3", "motor.m4"]:
        log3.add_variable(var, "float")
    log3.data_received_cb.add_callback(cb2)
    cf.log.add_config(log3)
    log3.start()

# === FLY FUNCTION ===
def fly_trajectory_tracking(uri, name, trajectory_id):
    log_data = []
    cb1, cb2 = make_logger(log_data, name)
    reference_trajectory = []

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        cf = scf.cf
        start_logging(cf, cb1, cb2)
        time.sleep(1.0)

        smooth_takeoff(cf, Z0)
        time.sleep(1.0)

        x, y = 0.0, 0.0
        kp = 20
        kpp = -2
        Ts = DURATION / POINTS

        for i in range(POINTS):
            t = i * Ts
            if trajectory_id == 1:
                initial_x, initial_y = 0.3, 0
                hx = 0.3 * math.cos(0.5 * t)
                hy = 0.3 * math.sin(0.5 * t)
                x3, y3 = shared_states["drone3"]["x"]+initial_x, shared_states["drone3"]["y"]+initial_y
                hx3 = 0.3 * math.cos(0.5 * t + 4*math.pi/3)
                hy3 = 0.3 * math.sin(0.5 * t + 4*math.pi/3)
                vx = kp * (hx - x) + kpp * ((x3 - hx3) - (x - hx))
                vy = kp * (hy - y) + kpp * ((y3 - hy3) - (y - hy))
                z = Z0
            else if trajectory_id == 2:
                initial_x, initial_y = -0.15, 0.2598
                hx = 0.3 * math.cos(0.5 * t + 2*math.pi/3)
                hy = 0.3 * math.sin(0.5 * t + 2*math.pi/3)
                x1, y1 = shared_states["drone1"]["x"]+initial_x, shared_states["drone1"]["y"]+initial_y
                hx1 = 0.3 * math.cos(0.5 * t)
                hy1 = 0.3 * math.sin(0.5 * t)
                vx = kp * (hx - x) + kpp * ((x1 - hx1) - (x - hx))
                vy = kp * (hy - y) + kpp * ((y1 - hy1) - (y - hy))
                z = Z0 + 0.4
            else:
                initial_x, initial_y =  -0.15, -0.2598
                hx = 0.3 * math.cos(0.5 * t + 4*math.pi/3)
                hy = 0.3 * math.sin(0.5 * t + 4*math.pi/3)
                x2, y2 = shared_states["drone2"]["x"]+initial_x, shared_states["drone2"]["y"]+initial_y
                hx2 = 0.3 * math.cos(0.5 * t + 2*math.pi/3)
                hy2 = 0.3 * math.sin(0.5 * t + 2*math.pi/3)
                vx = kp * (hx - x) + kpp * ((x2 - hx2) - (x - hx))
                vy = kp * (hy - y) + kpp * ((y2 - hy2) - (y - hy))
                z = Z0 + 0.2

            x += vx * Ts
            y += vy * Ts
            

            cf.commander.send_position_setpoint(x - initial_x, y - initial_y, z, 0)
            reference_trajectory.append((t, hx, hy))
            time.sleep(Ts)

        cf.commander.send_position_setpoint(x-initial_x, y - initial_y, Z0, 0)
        time.sleep(1.0)
        land(cf, x-initial_x,y - initial_y, Z0)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filepath = os.path.join(LOG_DIR, f"{name}_trajectory_log_{timestamp}.csv")
    with open(filepath, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "X", "Y", "Z", "VelX", "VelY", "VelZ", "Roll", "Pitch", "Yaw",
                         "GyroX", "GyroY", "GyroZ", "Motor1", "Motor2", "Motor3", "Motor4"])
        writer.writerows(log_data)
    print(f"✅ {name} log saved to {filepath}")

# === MAIN ===
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    thread1 = threading.Thread(target=fly_trajectory_tracking, args=(URI1, "drone1", 1))
    thread2 = threading.Thread(target=fly_trajectory_tracking, args=(URI2, "drone2", 2))
    thread3 = threading.Thread(target=fly_trajectory_tracking, args=(URI3, "drone3", 3))

    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()
    
    print("\n🎯 Both drones completed their coordinated trajectory flights!")
