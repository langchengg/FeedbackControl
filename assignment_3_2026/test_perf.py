import matplotlib
matplotlib.use('Agg')
import pybullet as p
import pybullet_data
import time
import numpy as np
from src.tello_controller import TelloController
import controller
import importlib

def run_sim_test(pos_kp, pos_ki, pos_kd, int_lim, vel_kp):
    
    # Override controller gains
    controller.POS_KP = pos_kp
    controller.POS_KI = pos_ki
    controller.POS_KD = pos_kd
    controller.pid_pos_x.kp = pos_kp
    controller.pid_pos_x.ki = pos_ki
    controller.pid_pos_x.kd = pos_kd
    controller.pid_pos_x.integral_limit = int_lim
    
    controller.pid_pos_y.kp = pos_kp
    controller.pid_pos_y.ki = pos_ki
    controller.pid_pos_y.kd = pos_kd
    controller.pid_pos_y.integral_limit = int_lim
    
    controller.pid_pos_z.kp = pos_kp
    controller.pid_pos_z.ki = pos_ki
    controller.pid_pos_z.kd = pos_kd
    controller.pid_pos_z.integral_limit = int_lim
    
    controller.pid_vel_x.kp = vel_kp
    controller.pid_vel_y.kp = vel_kp
    controller.pid_vel_z.kp = vel_kp
    
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    
    start_pos = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    drone_id = p.loadURDF("resources/tello.urdf", start_pos, start_orientation)
    
    tello_ctrl = TelloController(9.81, 0.088, 0.06, 0.35, 0.566e-5, 0.762e-7)
    
    timestep = 1.0 / 1000
    pos_control_timestep = 1.0 / 50
    steps_between_pos_control = int(round(pos_control_timestep / timestep))
    
    np.random.seed()
    # Random target between -4 and 4 as per rubric
    tx = np.random.uniform(-4, 4)
    ty = np.random.uniform(-4, 4)
    tz = np.random.uniform(1, 4)
    tyaw = np.random.uniform(-3.14, 3.14)
    target = (tx, ty, tz, tyaw)
    
    prev_rpm = np.array([0, 0, 0, 0])
    desired_vel = np.array([0, 0, 0])
    yaw_rate_setpoint = 0
    
    records = []
    
    for i in range(10000):
        pos, quat = p.getBasePositionAndOrientation(drone_id)
        lin_vel_world, ang_vel_world = p.getBaseVelocity(drone_id)
        
        roll, pitch, yaw = p.getEulerFromQuaternion(quat)
        yaw_quat = p.getQuaternionFromEuler([0, 0, yaw])
        
        _, inverted_quat = p.invertTransform([0, 0, 0], quat)
        _, inverted_quat_yaw = p.invertTransform([0, 0, 0], yaw_quat)
        
        lin_vel = p.rotateVector(inverted_quat_yaw, lin_vel_world)
        ang_vel = p.rotateVector(inverted_quat, ang_vel_world)
        
        if i % steps_between_pos_control == 0:
            state = np.concatenate((pos, [roll, pitch, yaw]))
            out = controller.controller(state, target, pos_control_timestep, False)
            desired_vel = np.array(out[:3])
            yaw_rate_setpoint = out[3]
            
            # Record last 5 seconds (5000 to 10000 ms)
            if i > 5000:
                err_x = target[0] - pos[0]
                err_y = target[1] - pos[1]
                err_z = target[2] - pos[2]
                err_yaw = target[3] - yaw
                if err_yaw > np.pi: err_yaw -= 2*np.pi
                if err_yaw < -np.pi: err_yaw += 2*np.pi
                records.append([err_x, err_y, err_z, err_yaw])
                
        rpm = tello_ctrl.compute_control(desired_vel, np.array(lin_vel), quat, np.array(ang_vel), yaw_rate_setpoint, timestep)
        rpm_derivative = (rpm - prev_rpm) / 0.0163
        real_rpm = prev_rpm + rpm_derivative * timestep
        prev_rpm = real_rpm
        
        omega = real_rpm * (2 * np.pi / 60)
        omega_squared = omega**2
        motor_forces = omega_squared * 0.566e-5
        thrust = np.array([0, 0, np.sum(motor_forces)])
        
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        vel_body = np.dot(rotation.T, lin_vel_world)
        drag_body = -np.array([3.365e-2, 3.365e-2, 3.365e-2]) * vel_body
        
        force = drag_body + thrust
        
        z_torques = omega_squared * 0.762e-7
        z_torque = -z_torques[0] - z_torques[1] + z_torques[2] + z_torques[3]
        x_torque = (-motor_forces[0] + motor_forces[1] + motor_forces[2] - motor_forces[3]) * 0.06
        y_torque = (-motor_forces[0] + motor_forces[1] - motor_forces[2] + motor_forces[3]) * 0.06
        torques = np.array([x_torque, y_torque, z_torque])
        
        p.applyExternalForce(drone_id, -1, force, [0, 0, 0], p.LINK_FRAME)
        p.applyExternalTorque(drone_id, -1, torques, p.LINK_FRAME)
        
        for joint_index in range(4):
            rad_s = real_rpm[joint_index] * (2.0 * np.pi / 60.0)
            current_angle = p.getJointState(drone_id, joint_index)[0]
            new_angle = current_angle + rad_s * timestep
            p.resetJointState(drone_id, joint_index, new_angle)
            
        p.stepSimulation()
        
    p.disconnect()
    
    records = np.array(records)
    mean_err = np.mean(np.abs(records), axis=0)
    std_err = np.std(records, axis=0)
    
    score = np.sum(mean_err[:3]) + np.sum(std_err[:3])
    
    return mean_err, std_err, score

if __name__ == "__main__":
    candidates = [
        # kp, ki, kd, int_lim, vel_kp
        (1.2, 0.4, 0.6, 0.5, 0.0), # No inner loop
        (1.5, 0.5, 0.8, 0.3, 0.0),
        (1.0, 0.3, 0.4, 0.5, 0.05),
        (1.8, 0.6, 0.9, 0.2, 0.0),
        (2.0, 0.8, 1.0, 0.2, 0.0),
        (2.5, 1.0, 1.2, 0.2, 0.0)
    ]
    
    best_score = float('inf')
    best_params = None
    best_metrics = None
    
    for params in candidates:
        importlib.reload(controller)
        m, s, score = run_sim_test(*params)
        print(f"Params: {params}")
        print(f"  Mean: {m}")
        print(f"  Std:  {s}")
        print(f"  Score: {score:.4f}")
        
        # Check constraints
        if np.max(m[:3]) < 0.01 and np.max(s[:3]) < 0.01 and m[3] < 0.01 and s[3] < 0.001:
            print("  >>> PASSED CONSTRAINTS <<<")
            
        if score < best_score:
            best_score = score
            best_params = params
            best_metrics = (m, s)
            
    print("\nBest params:", best_params, "Score:", best_score)
