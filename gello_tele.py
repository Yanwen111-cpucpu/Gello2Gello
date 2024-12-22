import arm_control_sim
import dxl_write
import time

def master2slave_angles(slave_motors,master_angles):
    for index in range(6):
        slave_motors[index].set_goal_position(master_angles[index])

def disable_all_motors(slave_motors):
    for motor in slave_motors:
        motor.stop_motor()
    print("All motors disabled.")


def main():
    master=arm_control_sim.DXL_Arm()
    init_master_angles=master.get_joint_angle() #7*1
    slave_motors=[]
    slave_init_angles=[]
    for dxl_id in range(8,14):
        slave_motors.append(dxl_write.DynamixelMotor(port='/dev/ttyUSB0', motor_id=dxl_id))
        slave_init_angles.append(slave_motors[dxl_id-8].get_position())
        # 设置工作模式为 Current-Based Position Control
        slave_motors[dxl_id-8].set_mode(5)

        slave_motors[dxl_id-8].set_max_current(400)

        slave_motors[dxl_id-8].enable_torque()

        
    try:
        while True:      
            master_angles = master.get_joint_angle()[:6] - init_master_angles[:6] + slave_init_angles  # Calibration
            master2slave_angles(slave_motors, master_angles)
            time.sleep(0.05)
            print(master_angles)
    except KeyboardInterrupt:
        print("Program interrupted by user.")
        disable_all_motors(slave_motors)

if __name__== "__main__":
    main()
