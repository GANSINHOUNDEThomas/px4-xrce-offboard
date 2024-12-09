import rclpy
from rclpy.node import Node
import math


from tkinter import *
from px4_msgs.msg import VehicleStatus,VehicleOdometry, BatteryStatus, VehicleControlMode, FailsafeFlags,VehicleAttitude
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# A voir
# /fmu/out/vehicle_control_mode
# /fmu/out/failsafe_flags  : alerte des situations de failsafe


dic_nav_state={0:"Manuel",
                1:"Altitude",
                2:"Position",
                3:"auto mission",
                4:"auto loiter",
                5:"autoreturn to launch",
                6:"POSITION_SLOW",
                10:"Acro",
                12:"Descend",
                13:"Termination",
                14:"OffBoard",
                15:"Stabilized",
                17:"TakeOff",
                18:"Land",
                19:"Auto Follow",
                20:"Precision Land",
                21:"Orbit"
                }
dic_arm_state={0:"Inconnu",
                1:"Disarmed",
                2:"Armed"}



def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class supervision(Node):
    def __init__(self):
        super().__init__("supervision")
        self.get_logger().info("demarrage supervision")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_vehicle_status=self.create_subscription(
            VehicleStatus,"/fmu/out/vehicle_status",self.cb_vehicle_status,qos_profile)
        self.sub_batt=self.create_subscription(BatteryStatus,"/fmu/out/battery_status",self.cb_batt,qos_profile)
        self.sub_vehicle_odometry=self.create_subscription(
            VehicleOdometry,"/fmu/out/vehicle_odometry",self.cb_vehicle_odometry,qos_profile)
        self.sub_vehicle_control_mode=self.create_subscription(
            VehicleControlMode,"/fmu/out/vehicle_control_mode",self.cb_vehicle_control_mode,qos_profile)
        self.sub_failsafe_flags=self.create_subscription(
            FailsafeFlags,"/fmu/out/failsafe_flags",self.cb_failsafe_flags,qos_profile)
        self.sub_vehicle_attitude=self.create_subscription(
            VehicleAttitude,"/fmu/out/vehicle_attitude",self.cb_vehicle_attitude,qos_profile)
            

        # self.sub_Joy=self.create_subscription(Joy,"/joy",self.cb_joy,1)
        
        self.upd=self.create_timer(0.2, self.update)

        self.vehicle_status=VehicleStatus()
        self.battery_status=BatteryStatus()
        self.vehicle_odometry=VehicleOdometry()
        self.vehicle_control_mode=VehicleControlMode()
        self.failsafe_flags=FailsafeFlags()
        self.vehicle_attitude=VehicleAttitude()
        # self.vehicule_status_date=now()
        id_row=0
        self.F=Tk()
        self.F.option_add("*font","Terminal 12")
        self.F.title("Supervision")
        self.F.geometry('+10+10')

        self.T_separ0=Label(self.F,text="---- VEHICLE STATUS------")
        self.T_separ0.grid(row=id_row,column=0,sticky='NW');id_row+=1

        self.T_arm_state=Label(self.F,text="arm_state : ??              ")
        self.T_arm_state.grid(row=id_row,column=0,sticky='W');id_row+=1
        self.T_nav_state=Label(self.F,text="nav_state : ??              ")
        self.T_nav_state.grid(row=id_row,column=0,sticky='W');id_row+=1
        self.T_failsafe=Label(self.F,text="failsafe : ??              ")
        self.T_failsafe.grid(row=id_row,column=0,sticky='W');id_row+=1
        self.T_calibration=Label(self.F,text="calibration : ??              ")
        self.T_calibration.grid(row=id_row,column=0,sticky='W');id_row+=1
        self.T_preflight_check=Label(self.F,text="preflight_check : ??              ")
        self.T_preflight_check.grid(row=id_row,column=0,sticky='W');id_row+=1
        

        self.T_separ1=Label(self.F,text="---- ODOM + ATTITUDE------")
        self.T_separ1.grid(row=id_row,column=0,sticky='NW');id_row+=1
        
        self.T_odometry=Label(self.F,text="odometry: \nx:???.?? y:???.?? z:???.?? \n vx:?.?? vy:?.?? vz:?.??             ")
        self.T_odometry.grid(row=id_row,column=0,sticky='NW');id_row+=1

        self.T_separ2=Label(self.F,text="---- BATT------")
        self.T_separ2.grid(row=id_row,column=0,sticky='NW');id_row+=1
        
        self.T_battery1=Label(self.F,text="time_remaining x"+f'{self.battery_status.time_remaining_s:+7.2f}')
        self.T_battery2=Label(self.F,text="voltage "+f'{self.battery_status.voltage_v:+7.2f}')
        self.T_battery3=Label(self.F,text="current "+f'{self.battery_status.current_a:+7.2f}')
        # self.T_battery4=Label(self.F,text="temperature "+f'{self.battery_status.temperature:+7.2f}')
        self.T_battery1.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_battery2.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_battery3.grid(row=id_row,column=0,sticky='NW');id_row+=1
        # self.T_battery4.grid(row=id_row,column=0,sticky='NW');id_row+=1
        
        self.T_separ3=Label(self.F,text="---- CONTROL MODE------")
        self.T_separ2.grid(row=id_row,column=0,sticky='NW');id_row+=1

        self.T_control_mode1=Label(self.F,text="flag_armed")
        self.T_control_mode2=Label(self.F,text="control_manual:"+str(self.vehicle_control_mode.flag_control_manual_enabled))
        self.T_control_mode2=Label(self.F,text="control_auto:"+str(self.vehicle_control_mode.flag_control_auto_enabled))
        self.T_control_mode3=Label(self.F,text="control_offboard:"+str(self.vehicle_control_mode.flag_control_offboard_enabled))
        self.T_control_mode4=Label(self.F,text="control_pos:"+str(self.vehicle_control_mode.flag_control_position_enabled))
        self.T_control_mode5=Label(self.F,text="control_vel:"+str(self.vehicle_control_mode.flag_control_velocity_enabled))
        self.T_control_mode6=Label(self.F,text="control_altitude:"+str(self.vehicle_control_mode.flag_control_altitude_enabled))
        self.T_control_mode7=Label(self.F,text="control_climb_rate:"+str(self.vehicle_control_mode.flag_control_climb_rate_enabled))
        self.T_control_mode8=Label(self.F,text="control_acceleration:"+str(self.vehicle_control_mode.flag_control_acceleration_enabled))
        self.T_control_mode9=Label(self.F,text="control_attitude:"+str(self.vehicle_control_mode.flag_control_attitude_enabled))
        self.T_control_mode10=Label(self.F,text="control_rates:"+str(self.vehicle_control_mode.flag_control_rates_enabled))
        self.T_control_mode11=Label(self.F,text="control_termination:"+str(self.vehicle_control_mode.flag_control_termination_enabled))
        self.T_control_mode1.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode2.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode3.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode4.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode5.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode6.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode7.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode8.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode9.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode10.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_control_mode11.grid(row=id_row,column=0,sticky='NW');id_row+=1
        
        #Failsafe Flags
        self.T_separ4=Label(self.F,text="---- FAILSAFE------")
        self.T_separ4.grid(row=id_row,column=0,sticky='NW');id_row+=1

        self.T_failsafe1=Label(self.F,text="battery_warning"+str(self.failsafe_flags.battery_warning))
        self.T_failsafe2=Label(self.F,text="battery_low_remaining_time"+str(self.failsafe_flags.battery_low_remaining_time))
        self.T_failsafe1.grid(row=id_row,column=0,sticky='NW');id_row+=1
        self.T_failsafe2.grid(row=id_row,column=0,sticky='NW');id_row+=1
        

        # lance le premier affichage
        self.F.update_idletasks()
        self.F.update()


    def update(self):
        # self.get_logger().info("update")
        #NAV_STATUS
        if self.vehicle_status.nav_state in dic_nav_state:
            self.T_nav_state.config(text="nav_state : "+dic_nav_state[self.vehicle_status.nav_state])
        else:
            self.get_logger().info("nav_state : INCONNU : "+str(self.vehicle_status.nav_state))
        #ARM_STATE
        self.T_arm_state.config(text="arm_state : "+dic_arm_state[self.vehicle_status.arming_state])
        self.T_failsafe.config(text="failsafe : "+str(self.vehicle_status.failsafe))
        self.T_calibration.config(text="calibration : "+str(self.vehicle_status.calibration_enabled))
        self.T_preflight_check.config(text="preflight_check : "+str(self.vehicle_status.pre_flight_checks_pass))
        
        #ODOMETRY
        # roll, pitch, yaw=euler_from_quaternion(self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3])
        roll, pitch, yaw=euler_from_quaternion(self.vehicle_attitude.q[0], self.vehicle_attitude.q[1], self.vehicle_attitude.q[2],self.vehicle_attitude.q[3])
        
        self.T_odometry.config(text="odometry: heading(deg):"+f'{yaw*180.0/math.pi:+7.2f}'+"     "+
                    "\nx:"+f'{self.vehicle_odometry.position[0]:+7.2f}'+
                    " y:"+f'{self.vehicle_odometry.position[1]:+7.2f}'+
                    " z:"+f'{self.vehicle_odometry.position[2]:+7.2f}'+" \n"+
                    "vx: "+f'{self.vehicle_odometry.velocity[0]:+5.2f}'+
                    " vy: "+f'{self.vehicle_odometry.velocity[1]:+5.2f}'+
                    " vz: "+f'{self.vehicle_odometry.velocity[2]:+5.2f}')
        # Battery
        self.T_battery1.config(text="time_remaining "+f'{self.battery_status.time_remaining_s:+7.2f}')
        self.T_battery2.config(text="voltage "+f'{self.battery_status.voltage_v:+7.2f}')
        self.T_battery3.config(text="current "+f'{self.battery_status.current_a:+7.2f}')
        # self.T_battery4.config(text="temperature "+f'{self.battery_status.temperature:+7.2f}')

        #Control Mode
        self.T_control_mode1.config(text="armed:"+str(self.vehicle_control_mode.flag_armed))
        self.T_control_mode2.config(text="control_manual:"+str(self.vehicle_control_mode.flag_control_manual_enabled))
        self.T_control_mode2.config(text="control_auto:"+str(self.vehicle_control_mode.flag_control_auto_enabled))
        self.T_control_mode3.config(text="control_offboard:"+str(self.vehicle_control_mode.flag_control_offboard_enabled))
        self.T_control_mode4.config(text="control_pos:"+str(self.vehicle_control_mode.flag_control_position_enabled))
        self.T_control_mode5.config(text="control_vel:"+str(self.vehicle_control_mode.flag_control_velocity_enabled))
        self.T_control_mode6.config(text="control_altitude:"+str(self.vehicle_control_mode.flag_control_altitude_enabled))
        self.T_control_mode7.config(text="control_climb_rate:"+str(self.vehicle_control_mode.flag_control_climb_rate_enabled))
        self.T_control_mode8.config(text="control_acceleration:"+str(self.vehicle_control_mode.flag_control_acceleration_enabled))
        self.T_control_mode9.config(text="control_attitude:"+str(self.vehicle_control_mode.flag_control_attitude_enabled))
        self.T_control_mode10.config(text="control_rates:"+str(self.vehicle_control_mode.flag_control_rates_enabled))
        self.T_control_mode11.config(text="control_termination:"+str(self.vehicle_control_mode.flag_control_termination_enabled))
        
        self.T_failsafe1.config(text="battery_warning : "+str(self.failsafe_flags.battery_warning))
        self.T_failsafe2.config(text="battery_low_remaining_time : "+str(self.failsafe_flags.battery_low_remaining_time))

        self.F.update_idletasks()
        self.F.update()
        

    def cb_vehicle_status(self,msg):
        # self.get_logger().info("sub_vehicle_status")
        self.vehicle_status=msg    
    def cb_vehicle_odometry(self,msg):
        # self.get_logger().info("sub_vehicle_status")
        self.vehicle_odometry=msg
        
    def cb_batt(self,msg):
        # self.get_logger().info("sub_batt "+str(self.battery_status.current_a))
        self.battery_status=msg    
    def cb_vehicle_control_mode(self,msg):
        self.vehicle_control_mode=msg    
    def cb_failsafe_flags(self,msg):
        self.failsafe_flag=msg
    def cb_vehicle_attitude(self,msg):
        self.vehicle_attitude=msg



def main(args=None):
    rclpy.init(args=args)
    node=supervision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()