#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
# from px4_msgs.msg import *
from sensor_msgs.msg import Joy
from math import sin,cos
from geometry_msgs.msg import Twist

B_CROIX=0
B_ROND=1
B_CARRE=2
B_TRIANGLE=3
B_SHARE=4
B_PS=5
B_OPT=6
B_JL=7
B_JR=8
B_L1=9
B_R1=10
B_H=11
B_B=12
B_L=13
B_R=14
B_CLIC=15

J_LH=0
J_LV=1
J_L2=4
J_RH=2
J_RV=3
J_R2=5

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
        

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.setPoint=TrajectorySetpoint()
        self.joy=Joy()
        self.joy.axes=[0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0]
        self.joy.buttons=[0,0,0,0,0,0,0,0,0,0,0,0,0]

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # https://docs.px4.io/main/en/msg_docs/VehicleCommand.html
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        #https://docs.px4.io/v1.13/en/msg_docs/vehicle_status.html
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.subscriber = self.create_subscription(Joy,"joy", self.cb_joy,1)

        self.subscriber_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel,1)
        

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.cpt2=0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -3.0

        self.yaw = 0.0
        self.max_speed=3.0
        

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    ### MOI : ??
    def takeoff(self):
        # uint16 VEHICLE_CMD_NAV_TAKEOFF = 22			
        # # Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), 
        # desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
        # param1 : Pitch/climb angle in degrees
        p1=0.0 #pitch
        p5=self.vehicle_local_position.ref_lat #latitude
        p6=self.vehicle_local_position.ref_lon #longitude
        p7=self.vehicle_local_position.ref_alt+3.5 #altitude + 3.5 de decollage
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=p1,param5=p5,param6=p6,param7=p7)
        self.get_logger().info('Takeoff command sent')


    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"cpt: {self.cpt2}  Publishing position setpoints {[x, y, z, yaw]}")

    def publish_velocity_setpoint(self, x: float, y: float, z: float, yaw:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.yaw = -yaw 
        # msg.yawspeed = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"cpt: {self.cpt2}  Publishing velocity setpoints {[x, y, z, yaw]}")


    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0) 
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def cb_joy(self,msg):
        self.joy=msg

    def cb_cmd_vel(self,msg):
        self.speed=msg    
    
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.joy.buttons[B_TRIANGLE]:
            self.engage_offboard_mode()
            self.arm()
            self.takeoff()

        if self.joy.buttons[B_CROIX]:
            self.land()

        # self.get_logger().info(f"nav_state: {self.vehicle_status.nav_state} ")
        if self.joy.buttons[B_ROND]==1: #active offboard
            self.engage_offboard_mode()
            self.position=[ self.vehicle_local_position.x,self.vehicle_local_position.y,self.vehicle_local_position.z]


        # set the do change speed before enterring offboard mode when armed
        if  self.vehicle_status.arming_state==2: # armed
            self.publish_offboard_control_heartbeat_signal()
            
            ## do change speed
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED, param1=0.0, param2=1.0)

        if self.vehicle_status.nav_state==14 and self.vehicle_status.arming_state==2: 


            cs=cos(self.vehicle_local_position.heading)
            ss=sin(self.vehicle_local_position.heading)


            # COMMANDE EN VITESSE et POSITION Z
            veloce_x=(self.joy.axes[J_LH]*ss+self.joy.axes[J_LV]*cs)*self.max_speed
            veloce_y=(-self.joy.axes[J_LH]*cs+self.joy.axes[J_LV]*ss)*self.max_speed
            veloce_z=-self.joy.axes[J_RV]*self.max_speed
            # new_yaw = self.vehicle_local_position.heading+ (self.joy.axes[J_RH])*0.05 # 0.05 is like a dt here
            self.yaw = self.yaw + (self.joy.axes[J_RH])*self.max_speed*0.05
            self.publish_velocity_setpoint(veloce_x,veloce_y,veloce_z,self.yaw)
            

    

        # if self.offboard_setpoint_counter == 10:
        #     self.engage_offboard_mode()
        #     self.arm()
        #     self.takeoff()

        # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     #self.publish_position_setpoint(0.0, 0.0, self.takeoff_height,0.0)
        #     # self.takeoff()
        #     self.cpt2+=1

        # elif self.vehicle_local_position.z <= self.takeoff_height and self.cpt2>100:
        #     self.land()
        #     exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        # print(f"timer :{self.offboard_setpoint_counter} local pos z: {self.vehicle_local_position.z}")


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)


































# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,VehicleOdometry,SensorGps
# from sensor_msgs.msg import Joy
# from math import sin,cos,asin,acos,atan2,pi
# from time import sleep

# B_CROIX=0
# B_ROND=1
# B_CARRE=2
# B_TRIANGLE=3
# B_SHARE=4
# B_PS=5
# B_OPT=6
# B_JL=7
# B_JR=8
# B_L1=9
# B_R1=10
# B_H=11
# B_B=12
# B_L=13
# B_R=14
# B_CLIC=15

# J_LH=0
# J_LV=1
# J_L2=4
# J_RH=2
# J_RV=3
# J_R2=5

# dic_nav_state={0:"Manuel",
#                 1:"Altitude",
#                 2:"Position",
#                 3:"auto mission",
#                 4:"auto loiter",
#                 5:"autoreturn to launch",
#                 6:"POSITION_SLOW",
#                 10:"Acro",
#                 12:"Descend",
#                 13:"Termination",
#                 14:"OffBoard",
#                 15:"Stabilized",
#                 17:"TakeOff",
#                 18:"Land",
#                 19:"Auto Follow",
#                 20:"Precision Land",
#                 21:"Orbit"
#                 }

# def euler_from_quaternion(w,x, y, z):
#         """
#         Convert a quaternion into euler angles (roll, pitch, yaw)
#         roll is rotation around x in radians (counterclockwise)
#         pitch is rotation around y in radians (counterclockwise)
#         yaw is rotation around z in radians (counterclockwise)
#         """
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll_x = atan2(t0, t1)
     
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch_y = asin(t2)
     
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw_z = atan2(t3, t4)
     
#         return roll_x, pitch_y, yaw_z # in radians
        

# class OffboardControl(Node):
#     """Node for controlling a vehicle in offboard mode."""

#     def __init__(self) -> None:
#         super().__init__('offboard_control_takeoff_and_land')

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.setPoint=TrajectorySetpoint()
#         self.joy=Joy()
#         self.joy.axes=[0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0]
#         self.joy.buttons=[0,0,0,0,0,0,0,0,0,0,0,0,0]
#         self.cons_position=[ 0.0,0.0,0.0]
#         self.yaw=0.0
#         self.heading=0.0

#         # Create publishers
#         self.offboard_control_mode_publisher = self.create_publisher(
#             OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         # https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html
#         self.trajectory_setpoint_publisher = self.create_publisher(
#             TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
#         # https://docs.px4.io/main/en/msg_docs/VehicleCommand.html
#         self.vehicle_command_publisher = self.create_publisher(
#             VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

#         # Create subscribers

#         self.vehicle_local_position_subscriber = self.create_subscription(
#             VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

#         self.vehicle_gps_position_subscriber = self.create_subscription(
#             SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_position_callback, qos_profile)
            
#         #https://docs.px4.io/v1.13/en/msg_docs/vehicle_status.html
#         self.vehicle_status_subscriber = self.create_subscription(
#             VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
#         self.sub_vehicle_odometry=self.create_subscription(
#             VehicleOdometry,"/fmu/out/vehicle_odometry",self.cb_vehicle_odometry,qos_profile)
        
#         self.subscriber = self.create_subscription(Joy,"joy", self.cb_joy,1)

#         # Initialize variables
#         self.offboard_setpoint_counter = 0
#         self.cpt2=0
#         self.vehicle_local_position = VehicleLocalPosition()
#         self.vehicle_status = VehicleStatus()
#         self.takeoff_height = -3.0
#         self.vehicle_odometry=VehicleOdometry()
#         self.vehicle_gps_position=SensorGps()

#         # Create a timer to publish control commands
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def vehicle_local_position_callback(self, vehicle_local_position):
#         """Callback function for vehicle_local_position topic subscriber."""
#         self.vehicle_local_position = vehicle_local_position
#         # self.get_logger().info(f"local alt {self.vehicle_local_position.alt}")

#     def vehicle_gps_position_callback(self, vehicle_gps_position):
#         """Callback function for vehicle_local_position topic subscriber."""
#         self.vehicle_gps_position = vehicle_gps_position
#         # self.get_logger().info(f"local alt {self.vehicle_gps_position.alt}")

#     def vehicle_status_callback(self, vehicle_status):
#         """Callback function for vehicle_status topic subscriber."""
#         self.vehicle_status = vehicle_status

#     def arm(self):
#         """Send an arm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
#         self.get_logger().info('Arm command sent')

#     def disarm(self):
#         """Send a disarm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
#         self.get_logger().info('Disarm command sent')

#     ### MOI : ??
#     def takeoff(self):
#         # uint16 VEHICLE_CMD_NAV_TAKEOFF = 22			
#         # # Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), 
#         # desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
#         # param1 : Pitch/climb angle in degrees
#         p1=0.0 #pitch
#         p5=self.vehicle_local_position.ref_lat #latitude
#         p6=self.vehicle_local_position.ref_lon #longitude
#         p7=self.vehicle_local_position.ref_alt+3.5 #altitude + 3.5 de decollage        
        
#         p5=self.vehicle_gps_position.latitude_deg #latitude
#         p6=self.vehicle_gps_position.longitude_deg #longitude
#         p7=self.vehicle_gps_position.altitude_msl_m+3.5 #altitude + 3.5 de decollage
        
        
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=p1,param5=p5,param6=p6,param7=p7)
#         self.get_logger().info('Takeoff command sent')


#     def engage_offboard_mode(self):
#         """Switch to offboard mode."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
#         self.get_logger().info("Switching to offboard mode")

#     def land(self):
#         """Switch to land mode."""
        
#         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
#         #https://hamishwillee.github.io/px4_vuepress/en/advanced_features/precland.html#performing-a-precision-landing
#         # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND,param2=0.0)
#         self.get_logger().info("Switching to land mode")

#     def publish_offboard_control_heartbeat_signal(self):
#         """Publish the offboard control mode."""
#         msg = OffboardControlMode()
#         msg.position = True
#         msg.velocity = False
#         msg.acceleration = False
#         msg.attitude = False
#         msg.body_rate = False
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.offboard_control_mode_publisher.publish(msg)

#     def publish_position_setpoint(self, x: float, y: float, z: float, yaw:float):
#         """Publish the trajectory setpoint."""
#         msg = TrajectorySetpoint()
#         msg.position = [x, y, z]
#         msg.yaw = yaw  # (90 degree)
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.trajectory_setpoint_publisher.publish(msg)
#         # self.get_logger().info(f"cpt: {self.cpt2}  Publishing position setpoints {[x, y, z, yaw]}")

#     def publish_vehicle_command(self, command, **params) -> None:
#         """Publish a vehicle command."""
#         msg = VehicleCommand()
#         msg.command = command
#         msg.param1 = params.get("param1", 0.0)
#         msg.param2 = params.get("param2", 0.0)
#         msg.param3 = params.get("param3", 0.0)
#         msg.param4 = params.get("param4", 0.0)
#         msg.param5 = params.get("param5", 0.0)
#         msg.param6 = params.get("param6", 0.0)
#         msg.param7 = params.get("param7", 0.0)
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.vehicle_command_publisher.publish(msg)

#     def cb_joy(self,msg):
#         self.joy=msg
#         # zone morte pour la verticale (parasitage si rotation sur lui meme)
#         if abs(self.joy.axes[J_RV])<0.3:
#             self.joy.axes[J_RV]=0.0
#         if abs(self.joy.axes[J_RH])<0.2:
#             self.joy.axes[J_RH]=0.0
            
#     def cb_vehicle_odometry(self,msg):
#         # self.get_logger().info("sub_vehicle_status")
#         self.vehicle_odometry=msg
#         p,r,self.heading=euler_from_quaternion(msg.q[0], msg.q[1], msg.q[2],msg.q[3])        
    
#     def timer_callback(self) -> None:
#         """Callback function for the timer."""
#         self.publish_offboard_control_heartbeat_signal()

#         if self.joy.buttons[B_TRIANGLE]:
#             self.engage_offboard_mode()
#             self.arm()
#             self.takeoff()

#         if self.joy.buttons[B_CROIX]:
#             self.land()

#         # self.get_logger().info(f"nav_state: {self.vehicle_status.nav_state} ")
#         if self.joy.buttons[B_ROND]==1 and self.vehicle_status.arming_state==2 : #active offboard 
#             self.engage_offboard_mode()
#             # self.position=[ self.vehicle_local_position.x,self.vehicle_local_position.y,self.vehicle_local_position.z]
#             self.cons_position=self.vehicle_odometry.position
#             p,r,self.yaw=euler_from_quaternion(self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3])

#         if self.vehicle_status.nav_state==14 and self.vehicle_status.arming_state==2: #offboard mode && armed
            
#             msg=TrajectorySetpoint()

#             #inversion x-y PX4/ROS
#             #  COMMANDE en POSITION : les joysticks font évoluer les valeurs de pos
#             #  La position initiale est obtenue lors du clic sur le bouton rond 

#             cs=cos(self.heading)
#             ss=sin(self.heading)



#             self.cons_position[0]+=(self.joy.axes[J_LH]*ss+self.joy.axes[J_LV]*cs)/10.0
#             self.cons_position[1]+=(-self.joy.axes[J_LH]*cs+self.joy.axes[J_LV]*ss)/10.0
#             self.cons_position[2]-=self.joy.axes[J_RV]/2.0
            
#             self.yaw-=self.joy.axes[J_RH]/5.0
#             if self.yaw>pi:
#                 self.yaw-=2*pi 
#             if self.yaw<-pi:
#                 self.yaw+=2*pi

#             msg.position=self.cons_position
#             msg.yaw=self.yaw

#             # self.get_logger().info(f"cmd pos: {msg.position}")
#             # msg.velocity=[self.joy.axes[J_LV],self.joy.axes[J_LH],self.joy.axes[J_RV]]
            
#             # msg.yawspeed=self.joy.axes[J_RH]
#             msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#             self.trajectory_setpoint_publisher.publish(msg)
#             # self.get_logger().info(f"cmd yawspeed: {msg.yawspeed} yaw: {msg.yaw}")
#             self.get_logger().info(f"heading :{self.heading} cmd yaw: {msg.yaw}")


#         if self.offboard_setpoint_counter < 11:
#             self.offboard_setpoint_counter += 1
#         # print(f"timer :{self.offboard_setpoint_counter} local pos z: {self.vehicle_local_position.z}")


# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)
