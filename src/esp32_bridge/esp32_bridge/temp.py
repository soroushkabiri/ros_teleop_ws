#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests
from std_msgs.msg import String , Bool


ESP32_IP_F2 = "192.168.1.119"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_L = "192.168.1.121"  # Default IP

WHEEL_BASE = 0.4  # meters

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # the variable to set the robots to start or pause from web
        self.robots_command='pause'

        # the variable to set the leader set its cmd_vel to fuzzy planner or joystick
        self.joystick_override = False  

        # the variable to set how many followers i have.
        self.followers_number = '2'  

        # Default PID values
        self.default_kp = 400.0
        self.default_ki = 100.0
        self.default_kd = 50.0


        self.def_initial_pr_L = 0.0
        self.def_initial_pl_L = 0.0
        self.def_initial_pr_F1 = 0.0
        self.def_initial_pl_F1 = 0.0
        self.def_initial_pr_F2 = 0.0
        self.def_initial_pl_F2 = 0.0
        # Subscribe to /robot_command to know when the process should begin
        self.create_subscription(String, '/robot_command', self.robots_command_callback, 10)
        
        # Subscribe to /followers_number to know when the process should begin
        self.create_subscription(String, '/followers_number', self.followers_number_callback, 10)
        
        # Subscribe to joystick override flag
        self.create_subscription(Bool, '/joystick_override', self.joystick_override_callback, 10)

        # Subscribe to /cmd_vel for leader
        self.create_subscription(Twist, '/cmd_vel_fuzzy', self.cmd_vel_fuzzy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.cmd_vel_joy_callback, 10)
        self.current_cmd_L = Twist()
        
        # Subscribe to /cmd_vel_des for follower 1
        self.create_subscription(Twist, '/robot0_1/cmd_vel_des', self.cmd_vel_callback_F1, 10)

        # Subscribe to /cmd_vel_des for follower 2
        self.create_subscription(Twist, '/robot1_0/cmd_vel_des', self.cmd_vel_callback_F2, 10)

        # Subscribe to /set_pwm_params
        self.create_subscription(Float32MultiArray, '/set_pwm', self.pwm_callback, 10)


        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pid_params', self.pid_callback, 10)

        # Publisher for actual velocities of leader
        self.actual_vel_pub_L = self.create_publisher(Twist, '/robot0_0/cmd_vel', 10)
        # Publisher for actual velocities of folower 1
        self.actual_vel_pub_F1 = self.create_publisher(Twist, '/robot0_1/cmd_vel', 10)
        # robot0_1=F1
        # Publisher for actual velocities of folower 2
        self.actual_vel_pub_F2 = self.create_publisher(Twist, '/robot1_0/cmd_vel', 10)

        # Timer to periodically fetch status from ESP32 
        self.create_timer(1/40, self.status_callback)

    def robots_command_callback(self, msg):
        self.robots_command = msg.data   # store the string ("start" or "pause")

    def followers_number_callback(self, msg):
        self.followers_number = msg.data   # store the string ("1" or "2")

    def joystick_override_callback(self, msg):
        self.joystick_override = msg.data
        #self.get_logger().info(f"Joystick override set to: {self.joystick_override}")

    def cmd_vel_fuzzy_callback(self, msg):
        if not self.joystick_override:   # only use fuzzy when joystick override is off
            self.current_cmd_L = msg
            self.send_to_esp32()

    def cmd_vel_joy_callback(self, msg):
        self.joystick_override = True    # enable joystick override
        self.current_cmd_L = msg
        self.send_to_esp32()

    def send_to_esp32(self):
        if self.robots_command == 'start':
            linear = max(min(self.current_cmd_L.linear.x, 0.2), -0.2)
            angular = max(min(self.current_cmd_L.angular.z, 0.6), -0.6)
        else:
            linear = 0.0
            angular = 0.0

        try:
            params = {
                'desired_linear_velocity_L': linear,
                'desired_angular_velocity_L': angular
            }
            response = requests.get(
                f'http://{ESP32_IP_L}/v_desired_L',
                params=params,
                timeout=1
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to ESP32: {e}")

    def cmd_vel_callback_F1(self, msg):
        
        if self.robots_command=='start':
            linear = max(min(msg.linear.x, 0.2), -0.2)
            angular = max(min(msg.angular.z, 0.6), -0.6)
        else:
            linear=0.0
            angular=0.0
        self.send_velocity_F1(linear, angular)


    def cmd_vel_callback_F2(self, msg):
        #if self.followers_number != '2':
        #    return  # ignore if only 1 follower is active
        
        if self.robots_command=='start':
            linear = max(min(msg.linear.x, 0.2), -0.2)
            angular = max(min(msg.angular.z, 0.6), -0.6)
        else:
            linear = 0.0
            angular = 0.0

        self.send_velocity_F2(linear, angular)

    def send_velocity_F1(self, linear, angular):
        try:
            params = {
                'desired_linear_velocity_F1': linear,
                'desired_angular_velocity_F1': angular
            }
            response = requests.get(f'http://{ESP32_IP_F1}/v_desired_F1', params=params, timeout=1)
            self.get_logger().info(f"Sent to F1 ESP32: v={linear:.2f}, w={angular:.2f}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to follower 1 ESP32: {e}")

    def send_velocity_F2(self, linear, angular):
        try:
            params = {
                'desired_linear_velocity_F2': linear,
                'desired_angular_velocity_F2': angular
            }
            response = requests.get(f'http://{ESP32_IP_F2}/v_desired_F2', params=params, timeout=1)
            self.get_logger().info(f"Sent to F2 ESP32: v={linear:.2f}, w={angular:.2f}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to follower 2 ESP32: {e}")

    def destroy_node(self):
        # Send zero velocity before shutting down
        self.get_logger().info("Node shutting down, stopping Follower 1 and 2...")
        self.send_velocity_F1(0.0, 0.0)
        #if self.followers_number=='2':
        self.send_velocity_F2(0.0, 0.0)

        super().destroy_node()

    def pid_callback(self, msg):
        
        data = list(msg.data)
        kp = self.default_kp
        ki = self.default_ki
        kd = self.default_kd

        try:
            if len(data) >= 1:
                kp = data[0]
            if len(data) >= 2:
                ki = data[1]
            if len(data) >= 3:
                kd = data[2]
            #kp, ki, kd = msg.data
            params = {'kp': kp,'ki': ki,'kd': kd}
            res_L = requests.get(f'http://{ESP32_IP_L}/set_pid', params=params, timeout=1)
            resp_F1 = requests.get(f'http://{ESP32_IP_F1}/set_pid', params=params, timeout=1)
            #if self.followers_number=='2':
            resp_F2 = requests.get(f'http://{ESP32_IP_F2}/set_pid', params=params, timeout=1)


            #self.get_logger().info(f"Sent PID to ESP32: kp={kp}, ki={ki}, kd={kd}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PID params to ESP32: {e}")


    def pwm_callback(self, msg):
        
        data = list(msg.data)
        initial_pr_L = self.def_initial_pr_L
        initial_pl_L = self.def_initial_pl_L
        initial_pr_F2 = self.def_initial_pr_F2
        initial_pl_F2 = self.def_initial_pl_F2        
        initial_pr_F1 = self.def_initial_pr_F1
        initial_pl_F1 = self.def_initial_pl_F1     
        try:
            if len(data) >= 1:
                initial_pr_L = data[0]
                initial_pr_F1 = data[0]
                initial_pr_F2 = data[0]

            if len(data) >= 2:
                initial_pl_L = data[1]
                initial_pl_F1 = data[1]
                initial_pl_F2 = data[1]

            params_l = {'initial_pr_L': initial_pr_L,'initial_pl_L': initial_pl_L,}
            params_F1 = {'initial_pr_F1': initial_pr_F1,'initial_pl_F1': initial_pl_F1,}
            #if self.followers_number=="2":
            params_F2 = {'initial_pr_F2': initial_pr_F2,'initial_pl_F2': initial_pl_F2,}
            response_F2 = requests.get(f'http://{ESP32_IP_F2}/set_pwm_F2', params=params_F2, timeout=1)

            response_l = requests.get(f'http://{ESP32_IP_L}/set_pwm_L', params=params_l, timeout=1)
            response_F1 = requests.get(f'http://{ESP32_IP_F1}/set_pwm_F1', params=params_F1, timeout=1)

            #self.get_logger().info(f"Sent Pwm to ESP32: initial_pr_F2={initial_pr_F2}, initial_pl_F2={initial_pl_F2}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send pwm params to ESP32: {e}")


    def status_callback(self):
        # This is "periodic callback" to GET /status and handle data
        try:
            response_F1 = requests.get(f"http://{ESP32_IP_F1}/status_F1", timeout=1)
            response_F1.raise_for_status()
            data_F1 = response_F1.json()
            
            #if self.followers_number=='2':
            response_F2 = requests.get(f"http://{ESP32_IP_F2}/status_F2", timeout=1)
            response_F2.raise_for_status()
            data_F2 = response_F2.json()
            speed_m1_F2 = float(data_F2.get("actual_speed_m1_F2", 0.0))
            speed_m2_F2  = float(data_F2 .get("actual_speed_m2_F2", 0.0))
            v_actual_F2  = (speed_m1_F2  + speed_m2_F2 ) / 2.0
            w_actual_F2  = (speed_m1_F2  - speed_m2_F2 ) / WHEEL_BASE
            speed_m1_d_F2  = float(data_F2 .get("desired_speed_m1_F2", 0.0))
            speed_m2_d_F2  = float(data_F2 .get("desired_speed_m2_F2", 0.0))
            v_d_F2  = (speed_m1_d_F2  + speed_m2_d_F2 ) / 2.0
            w_d_F2  = (speed_m1_d_F2  - speed_m2_d_F2 ) / WHEEL_BASE
            twist_msg_F2 = Twist()
            twist_msg_F2.linear.x = v_actual_F2
            twist_msg_F2.angular.z = w_actual_F2
            self.actual_vel_pub_F2.publish(twist_msg_F2)


            response_L = requests.get(f"http://{ESP32_IP_L}/status_L", timeout=1)
            response_L.raise_for_status()
            data_L = response_L.json()


            speed_m1_F1 = float(data_F1.get("actual_speed_m1_F", 0.0))
            speed_m2_F1  = float(data_F1 .get("actual_speed_m2_F", 0.0))

            speed_m1_L = float(data_L.get("actual_speed_m1_L", 0.0))
            speed_m2_L  = float(data_L .get("actual_speed_m2_L", 0.0))

            v_actual_F1  = (speed_m1_F1  + speed_m2_F1 ) / 2.0
            w_actual_F1  = (speed_m1_F1  - speed_m2_F1 ) / WHEEL_BASE

            v_actual_L  = (speed_m1_L  + speed_m2_L ) / 2.0
            w_actual_L  = (speed_m1_L  - speed_m2_L ) / WHEEL_BASE

            speed_m1_d_F1  = float(data_F1 .get("desired_speed_m1_F1", 0.0))
            speed_m2_d_F1  = float(data_F1 .get("desired_speed_m2_F1", 0.0))

            speed_m1_d_L  = float(data_L .get("desired_speed_m1_L", 0.0))
            speed_m2_d_L  = float(data_L .get("desired_speed_m2_L", 0.0))

            v_d_F1  = (speed_m1_d_F1  + speed_m2_d_F1 ) / 2.0
            w_d_F1  = (speed_m1_d_F1  - speed_m2_d_F1 ) / WHEEL_BASE
            
            v_d_L  = (speed_m1_d_L  + speed_m2_d_L ) / 2.0
            w_d_L  = (speed_m1_d_L  - speed_m2_d_L ) / WHEEL_BASE

            #self.get_logger().info(f"Actual speeds -> M1: {speed_m1:.3f}, M2: {speed_m2:.3f}")
            #self.get_logger().info(f"actual linear velocities -> v_actual_Leader: {v_actual_L:.3f} m/s, v_actual_follower: {v_actual_F1:.3f} rad/s")
            #self.get_logger().info(f"desired linear velocities -> v_desired_Leader: {v_d_L:.3f} m/s, v_desired_follower: {v_d_F1:.3f} rad/s")

            #self.get_logger().info(f"actual angular velocities -> w_actual_leader: {w_actual_L:.3f} m/s, w_actual_follower: {w_actual_F1:.3f} rad/s")
            #self.get_logger().info(f"desired angular velocities -> w_desired_leader: {w_d_L:.3f} m/s, w_desired_follower: {w_d_F1:.3f} rad/s")

            twist_msg_F1 = Twist()
            twist_msg_F1.linear.x = v_actual_F1
            twist_msg_F1.angular.z = w_actual_F1
            self.actual_vel_pub_F1.publish(twist_msg_F1)

            twist_msg_L = Twist()
            twist_msg_L.linear.x = v_actual_L
            twist_msg_L.angular.z = w_actual_L
            self.actual_vel_pub_L.publish(twist_msg_L)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get status from ESP32: {e}")
        except ValueError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# the way to set pid parameters:
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [500, 0.01, 0.1]}"








#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests
from std_msgs.msg import String , Bool
import aiohttp
import asyncio

ESP32_IP_F2 = "192.168.1.119"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_L = "192.168.1.121"  # Default IP

WHEEL_BASE = 0.4  # meters

class ESP32BridgeAsync(Node):
    def __init__(self):
        super().__init__('esp32_bridge_async')
        
        # the variable to set the robots to start or pause from web
        self.robots_command='pause'
        # the variable to set the leader set its cmd_vel to fuzzy planner or joystick
        self.joystick_override = False  
        # the variable to set how many followers i have.
        self.followers_number = '2'  

        # Default PID and pwm values
        self.default_kp = 400.0
        self.default_ki = 100.0
        self.default_kd = 50.0
        self.def_initial_pr_L = 0.0
        self.def_initial_pl_L = 0.0
        self.def_initial_pr_F1 = 0.0
        self.def_initial_pl_F1 = 0.0
        self.def_initial_pr_F2 = 0.0
        self.def_initial_pl_F2 = 0.0
    
        # Current commands
        self.current_cmd_L = Twist()
        self.cmd_F1 = Twist()
        self.cmd_F2 = Twist()
        #self.current_cmd_L = Twist()
        
        # Subscribe to /robot_command to know when the process should begin
        self.create_subscription(String, '/robot_command', self.robots_command_callback, 10)
        # Subscribe to /followers_number to know when the process should begin
        self.create_subscription(String, '/followers_number', self.followers_number_callback, 10)
        # Subscribe to joystick override flag
        self.create_subscription(Bool, '/joystick_override', self.joystick_override_callback, 10)
        # Subscribe to /cmd_vel for leader
        self.create_subscription(Twist, '/cmd_vel_fuzzy', self.cmd_vel_fuzzy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.cmd_vel_joy_callback, 10)
        # Subscribe to /cmd_vel_des for follower 1
        self.create_subscription(Twist, '/robot0_1/cmd_vel_des', self.cmd_vel_callback_F1, 10)
        # Subscribe to /cmd_vel_des for follower 2
        self.create_subscription(Twist, '/robot1_0/cmd_vel_des', self.cmd_vel_callback_F2, 10)
        # Subscribe to /set_pwm_params
        self.create_subscription(Float32MultiArray, '/set_pwm', self.pwm_callback, 10)
        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pid_params', self.pid_callback, 10)
        
        # Publisher for actual velocities of leader
        self.actual_vel_pub_L = self.create_publisher(Twist, '/robot0_0/cmd_vel', 10)
        # Publisher for actual velocities of folower 1
        self.actual_vel_pub_F1 = self.create_publisher(Twist, '/robot0_1/cmd_vel', 10)
        # robot0_1=F1
        # Publisher for actual velocities of folower 2
        self.actual_vel_pub_F2 = self.create_publisher(Twist, '/robot1_0/cmd_vel', 10)

        # Start async loop
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.periodic_send())
        self.loop.create_task(self.periodic_status())
        
        self.create_timer(1/40, lambda: asyncio.create_task(self.periodic_send()))
        self.create_timer(1/40, lambda: asyncio.create_task(self.periodic_status()))

        
        # Timer to periodically fetch status from ESP32 
        #self.create_timer(1/40, self.status_callback)
   
    # ================= Sub callbacks =================
    def robots_command_callback(self, msg):
        self.robots_command = msg.data   # store the string ("start" or "pause")
    def followers_number_callback(self, msg):
        self.followers_number = msg.data   # store the string ("1" or "2")
    def joystick_override_callback(self, msg):
        self.joystick_override = msg.data
        #self.get_logger().info(f"Joystick override set to: {self.joystick_override}")

    def cmd_vel_fuzzy_callback(self, msg):
        if not self.joystick_override:   # only use fuzzy when joystick override is off
            self.current_cmd_L = msg
            #self.send_to_esp32()

    def cmd_vel_joy_callback(self, msg):
        self.joystick_override = True    # enable joystick override
        self.current_cmd_L = msg
        #self.send_to_esp32()

    def cmd_vel_callback_F1(self, msg):
        self.cmd_F1 = msg

    def cmd_vel_callback_F2(self, msg):
        self.cmd_F2 = msg


   # ================= Async HTTP =================
    async def send_velocity(self, ip, linear, angular, name):
        params = {
            f'desired_linear_velocity_{name}': linear,
            f'desired_angular_velocity_{name}': angular
        }
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(f'http://{ip}/v_desired_{name}', params=params, timeout=0.5) as resp:
                    text = await resp.text()
                    self.get_logger().info(f"Sent to {name} ESP32: v={linear:.2f}, w={angular:.2f}, Response: {text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send velocity to {name} ESP32: {e}")

    async def send_pid(self, kp, ki, kd):
        params = {'kp': kp, 'ki': ki, 'kd': kd}
        async with aiohttp.ClientSession() as session:
            for ip, name in [(ESP32_IP_L, 'L'), (ESP32_IP_F1, 'F1'), (ESP32_IP_F2, 'F2')]:
                try:
                    async with session.get(f'http://{ip}/set_pid', params=params, timeout=0.5):
                        self.get_logger().info(f"Sent PID to {name}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send PID to {name}: {e}")





    async def send_pid(self, kp, ki, kd):
        params = {'kp': kp, 'ki': ki, 'kd': kd}
        async with aiohttp.ClientSession() as session:
            for ip, name in [(ESP32_IP_L, 'L'), (ESP32_IP_F1, 'F1'), (ESP32_IP_F2, 'F2')]:
                try:
                    async with session.get(f'http://{ip}/set_pid', params=params, timeout=0.5):
                        self.get_logger().info(f"Sent PID to {name}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send PID to {name}: {e}")

    async def send_pwm(self, pr_L, pl_L, pr_F1, pl_F1, pr_F2, pl_F2):
        async with aiohttp.ClientSession() as session:
            try:
                await asyncio.gather(
                    session.get(f'http://{ESP32_IP_L}/set_pwm_L', params={'initial_pr_L': pr_L, 'initial_pl_L': pl_L}),
                    session.get(f'http://{ESP32_IP_F1}/set_pwm_F1', params={'initial_pr_F1': pr_F1, 'initial_pl_F1': pl_F1}),
                    session.get(f'http://{ESP32_IP_F2}/set_pwm_F2', params={'initial_pr_F2': pr_F2, 'initial_pl_F2': pl_F2}),
                )
                self.get_logger().info("PWM sent to all ESP32s")
            except Exception as e:
                self.get_logger().error(f"Failed to send PWM: {e}")

    # ================= Periodic tasks =================
    async def periodic_send(self):
        while rclpy.ok():
            if self.robots_command == 'start':
                v_L = max(min(self.current_cmd_L.linear.x, 0.2), -0.2)
                w_L = max(min(self.current_cmd_L.angular.z, 0.6), -0.6)
                v_F1 = max(min(self.cmd_F1.linear.x, 0.2), -0.2)
                w_F1 = max(min(self.cmd_F1.angular.z, 0.6), -0.6)
                v_F2 = max(min(self.cmd_F2.linear.x, 0.2), -0.2)
                w_F2 = max(min(self.cmd_F2.angular.z, 0.6), -0.6)
            else:
                v_L = w_L = v_F1 = w_F1 = v_F2 = w_F2 = 0.0

            await asyncio.gather(
                self.send_velocity(ESP32_IP_L, v_L, w_L, 'L'),
                self.send_velocity(ESP32_IP_F1, v_F1, w_F1, 'F1'),
                self.send_velocity(ESP32_IP_F2, v_F2, w_F2, 'F2')
            )
            await asyncio.sleep(1/40)  # 40 Hz

    async def periodic_status(self):
        while rclpy.ok():
            try:
                async with aiohttp.ClientSession() as session:
                    resp_F1 = await session.get(f"http://{ESP32_IP_F1}/status_F1", timeout=0.5)
                    data_F1 = await resp_F1.json()

                    resp_F2 = await session.get(f"http://{ESP32_IP_F2}/status_F2", timeout=0.5)
                    data_F2 = await resp_F2.json()

                    resp_L = await session.get(f"http://{ESP32_IP_L}/status_L", timeout=0.5)
                    data_L = await resp_L.json()

                    # Compute velocities
                    def compute_vw(data, prefix):
                        m1 = float(data.get(f"actual_speed_m1_{prefix}", 0.0))
                        m2 = float(data.get(f"actual_speed_m2_{prefix}", 0.0))
                        v = (m1 + m2)/2
                        w = (m1 - m2)/WHEEL_BASE
                        return v, w

                    v_L, w_L = compute_vw(data_L, "L")
                    v_F1, w_F1 = compute_vw(data_F1, "F1")
                    v_F2, w_F2 = compute_vw(data_F2, "F2")

                    # Publish
                    for pub, v, w in [(self.actual_vel_pub_L, v_L, w_L),
                                      (self.actual_vel_pub_F1, v_F1, w_F1),
                                      (self.actual_vel_pub_F2, v_F2, w_F2)]:
                        msg = Twist()
                        msg.linear.x = v
                        msg.angular.z = w
                        pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Failed to fetch status: {e}")
            await asyncio.sleep(1/40)

    # ================= PID & PWM =================
    def pid_callback(self, msg):
        data = list(msg.data)
        kp = data[0] if len(data) > 0 else self.default_kp
        ki = data[1] if len(data) > 1 else self.default_ki
        kd = data[2] if len(data) > 2 else self.default_kd
        self.loop.create_task(self.send_pid(kp, ki, kd))

    def pwm_callback(self, msg):
        data = list(msg.data)
        pr_L = pl_L = pr_F1 = pl_F1 = pr_F2 = pl_F2 = 0.0
        if len(data) >= 1:
            pr_L = pr_F1 = pr_F2 = data[0]
        if len(data) >= 2:
            pl_L = pl_F1 = pl_F2 = data[1]
        self.loop.create_task(self.send_pwm(pr_L, pl_L, pr_F1, pl_F1, pr_F2, pl_F2))

    # ================= Shutdown =================
    def destroy_node(self):
        self.loop.create_task(self.send_velocity(ESP32_IP_F1, 0, 0, 'F1'))
        self.loop.create_task(self.send_velocity(ESP32_IP_F2, 0, 0, 'F2'))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeAsync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# the way to set pid parameters:
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [500, 0.01, 0.1]}"
# the way to set pwm parameters:
#ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data:[72, 72]}"











#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
# --- IEEE plotting style ---
plt.rcParams.update({
    "font.family": "serif",      # Times/Serif font
    "font.size": 12,              # base font
    "axes.titlesize": 13,
    "axes.labelsize": 13,
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11
})

class BagPlotter(Node):
    def __init__(self):
        super().__init__('bag_plotter')

        # --- Path to your bag file ---
        bag_path = "map_real/map_real_0.db3"
        self.get_logger().info(f"Loading bag: {bag_path}")

        # Open bag database
        con = sqlite3.connect(bag_path)
        cur = con.cursor()

        # --- Get topics ---
        topics = cur.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_map = {name: (id, type) for id, name, type in topics}
        self.get_logger().info(f"Topics in bag: {list(self.topic_map.keys())}")

        # --- Global time zero (earliest message in bag) ---
        cur.execute("SELECT MIN(timestamp) FROM messages")
        t0_ns = cur.fetchone()[0]
        self.global_t0 = t0_ns / 1e9  # ns → s
        self.get_logger().info(f"Global start time: {self.global_t0:.3f} s (epoch)")

        # --- Create one figure with 2 subplots (stacked vertically) ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True)

        # --- Subplot 1: leader cmd_vel vs follower v_hat ---
        leader_topic = "/cmd_vel_joy"
        follower_vhat_topics = [
            "/robot0_1/v_hat",
            "/robot1_0/v_hat",
        ]

        # Leader (Twist)
        df_leader = self.load_twist(cur, leader_topic)
        if not df_leader.empty:
            ax1.plot(df_leader["time"], df_leader["linear_x"], "k--", 
                    label="Leader desired V")

        # Followers (Float32)
        for i, topic in enumerate(follower_vhat_topics, start=1):
            df = self.load_float(cur, topic)
            if not df.empty:
                ax1.plot(df["time"], df["value"], label=f"Follower {i} estimated V")

        ax1.set_ylabel("Velocity [m/s]")
        ax1.set_title("Leader desired V vs Followers estimated V")
        ax1.grid(True)
        ax1.legend()

        # --- Subplot 2: yaw comparison ---
        yaw_leader_topic = "/robot0_0/yaw_deg"  # already in degrees
        yaw_hat_topics = [
            "/robot0_1/yaw_hat",
            "/robot1_0/yaw_hat",
        ]

        df_leader = self.load_float(cur, yaw_leader_topic, radians=False, wrap360=True)
        if not df_leader.empty:
            ax2.plot(df_leader["time"], df_leader["value"], "k--", 
                    label=r"Leader desired $\theta$")

        for i, topic in enumerate(yaw_hat_topics, start=1):
            df = self.load_float(cur, topic, radians=True, wrap360=True)  # rad→deg then wrap
            if not df.empty:
                ax2.plot(df["time"], df["value"], label=rf"Follower {i} estimated $\theta$")

        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel(r"Yaw [deg] (0–360)")
        ax2.set_title(r"Leader desired $\theta$ vs Followers estimated $\theta$")
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.show()


        # Shutdown after plotting
        rclpy.shutdown()

    def load_twist(self, cur, topic_name):
        """Load geometry_msgs/Twist messages and align to global t0"""
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "linear_x"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=?",
            (topic_id,)
        ).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Twist)
            time_s = (t / 1e9) - self.global_t0
            data.append([time_s, msg.linear.x])
        return pd.DataFrame(data, columns=["time", "linear_x"])

    def load_float(self, cur, topic_name, radians=False, wrap360=False):
        """Load std_msgs/Float32 messages and align to global t0"""
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "value"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=?",
            (topic_id,)
        ).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Float32)
            val = msg.data
            if radians:
                val = math.degrees(val)
            if wrap360:
                val = (val + 360) % 360
            time_s = (t / 1e9) - self.global_t0
            data.append([time_s, val])
        return pd.DataFrame(data, columns=["time", "value"])


def main(args=None):
    rclpy.init(args=args)
    BagPlotter()
    rclpy.spin(rclpy.get_global_executor())


if __name__ == "__main__":
    main()
