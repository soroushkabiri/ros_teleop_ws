
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests
from std_msgs.msg import String , Bool
import aiohttp
import asyncio
import threading

ESP32_IP_L = "192.168.1.121"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_F2 = "192.168.1.119"  # Default IP

#ESP32_IP_L = "172.20.10.2"  # Default IP
#ESP32_IP_F1 = "172.20.10.5"  # Default IP
#ESP32_IP_F2 = "172.20.10.4"  # Default IP


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

        # --- Asyncio background loop ---
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        # Schedule async tasks on the background loop
        asyncio.run_coroutine_threadsafe(self.periodic_send(), self.loop)
        asyncio.run_coroutine_threadsafe(self.periodic_status(), self.loop)

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

    #async def send_pid(self, kp, ki, kd):
     #   params = {'kp': kp, 'ki': ki, 'kd': kd}
     #   async with aiohttp.ClientSession() as session:
     #       for ip, name in [(ESP32_IP_L, 'L'), (ESP32_IP_F1, 'F1'), (ESP32_IP_F2, 'F2')]:
     #           try:
     #               async with session.get(f'http://{ip}/set_pid', params=params, timeout=0.5):
     #                   self.get_logger().info(f"Sent PID to {name}")
     #           except Exception as e:
     #               self.get_logger().error(f"Failed to send PID to {name}: {e}")

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