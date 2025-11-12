import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import socket
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
import uvicorn
import threading
import subprocess
import os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class WebCommandNode(Node):
    def __init__(self):
        super().__init__('web_command_node')

       # Keep track of the subprocess
        self.robot_stack_process = None

        # Publisher for start/pause commands
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)

        # Publisher for yaw calibration state
        self.yaw_calib_publisher_ = self.create_publisher(String, 'robot_command_yaw_calib', 10)

        self.override_publisher_ = self.create_publisher(Bool, 'joystick_override', 10)
        
        # publisher for number of followers
        self.followers_pub_ = self.create_publisher(String, 'followers_number', 10)

        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        #self.goal_publisher_ = self.create_publisher(PoseStamped, '/rtabmap/goal', qos_profile)
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/dijkstra_goal', 10)

        # store current states
        self.current_command = "pause"            # default
        self.current_yaw_calib = "calibrating"    # default
        self.joystick_override = False            # default: autonomous
        self.last_goal = None   # <- no goal initially
        self.followers_number = "2"   # default one follower

        # publish commands every 0.5s
        self.timer = self.create_timer(0.5, self.publish_commands)

        # Start FastAPI server in a background thread
        self.app = FastAPI()
        self.setup_routes()

        thread = threading.Thread(
            target=lambda: uvicorn.run(self.app, host="0.0.0.0", port=8000, log_level="info"),
            daemon=True
        )
        thread.start()
        self.get_logger().info("Web server running on http://192.168.1.115:8000")


    def publish_commands(self):
        # Publish robot command
        msg = String()
        msg.data = self.current_command
        self.publisher_.publish(msg)

        # Publish yaw calibration state
        yaw_msg = String()
        yaw_msg.data = self.current_yaw_calib
        self.yaw_calib_publisher_.publish(yaw_msg)

        # Publish joystick override state
        override_msg = Bool()
        override_msg.data = self.joystick_override
        self.override_publisher_.publish(override_msg)

        # NEW: publish followers_number
        followers_msg = String()
        followers_msg.data = self.followers_number
        self.followers_pub_.publish(followers_msg)


        self.get_logger().info(
            f"Publishing: {msg.data}, YawCalib: {yaw_msg.data}, Override: {override_msg.data}, "
            f"Followers: {self.followers_number}, Goal: {self.last_goal}")

    def publish_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_publisher_.publish(msg)
        self.get_logger().info(f"Published goal: x={x}, y={y}")
        self.last_goal = (x, y)



    def setup_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            goal_text = "None" if self.last_goal is None else str(self.last_goal)
            return f"""
            <html>
                <head>
                    <title>Robot Control Panel</title>
                    <style>
                        body {{ font-family: Arial; text-align: center; margin-top: 50px; }}
                        button, input {{ font-size: 20px; padding: 10px; margin: 10px; }}
                        #status, #yaw_status, #override_status, #followers_status 
                            {{ font-size: 20px; margin-top: 30px; color: green; }}
                        h2 {{ margin-top: 40px; }}
                    </style>
                </head>
                <body>
                    <h1>Robot Control Panel</h1>

                    <h2>Launch Robot Stack</h2>
                    <button onclick="startStack()">Start Localization + Teleop</button>
                    <div id="stack_status">Not started</div>

                    <h2>Send Goal</h2>
                    <input type="number" id="goal_x" placeholder="X" step="0.1">
                    <input type="number" id="goal_y" placeholder="Y" step="0.1">
                    <button onclick="sendGoal()">Send Goal</button>
                    <div id="goal_status">Last goal: {goal_text}</div>

                    <h2>Robot Command</h2>
                    <button onclick="sendCommand('start')">Start</button>
                    <button onclick="sendCommand('pause')">Pause</button>
                    <div id="status">Current command: {self.current_command}</div>

                    <h2>Yaw Calibration</h2>
                    <button onclick="sendYaw('calibrated')">calibrated</button>
                    <button onclick="sendYaw('calibrating')">calibrating</button>
                    <div id="yaw_status">Yaw calibration state: {self.current_yaw_calib}</div>

                    <h2>Control Source</h2>
                    <button onclick="sendOverride('false')">Autonomous (fuzzy)</button>
                    <button onclick="sendOverride('true')">Manual (joystick)</button>
                    <div id="override_status">Joystick override: {self.joystick_override}</div>
                    
                    <h2>Followers Number</h2>
                    <button onclick="setFollowers('1')">1 Follower</button>
                    <button onclick="setFollowers('2')">2 Followers</button>
                    <div id="followers_status">Current followers: {self.followers_number}</div>
                     
                    
                    <script>


                        function startStack() {{
                            fetch('/start/robot_stack')
                                .then(r => r.json())
                                .then(data => {{
                                    document.getElementById("stack_status").innerText = data.message;
                                }});
                        }}



                        function sendGoal() {{
                            let gx = document.getElementById("goal_x").value;
                            let gy = document.getElementById("goal_y").value;
                            fetch(`/goal/${{gx}}/${{gy}}`).then(r=>r.json()).then(data=>{{
                                document.getElementById("goal_status").innerText = 
                                    "Last goal: (" + data.x + ", " + data.y + ")";
                            }});
                        }}

                        function sendCommand(cmd) {{
                            fetch('/command/' + cmd)
                                .then(response => response.json())
                                .then(data => {{
                                    document.getElementById("status").innerText =
                                        "Current command: " + data.command;
                                }});
                        }}
                        function sendYaw(state) {{
                            fetch('/yaw/' + state)
                                .then(response => response.json())
                                .then(data => {{
                                    document.getElementById("yaw_status").innerText =
                                        "Yaw calibration state: " + data.yaw;
                                }});
                        }}
                        function sendOverride(state) {{
                            fetch('/override/' + state)
                                .then(response => response.json())
                                .then(data => {{
                                    document.getElementById("override_status").innerText =
                                        "Joystick override: " + data.override;
                                }});
                        }}

                        function setFollowers(n) {{
                            fetch('/followers/' + n)
                                .then(response => response.json())
                                .then(data => {{
                                    document.getElementById("followers_status").innerText =
                                        "Current followers: " + data.followers;
                                }});
                        }}

                    </script>
                </body>
            </html>
            """

        @self.app.get("/command/{cmd}")
        async def command(cmd: str):
            self.current_command = cmd
            self.get_logger().info(f"Set command: {cmd}")
            return {"status": "ok", "command": cmd}

        @self.app.get("/yaw/{state}")
        async def yaw(state: str):
            if state in ["calibrating", "calibrated"]:
                self.current_yaw_calib = state
                self.get_logger().info(f"Set yaw calibration: {state}")
                return {"status": "ok", "yaw": state}
            else:
                return {"status": "error", "message": "Invalid yaw state"}

        @self.app.get("/override/{state}")
        async def override(state: str):
            if state.lower() in ["true", "false"]:
                self.joystick_override = (state.lower() == "true")
                self.get_logger().info(f"Set joystick_override: {self.joystick_override}")
                return {"status": "ok", "override": self.joystick_override}
            else:
                return {"status": "error", "message": "Invalid override state"}
        @self.app.get("/goal/{x}/{y}")
        async def goal(x: float, y: float):
            self.publish_goal(x, y)
            return {"status": "ok", "x": x, "y": y}
        


        @self.app.get("/start/robot_stack")
        async def start_robot_stack():
            script_path = "/home/soroush/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/actual_robot_localising_and_teleop.sh"
            try:
                # Run the script in the same terminal
                subprocess.Popen(["bash", script_path], cwd=os.path.dirname(script_path))
                self.get_logger().info(f"Started robot stack in the same terminal: {script_path}")
                return {"status": "ok", "message": "Robot stack launched in the same terminal"}
            except Exception as e:
                self.get_logger().error(f"Failed to launch script: {e}")
                return {"status": "error", "message": str(e)}

        @self.app.get("/followers/{n}")
        async def followers(n: str):
            if n in ["1", "2"]:   # only allow 1 or 2
                self.followers_number = n
                self.get_logger().info(f"Set followers_number: {n}")
                return {"status": "ok", "followers": n}
            else:
                return {"status": "error", "message": "Invalid followers number"}

def main(args=None):
    rclpy.init(args=args)
    node = WebCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
