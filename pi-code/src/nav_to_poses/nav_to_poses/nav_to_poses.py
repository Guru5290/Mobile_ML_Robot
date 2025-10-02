#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import sys
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import subprocess, signal
import time
from std_msgs.msg import String,Float64MultiArray
from rclpy.node import Node
from py_trees.blackboard import Blackboard
import subprocess



def print_same_line(msg1, msg2):
    sys.stdout.write("\033[F\033[F")  # Move cursor 2 lines up
    sys.stdout.write(f"{msg1}\n")
    sys.stdout.write(f"{msg2}\n")
    sys.stdout.flush()


# ---- Task behaviours ----

#Implementing loading and offloading tasks with colour detection
loaded_colour=None
class Loading(py_trees.behaviour.Behaviour):
    def __init__(self, name="Loading"):
        super().__init__(name)
        self.executed = False
        self.node = None
        self.colour = None
        self.process = None
        self.start_time = None
        self.timeout = 15  # seconds to run detector
        self.blackboard = Blackboard()

    def update(self):
        if self.executed:
            return py_trees.common.Status.SUCCESS

        # Step 1: Start process if not running
        if self.process is None:
            # self.process = subprocess.Popen(
            #     ["ros2", "run", "colour_door_controller", "colour_detector","--ros-args","-p","IP:=10.122.180.67"], #b - Fundi
            #     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid #start new process group
            # )
            self.process = subprocess.Popen(
                ["ros2", "run", "colour_door_controller", "colour_detector","--ros-args","-p","IP:=10.226.56.67"], #Kabbage - Gareth 
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid #start new process group
            )
            # self.process = subprocess.Popen(
            #     ["ros2", "run", "colour_door_controller", "colour_detector","--ros-args","-p","IP:=192.168.0.112"], #Gamefield
            #     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid #start new process group
            # )
            self.start_time = time.time()
            print(f"{self.name}: Started colour detector for loading...")
            # time.sleep(5)
            
            return py_trees.common.Status.RUNNING

        # Step 2: Wait for timeout to expire
        if time.time() - self.start_time < self.timeout:
            return py_trees.common.Status.RUNNING

        # Step 3: Stop detector after timeout
        self._stop_detector()
        

        # Take the last detected colour from callback
        if self.colour:
            self.blackboard.loading_colour = self.colour
            global loaded_colour 
            loaded_colour = self.colour
            print(f"{self.name}: Stored loaded colour = {loaded_colour}")
            
        else:
            print(f"{self.name}: No colour detected, defaulting to None")
        
        self.executed = True     
        return py_trees.common.Status.SUCCESS

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node:
            self.node = node
            self.sub = node.create_subscription(
                String, "/detected_colour", self.colour_callback, 10
            )

    def colour_callback(self, msg):
        self.colour = msg.data.strip().lower()
        print(f"{self.name}: Detected colour: {self.colour}")

    def _stop_detector(self):
        if self.process:            
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)  #send Ctrl+C to the process group
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"{self.name}: Colour detector did not stop in time, killing...")
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            print(f"{self.name}: Colour detector stopped after loading")
            self.process = None

class OffLoading(py_trees.behaviour.Behaviour):
    def __init__(self, name="OffLoading"):
        super().__init__(name)
        self.executed = False
        self.node = None
        self.servo_pub = None
        self.blackboard = Blackboard()

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node:
            self.node = node
            self.servo_pub = node.create_publisher(Float64MultiArray, "/servo_controller/commands", 10)
            print(f"{self.name}: Ready to publish to servo")

    def update(self):
        if self.executed:
            return py_trees.common.Status.SUCCESS

        # Get colour detected during loading
        # loading_colour = self.blackboard.loading_colour

        # Infer expected colour from waypoint name (e.g. "offloading_blue")
        expected_colour = None
        if "blue" in self.name.lower():
            expected_colour = "blue"
        elif "red" in self.name.lower():
            expected_colour = "red"

        if loaded_colour is None:
            print(f"{self.name}: No loading colour stored, skipping offloading")
            self.executed = True
            return py_trees.common.Status.SUCCESS

        if expected_colour and loaded_colour == expected_colour:
            print(f"{self.name}: Match found ({loaded_colour}), opening servo...")
            self._publish_servo(90.0)  # OPEN
            time.sleep(10)  # keep open
            print(f"{self.name}: Closing servo after 20s")
            self._publish_servo(170.0)  # CLOSE
        else:
            print(f"{self.name}: Mismatch (loading={loaded_colour}, expected={expected_colour}), no action")

        self.executed = True
        return py_trees.common.Status.SUCCESS

    def _publish_servo(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        self.servo_pub.publish(msg)
        print(f"{self.name}: Servo set to {value}")

class Potato_Detector(py_trees.behaviour.Behaviour):
    def __init__(self, name="Potato_Detector"):
        super().__init__(name)
        self.process = None
        self.start_time = None
        self.timeout = 15  
        self.phase = "start"

    def update(self):
        if self.phase == "start":
            launch_cmd = [
                "ros2", "launch",
                "rdj2025_potato_disease_detection",   # package
                "potato_detection.launch.py"          # launch file
            ]

            try:
                self.process = subprocess.Popen(launch_cmd, preexec_fn=os.setsid) #start new process group
                print(f"{self.name}: Started potato detection launch file")
            except FileNotFoundError:
                print(f"{self.name}: ros2 launch not found")
                return py_trees.common.Status.FAILURE

            self.start_time = time.time()
            self.phase = "running"
            return py_trees.common.Status.RUNNING

        elif self.phase == "running":
            if time.time() - self.start_time < self.timeout:
                return py_trees.common.Status.RUNNING
            else:
                self.phase = "stopping"
                return py_trees.common.Status.RUNNING

        elif self.phase == "stopping":
            if self.process:
                
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGINT)  #send Ctrl+C to the process group
                    self.process.wait(timeout=5)
                    print(f"{self.name}: Potato detection stopped after {self.timeout}s")
                except subprocess.TimeoutExpired:
                    print(f"{self.name}: Potato detection did not stop in time, killing...")
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.phase = "done"
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS


# ---- Nav2 MoveToPosition ----
class MoveToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_x, target_y, yaw=0.0):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = yaw
        self.action_client = None
        self.goal_future = None
        self.result_future = None
        self.completed = False
        self.node = None

    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.node = node
            self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            print(f"{self.name}: Setup completed")

    def update(self):
        if self.completed:
            return py_trees.common.Status.SUCCESS

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            print(f"{self.name}: Waiting for Nav2 action server...")
            return py_trees.common.Status.RUNNING

        if self.goal_future is None:
            # Build goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = make_pose(self.target_x, self.target_y, self.target_yaw)

            print(f"{self.name}: Sending NavigateToPose goal ({self.target_x}, {self.target_y}, yaw={self.target_yaw})")
            self.goal_future = self.action_client.send_goal_async(goal_msg)

            def goal_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    print(f"{self.name}: Goal rejected by Nav2")
                    self.completed = True
                else:
                    print(f"{self.name}: Goal accepted, waiting for result...")
                    self.result_future = goal_handle.get_result_async()
                    self.result_future.add_done_callback(self.result_cb)

            self.goal_future.add_done_callback(goal_response_cb)

        return py_trees.common.Status.RUNNING

    def result_cb(self, future):
        result = future.result().result
        print(f"{self.name}: Nav2 result received: {result}")
        self.completed = True


# ---- Helper for goals ----
def make_pose(x, y, yaw=0.0, frame="map"):
    import math
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Convert yaw to quaternion
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    print(f"Goal created: x={x}, y={y}, yaw={yaw}, frame={frame}")
    return pose


# ---- YAML Loader ----
def load_goals_from_yaml(filename="nav_goals.yaml"):
    # Path of this file (app.py)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up one level to gazebo_ignition_fortress/
    project_root = os.path.dirname(current_dir)
    # Build path to src/ppp_bot/config/nav_goals.yaml
    file_path = os.path.join(project_root, "config", filename)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"YAML file not found at: {file_path}")

    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    return data["goals"]


# ---- Tree Definition ----
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
    yaml_dir = os.path.join(root_dir, 'src','nav_to_poses', 'nav_to_poses','config', 'nav_goals.yaml')
    goals = load_goals_from_yaml(yaml_dir)

    children = []
    for g in goals:
        move = MoveToPosition(
            g["name"],
            g["x"],
            g["y"],
            g.get("yaw", 0.0)
        )
        
        children.append(move)

        # Example: add a hello/hi task after each move
        if "potato" in g["name"].lower():
            children.append(Potato_Detector(f"Image_detection_in_{g['name']}"))
        elif "loading_zone" in g["name"].lower():
            children.append(Loading(f"Loading_in_{g['name']}"))
        elif "offloading" in g["name"].lower():
            children.append(OffLoading(f"Offloading_in__{g['name']}"))
        else:
            #children.append(Waypoint(f"Reached_{g['name']}"))
            print(f"Reached_{g['name']}")

    root.add_children(children)
    return root


# ---- Main ----
def main():
    rclpy.init()

    root = create_root()
    node = py_trees_ros.trees.BehaviourTree(root)

    try:
        node.setup(timeout=10.0, node=node.node)  # Pass the ROS node to behaviors

        print("Behavior tree starting...")
        tree_completed = False

        def tick_tree():
            nonlocal tree_completed
            if tree_completed:
                return
            node.tick_tock(period_ms=100)
            # for i, child in enumerate(node.root.children):
            #     if child.status in (py_trees.common.Status.SUCCESS,
            #                         py_trees.common.Status.RUNNING):
            #         print(f"  Child {i} ({child.name}): {child.status.name}")
            if node.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior tree completed successfully!")
                tree_completed = True
            elif node.root.status == py_trees.common.Status.FAILURE:
                print("Behavior tree failed!")
                tree_completed = True

        timer = node.node.create_timer(0.5, tick_tree)
        print("Behavior tree is running...")
        rclpy.spin(node.node)
    except RuntimeError as e:
        print(f"Setup failed: {e}")
        print("Make sure Nav2 is running: ros2 launch nav2_bringup navigation_launch.py")
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# 