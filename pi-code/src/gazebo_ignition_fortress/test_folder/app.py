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


def print_same_line(msg1, msg2):
    sys.stdout.write("\033[F\033[F")  # Move cursor 2 lines up
    sys.stdout.write(f"{msg1}\n")
    sys.stdout.write(f"{msg2}\n")
    sys.stdout.flush()


# ---- Task behaviours ----
class PrintHello(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHello"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hello")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class PrintHi(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHi"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hi")
            self.executed = True
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
    file_path = os.path.join(project_root, "src", "ppp_bot", "config", filename)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"YAML file not found at: {file_path}")

    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    return data["goals"]


# ---- Tree Definition ----
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # Load goals from YAML
    goals = load_goals_from_yaml("nav_goals.yaml")

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
        if "hello" in g["name"].lower():
            children.append(PrintHello(f"Task_after_{g['name']}"))
        else:
            children.append(PrintHi(f"Task_after_{g['name']}"))

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
            for i, child in enumerate(node.root.children):
                if child.status in (py_trees.common.Status.SUCCESS,
                                    py_trees.common.Status.RUNNING):
                    print(f"  Child {i} ({child.name}): {child.status.name}")
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
