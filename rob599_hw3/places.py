from asyncio import sleep
import rclpy
import os
import json
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from ros2_msgs.srv import Location, Clear, Save, Load, KnockKnock
from ros2_msgs.action import GoTo, Patrol


class PlacesNode(Node):
    def __init__(self):
        super().__init__('places')
        self.saved_positions = {}  # Data structure to store positions

        # Publisher for visualizing markers
        self.marker_publisher = self.create_publisher(Marker, 'saved_positions_markers', 10)

        # Services
        self.memorize_position_service = self.create_service(Location, 'memorize_position', self.memorize_position_callback)
        self.clear_positions_service = self.create_service(Clear, 'clear_positions', self.clear_positions_callback)

        self.save_positions_service = self.create_service(Save, 'save_positions', self.save_positions_callback)
        self.load_positions_service = self.create_service(Load, 'load_positions', self.load_positions_callback)

        self._action_server = ActionServer(self, GoTo, 'go_to', self.execute_go_to_callback)
        self.patrol_action_server = ActionServer(self, Patrol, 'patrol', self.execute_patrol_callback)
        self.knock_knock_service = self.create_service(KnockKnock, 'knock_knock', self.knock_knock_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.counter = 0

        self.navigator = BasicNavigator()

    def get_current_robot_pose(self):
        try:
            # Get the transform from the "base_link" frame to the "map" frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Create a PoseStamped message to represent the current robot pose
            current_pose = PoseStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = transform.header.stamp
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation

            return current_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get current robot pose: %s' % str(e))
            return None

    def memorize_position_callback(self, request, response):
        # Memorize the current pose with the provided name
        self.get_logger().info('Memorizing current position')
        position_name = request.location_name 
        current_pose = self.get_current_robot_pose()  
        self.saved_positions[position_name] = current_pose.pose
        self.create_marker(position_name, current_pose)
        for name in self.saved_positions.keys():
            self.get_logger().info(f'Memorized position: {name} at ( {current_pose.pose.position.x}, {current_pose.pose.position.y} )')

        return Location.Response()

    def clear_positions_callback(self, request, response):
        if request.reset:
            # Create a marker for deleting all markers
            self.saved_positions = {}  # Clear data structure that stored positions
            delete_marker = Marker()
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.header.frame_id = 'map'  # Use the same frame as your other markers
            delete_marker.ns = 'places'  # Use the same namespace as your other markers
            delete_marker.action = Marker.DELETEALL  # Action to delete all markers

            self.marker_publisher.publish(delete_marker)  # Publish the delete marker

            response.done = True
            self.get_logger().info('All positions cleared and markers deleted.')
            response.done = True
        else:
            response.done = False
        return response
    
    def knock_knock_callback(self, request, response):
        # Stop any ongoing action
        self.navigator.cancelTask()

        # Load positions from the HW3.txt file
        load_request = Load.Request()
        load_request.filename = 'places.txt'
        self.load_positions_callback(load_request, Load.Response())

        # Send the robot to the front door
        if 'front_door' in self.positions:
            x = self.positions['front_door'].position.x
            y = self.positions['front_door'].position.y
            self.go_to(x, y)
            self.get_logger().info("Robot is moving to the front door")

            # Wait for the robot to reach the front door
            while not self.navigator.isTaskComplete():
                pass

            self.get_logger().info("Knock Knock. Who's there? AI. AI who? AI haven't heard a better knock-knock joke than this!")
            sleep(5)
        else:
            self.get_logger().error("Front door position not found")

        return response
    
    async def execute_go_to_callback(self, goal_handle):
        goal = goal_handle.request
        if goal.place not in self.saved_positions:
            goal_handle.abort()
            self.get_logger().info(f"Place {goal.place} does not exist")
            return GoTo.Result(success=False, message="Place does not exist")

        # Code to move the robot to the place goes here
        for name, pose in self.saved_positions.items():
            if name == goal.place:
                self.get_logger().info(f"Going to: {pose.position.x}, {pose.position.y}")
                self.go_to(pose.position.x, pose.position.y)
            

        goal_handle.succeed()
        return GoTo.Result(success=True, message="Successfully moved to the place")
    
    async def execute_patrol_callback(self, goal_handle):
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()
        for name, pose in self.saved_positions.items():
            x = pose.position.x
            y = pose.position.y
            self.go_to(x, y)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                self.get_logger().info(f'Estimated time of patrol: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds')
            if not goal_handle.is_active:
                result.success = False
                result.message = "Patrol was cancelled"
                return result
        result.success = True
        result.message = "Patrol complete"
        return result
    
    def go_to(self, x, y):
        # Set a goal pose.  This is the starting location from our simulation, with an unspecified orientation.
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        # Just go to that pose.  This call does not block.  Sending this preempts other tasks. 
        # If you want to wait for the task to complete, you can use the isTaskComplete() method.
        self.navigator.goToPose(goal_pose)

        # Loop until we're done
        while not self.navigator.isTaskComplete():
            # Retrieve feedback on how we're doing.
            feedback = self.navigator.getFeedback()

            self.get_logger().info(f'Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds')

            # We can cancel tasks.
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()

        # Did we succeed?
        match self.navigator.getResult():
            case TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            case TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            case TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
            case _:
                self.get_logger().info('Goal has an invalid return status!')

    def save_positions_callback(self, request, response):
        # Save positions to a file
        try:
            if not request.filename:
                raise ValueError("Filename is not provided")

            os.makedirs('resources', exist_ok=True)  # Create the directory if it does not exist

            for name, pose in self.saved_positions.items():
                self.get_logger().info(f"Pose: {name}: {pose}")

            # Convert Pose objects to dictionaries
            positions_dict = {name: {'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
                                    'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y,
                                                    'z': pose.orientation.z, 'w': pose.orientation.w}}
                            for name, pose in self.saved_positions.items()}

            with open(os.path.join('resources', request.filename), 'w') as f:
                json.dump(positions_dict, f)

            response.success = True
            self.get_logger().info("Positions saved")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to save positions: {e}")
        return response

    def load_positions_callback(self, request, response):
    # Load positions from a file
        try:
            with open(os.path.join('resources', request.filename), 'r') as f:
                positions_dict = json.load(f)

            # Convert dictionaries back to Pose objects
            self.positions = {name: Pose(position=Point(**pose_dict['position']),
                                        orientation=Quaternion(**pose_dict['orientation']))
                            for name, pose_dict in positions_dict.items()}

            response.success = True
            self.get_logger().info("Positions loaded")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to load positions: {e}")
        return response

    def create_marker(self, name, pose):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        # Set the size of the sphere.  It can be oblate, so we set three scales.
        marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color.
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.id = len(self.saved_positions)  # Unique ID for each marker
        marker.text = name
        self.marker_publisher.publish(marker)

    def clear_markers(self):
        # Clear all markers by republishing empty markers with delete action
        delete_marker = Marker()
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.header.frame_id = 'map'  # Use the same frame as your other markers
        delete_marker.ns = 'places'  # Use the same namespace as your other markers
        delete_marker.action = Marker.DELETEALL  # Action to delete all markers
        self.marker_publisher.publish(delete_marker)  # Publish the delete marker
        self.get_logger().info(f'delete_marker: {delete_marker}')

        

def main(args=None):
    rclpy.init(args=args)
    node = PlacesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
