import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import tf2_ros
from ros2_msgs.srv import Location

class PlacesNode(Node):
    def __init__(self):
        super().__init__('places')
        self.saved_positions = {}  # Data structure to store positions

        # Publisher for visualizing markers
        self.marker_publisher = self.create_publisher(Marker, 'saved_positions_markers', 10)

        # Services
        #self.memorize_position_service = self.create_service(Empty, 'memorize_position', self.memorize_position_callback)
        self.memorize_position_service = self.create_service(Location, 'memorize_position', self.memorize_position_callback)
        self.clear_positions_service = self.create_service(Empty, 'clear_positions', self.clear_positions_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.counter = 0

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
        self.saved_positions[position_name] = request.location_name
        self.create_marker(position_name, current_pose)
        #self.make_breadcrumb()
        for name in self.saved_positions.keys():
            self.get_logger().info(f'Memorized position: {name} at {self.saved_positions[name]}')

        return Location.Response()

    def clear_positions_callback(self, request, response):
        # Clear all saved positions
        self.get_logger().info('Clearing all saved positions')
        self.saved_positions.clear()
        self.clear_markers()
        return Empty.Response()
    
    def make_breadcrumb(self):
        # Make the marker.
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.id = self.counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the size of the sphere.  It can be oblate, so we set three scales.
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color.
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Get the current pose of the robot, in the map frame.
        marker.pose = self.get_current_pose(marker.header.frame_id)

        self.counter += 1

        return marker

    def get_current_pose(self, frame_id):
        # Build a stamped pose for the base link origin.  If we don't set the time in the header,
        # then we get the latest TF update.
        origin = PoseStamped()
        origin.header.frame_id = 'base_link'

        # Set the position.
        origin.pose.position.x = 0.0
        origin.pose.position.y = 0.0
        origin.pose.position.z = 0.0

        # Set an arbitrary orientation.
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0

        # Get the transform to the map frame.  This will cause an exception if it fails, but we'll
        # deal with that in the calling function.
        new_pose = self.tf_buffer.transform(origin, frame_id, rclpy.duration.Duration(seconds=1))

        return new_pose.pose

    def create_marker(self, name, pose):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        # Set the size of the sphere.  It can be oblate, so we set three scales.
        marker.scale.x = 0.1
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
        for name in self.saved_positions.keys():
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.DELETE
            marker.id = len(self.saved_positions)
            self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PlacesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
