import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ros2_msgs.action import GoTo

class GoToClientNode(Node):
    def __init__(self):
        super().__init__('go_to_client')
        self._client = ActionClient(self, GoTo, 'go_to')

    def send_goal(self, place):
        goal_msg = GoTo.Goal()
        goal_msg.place = place
        self._client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToClientNode()
    while rclpy.ok():
        place = input("Enter a place: ")
        node.send_goal(place)
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()