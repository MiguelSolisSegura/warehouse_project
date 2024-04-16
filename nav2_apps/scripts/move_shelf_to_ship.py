import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from attach_service.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Client for shelf lifting service
class ClientAsync(Node):
    def __init__(self):
        super().__init__('go_to_loading')
        self.client = self.create_client(GoToLoading, 'approach_shelf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        req = GoToLoading.Request()
        self.future = self.client.call_async(req)

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        self.global_publisher_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_publisher_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.timer = self.create_timer(1, self.publish_polygon)

        self.square_size = 0.9
        self.half_size = self.square_size / 2 

        self.circle_shape = [
            Point32(x=3.050741195678711, y=-1.0888712406158447, z=0.0),
            Point32(x=3.066046714782715, y=-0.9825969934463501, z=0.0),
            Point32(x=3.0402321815490723, y=-0.8885296583175659, z=0.0),
            Point32(x=2.9803848266601562, y=-0.8115015625953674, z=0.0),
            Point32(x=2.895615339279175, y=-0.7632395029067993, z=0.0),
            Point32(x=2.7798526287078857, y=-0.7447766065597534, z=0.0),
            Point32(x=2.6857852935791016, y=-0.770591139793396, z=0.0),
            Point32(x=2.608757257461548, y=-0.8304387331008911, z=0.0),
            Point32(x=2.560495138168335, y=-0.9152079820632935, z=0.0),
            Point32(x=2.542032241821289, y=-1.030970811843872, z=0.0),
            Point32(x=2.5678467750549316, y=-1.1250381469726562, z=0.0),
            Point32(x=2.6276943683624268, y=-1.20206618309021, z=0.0),
            Point32(x=2.712463617324829, y=-1.2503283023834229, z=0.0),
            Point32(x=2.828226327896118, y=-1.2687911987304688, z=0.0),
            Point32(x=2.9222936630249023, y=-1.2429766654968262, z=0.0),
            Point32(x=2.999321937561035, y=-1.183129072189331, z=0.0),
        ]

        self.square_shape = [
            Point32(x=-self.half_size, y=self.half_size, z=0.0), 
            Point32(x=self.half_size, y=self.half_size, z=0.0),    
            Point32(x=self.half_size, y=-self.half_size, z=0.0),   
            Point32(x=-self.half_size, y=-self.half_size, z=0.0)   
        ]

    def publish_polygon(self, mode):
        polygon_msg = Polygon()
        if (mode == 'circle'):
            polygon_msg.points = self.circle_shape
        elif (mode == 'square'):
            polygon_msg.points = self.square_shape
        else:
            self.get_logger().info(f'Invalid shape type {mode}')
            return None
        self.global_publisher_.publish(polygon_msg)
        self.local_publisher_.publish(polygon_msg)
        self.get_logger().info(f'Publishing polygon footprint of type {mode}')
        return None


# Shelf positions for picking
shelf_positions = {
    "init": [0.0, 0.0, 0.0, 1.0],
    "loading_position": [5.68239, -0.0438051, -0.719685, 0.694301],
    }

# Shipping destination for picked products
shipping_destinations = {

}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''


def main():
    ####################
    request_item_location = 'loading_position'
    request_destination = ''
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions['init'][0]
    initial_pose.pose.position.y = shelf_positions['init'][1]
    initial_pose.pose.orientation.z = shelf_positions['init'][2]
    initial_pose.pose.orientation.w = shelf_positions['init'][3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Calling shelf lifting service.')

        # Instance the service client for shelf lifting
        client = ClientAsync()
        client.send_request()
        # Instance the footprint publisher
        footprint_publisher = PolygonPublisher()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'Result of service call: {response.complete}')
                    footprint_publisher.publish_polygon('square')
                break

        client.destroy_node()

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()