import math
import rclpy

from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class PlotterNode(Node):

    def __init__(self):
        super().__init__('plotter_node')

        self.x_positions = []
        self.y_positions = []
        self.thetas = []
        self.first_theta = 0
        self.final_theta = 0
        self.status = False

        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.record, 10)
        self.status_subscriber = self.create_subscription(String, 'status', self.get_status, 10)
        self.params_client = self.create_client(GetParameters, '/params_node/get_parameters')

        request = GetParameters.Request()
        request.names = ['x_goal']

        self.client.wait_for_service()

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)
    
    def param_callback():
        
    
    def conv_rad(self, z, w):
        return 2 * math.atan2(z, w)
    
    def record(self, msg):
        self.x_positions.append(msg.pose.pose.position.x)
        self.y_positions.append(msg.pose.pose.position.y)
        self.thetas.append(self.conv_rad(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    
    def get_status(self, msg):
        self.status = msg.data
        if self.status == 'ready':
            self.plot_x_data()
            self.plot_y_data()
            self.destroy_node()
        elif self.status == 'record_first':
            self.first_theta = self.thetas[-1]
        elif self.status == 'record_final':
            self.final_theta = self.thetas[-1]

    def plot_theta_data(self):
        plt.figure()

        plt.xlabel('Timestep')
        plt.ylabel('Theta (rad)')
        plt.title('Theta vs Timestep, d_theta = 0.2')

        plt.axhline(y=math.pi, color='r', label=f'final turn angle, delta = {abs(math.pi/4 - self.first_theta)}')
        plt.axhline(y=math.pi/4, color='g', label=f'initial turn angle, delta = {abs(math.pi - self.final_theta)}')

        plt.legend()
        plt.plot(self.thetas, label = 'Theta')
        plt.show()

    def plot_x_data(self):

        plt.figure()

        plt.xlabel('Timestep')
        plt.ylabel('x coordinate (m)')
        plt.title('X vs Timestep, d_theta = 0.2, vel = 0.5')

        plt.axhline(y=self.x_goal, color='r', label=f'final dist x, delta = {abs(self.x_goal - self.x_positions[-1])} m')

        plt.legend()
        plt.plot(self.thetas, label = 'X')
        plt.show()

    def plot_y_data(self):

        plt.figure()

        plt.xlabel('Timestep')
        plt.ylabel('y coordinate (m)')
        plt.title('Y vs Timestep, d_theta = 0.2, vel = 0.5')

        plt.axhline(y=3, color='r', label=f'final dist x, delta = {abs(3 - self.y_positions[-1])} m')

        plt.legend()
        plt.plot(self.thetas, label = 'Y')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    plotterNode = PlotterNode()
    rclpy.spin(plotterNode)
    rclpy.shutdown()