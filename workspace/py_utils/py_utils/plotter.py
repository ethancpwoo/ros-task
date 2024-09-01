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
        self.set_params()
    
    def set_params(self):
        request = GetParameters.Request()
        request.names = ['x_goal', 'y_goal', 'theta_goal', 'd_x', 'd_theta']

        self.params_client.wait_for_service()

        res = self.params_client.call_async(request)
        res.add_done_callback(self.param_callback)

    def param_callback(self, future):
        result = future.result()
        param = result.values
        self.x_goal = param[0].double_value
        self.y_goal = param[1].double_value
        self.theta_goal = param[2].double_value
        self.d_x = param[3].double_value
        self.d_theta = param[4].double_value

    def conv_rad(self, z, w):
        return 2 * math.atan2(z, w)
    
    def record(self, msg):
        self.x_positions.append(msg.pose.pose.position.x)
        self.y_positions.append(msg.pose.pose.position.y)
        self.thetas.append(self.conv_rad(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    
    def get_status(self, msg):
        self.status = msg.data
        if self.status == 'ready':
            self.plot_data("Theta", "rad")
            self.plot_data("X", "m")
            self.plot_data("Y", "m")
            self.destroy_node()
        elif self.status == 'record_first':
            self.first_theta = self.thetas[-1]
        elif self.status == 'record_final':
            self.final_theta = self.thetas[-1]

    def plot_data(self, name, unit):
        plt.figure()

        plt.xlabel('Timestep')
        plt.ylabel(f'{name} in {unit}')
        plt.title(f'{name} vs Timestep, d_theta = {self.d_theta}, vel = {self.d_x}')

        if name == "Theta":
            plt.axhline(y=self.theta_goal, color='g', label=f'initial {unit}, delta = {abs(self.theta_goal - self.final_theta)}')
            plt.plot(self.thetas, label = f'{name}')
        elif name == "X":
            plt.axhline(y=self.x_goal, color='r', label=f'final dist x, delta = {abs(self.x_goal - self.x_positions[-1])} {unit}')
            plt.plot(self.x_positions, label = f'{name}')
        elif name == "Y":
            plt.axhline(y=self.y_goal, color='r', label=f'final dist y, delta = {abs(self.y_goal - self.y_positions[-1])} {unit}')
            plt.plot(self.y_positions, label = 'Y')

        plt.legend()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    plotterNode = PlotterNode()
    rclpy.spin(plotterNode)
    rclpy.shutdown()

if __name__ == '__main__':
    main()