import rclpy

from rclpy.node import Node

class ParamsNode(Node):

    def __init__(self):
        super().__init__('params_node',
            parameter_overrides=[],
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
    

def main(args=None):
    rclpy.init(args=args)
    node = ParamsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
