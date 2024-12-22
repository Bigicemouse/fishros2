import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self,node_name,name_value:str,age_value:int):
        print("PersonNode_init")
        super().__init__(node_name)
        self.name = name_value
        self.age = age_value

    def eat(self,food_name:str):
        # print(f"{self.name},{self.age}岁,like eat{food_name}")
        self.get_logger().info(f"{self.name},{self.age}岁,like eat{food_name}")

def main():
    rclpy.init()
    node = PersonNode('ros2',"zhangsan",18)       
    node.eat("fishros")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()    