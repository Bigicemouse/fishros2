from demo_python_pkg.person_node import PersonNode
import rclpy

class WriterNode(PersonNode):
    def __init__(self, node_name,name_value,age_value,book):
        print("Writer_init")
        super().__init__(node_name, name_value,age_value)  # 正确调用父类构造函数
        self.book = book


    def eat(self):
        #重写person的eat方法
        # print(f"{self.name},{self.age}岁,like write,{food_name}")   
        self.get_logger().info(f"{self.name},{self.age}岁,like eat{self.book}")





def main():
    rclpy.init()

    node = WriterNode('fishros',"zhangsan", 18, "Learn_ros2")  # 提供所有必要的参数
    node.eat()

    rclpy.spin(node)
    rclpy.shutdown()

