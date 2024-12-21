import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)  # 初始化节点
        self.get_logger().info(f"{node_name}, Starting")  # 记录日志，表示节点已启动       
        self.novels_queue_ = Queue()  # 创建队列来存储从网络下载的文本行
        # 创建一个发布者对象，用于发布 String 类型的消息到名为 "novel" 的主题上，队列大小为 10
        self.novel_publisher_ = self.create_publisher(String, "novel", 10)
        # 创建一个定时器，每隔 5 秒调用一次 timer_callback 方法
        self.create_timer(5, self.timer_callback)
    def timer_callback(self):
        # 检查队列是否为空
        if not self.novels_queue_.empty():
            line = self.novels_queue_.get()  # 从队列中取出一行文本         
            msg = String()  # 创建一个 String 类型的消息对象
            msg.data = line  # 将取出的文本行赋值给消息的数据字段          
            # 使用发布者发布消息
            self.novel_publisher_.publish(msg)       
            # 记录日志，表示已发布的消息内容
            self.get_logger().info(f'发布了：{msg.data}')
    def download(self, url):
        print(f"开始下载：{url}")  # 打印日志，表示开始下载指定 URL 的内容      
        try:
            response = requests.get(url)  # 发送 HTTP GET 请求获取网页内容
            response.encoding = "utf-8"
            text = response.text  # 获取网页的文本内容  
            # 记录日志，表示下载完成及文本长度
            self.get_logger().info(f"下载{url}, 共{len(text)}字")   
            # 将文本按行分割并将每一行放入队列中
            for line in text.splitlines():
                self.novels_queue_.put(line)

        except requests.RequestException as e:
            # 捕获并打印请求过程中发生的异常
            print(f"下载 {url} 时出错: {e}")

def main():
    rclpy.init()  # 初始化 ROS 2 
    node = NovelPubNode("novel_pub")   # 创建节点实例，并命名为 "novel_pub"
    # 下载小说文本
    node.download("https://www.gutenberg.org/files/2701/2701-0.txt")
    # 运行节点，直到手动停止
    rclpy.spin(node)
    rclpy.shutdown()  # 关闭 ROS 2


