import time
import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue
import espeakng 
import threading


class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)  # 初始化节点
             
        self.novels_queue_ = Queue()  # 创建队列来存储从网络下载的文本行 
        self.novel_subscription_ = self.create_subscription(String, "novel", self.novel_callback,10)
        self.speech_thread_ = threading.Thread(target=self.speak_thread) 
        self.speech_thread_.start()
        
    def novel_callback(self,msg):
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = "zh"
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'正在阅读{text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)    

def main():
    rclpy.init()  # 初始化 ROS 2 
    node = NovelSubNode("novel_read")   # 创建节点实例，并命名为 "novel_read"

    rclpy.spin(node)
    rclpy.shutdown()  # 关闭 ROS 2


