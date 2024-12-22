import time
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import espeakng 
import threading

class NovelSubNode(Node):
    """
    NovelSubNode 类用于订阅小说内容，并使用文本转语音技术朗读小说。

    该节点从名为 "novel" 的主题订阅 String 类型的消息，接收到消息后，
    将其加入到队列中，然后在一个单独的线程中使用 espeakng 库将文本转换为语音输出。
    """
    def __init__(self, node_name):
        """
        初始化 NovelSubNode 节点。

        参数:
        - node_name: 节点的名称。

        初始化包括设置节点名称，创建文本行的队列，订阅 "novel" 主题，
        以及启动文本转语音的线程。
        """
        super().__init__(node_name)  # 初始化节点
             
        self.novels_queue_ = Queue()  # 创建队列来存储从网络下载的文本行 
        self.novel_subscription_ = self.create_subscription(String, "novel", self.novel_callback,10)
        self.speech_thread_ = threading.Thread(target=self.speak_thread) 
        self.speech_thread_.start()
        
    def novel_callback(self,msg):
        """
        novel_callback 函数作为订阅小说内容的回调函数。
        参数:
        - msg: 接收到的消息，其中包含小说的文本行。
        该函数将接收到的文本行加入到队列中，以供 speak_thread 函数处理。
        """
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        """
        speak_thread 函数在一个单独的线程中运行，负责将队列中的文本转换为语音输出。

        该函数创建一个 espeakng 的 Speaker 实例，设置其语音为中文，
        然后在一个循环中检查队列。如果有文本行，则将其转换为语音并播放；
        否则，线程休眠一秒钟。
        """
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
    """
    main 函数是程序的入口点。

    该函数初始化 ROS 2 环境，创建 NovelSubNode 节点实例，并命名为 "novel_read"，
    然后进入循环等待直到节点被关闭，最后关闭 ROS 2 环境。
    """
    rclpy.init()  # 初始化 ROS 2 
    node = NovelSubNode("novel_read")   # 创建节点实例，并命名为 "novel_read"

    rclpy.spin(node)
    rclpy.shutdown()  # 关闭 ROS 2