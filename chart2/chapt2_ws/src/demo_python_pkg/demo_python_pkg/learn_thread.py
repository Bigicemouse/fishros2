import threading
import requests

class Download:
    def download(self, url, callback_word_count):
        print(f"线程：{threading.get_ident()}, 开始下载：{url}")
        try:
            # 发送HTTP GET请求获取网页内容
            response = requests.get(url)
            # 检查请求是否成功
            response.raise_for_status()
            # 获取网页文本内容
            text = response.text  
            # 调用回调函数处理文本内容
            callback_word_count(url, text)
        except requests.RequestException as e:
            # 捕获并打印请求过程中发生的异常
            print(f"下载 {url} 时出错: {e}")

    def start_download(self, url, call_back_count):
        # 创建一个新的线程来执行download方法
        thread = threading.Thread(target=self.download, args=(url, call_back_count))
        # 启动线程
        thread.start()

def word_count(url, result):
    """
    simple funtion uesd for callback
    """
    # 计算文本中的单词数
    words = len(result.split())
    # 打印URL、单词数和文本的前50个字符
    print(f"{url}: 单词数 -> {words}, 前50个字符 -> {result[:50]}")

def main():
    # 创建Download类的实例
    download_1 = Download()
    # 定义要下载的URL列表
    urls = [
        "https://www.gutenberg.org/files/1342/1342-0.txt",  # 简爱
        "https://www.gutenberg.org/files/2701/2701-0.txt",  # 白鲸记
        "https://www.gutenberg.org/files/98/98-0.txt"      # 格列佛游记
    ]
    # 遍历URL列表并启动下载
    for url in urls:
        download_1.start_download(url, word_count)



