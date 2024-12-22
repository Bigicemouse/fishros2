import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xiaomi/fishros_src_learn/chart4/chapt4_ws/install/demo_python_service'
