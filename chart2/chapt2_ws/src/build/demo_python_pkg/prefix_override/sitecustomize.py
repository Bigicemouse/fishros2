import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xiaomi/chart2/chapt2_ws/src/install/demo_python_pkg'
