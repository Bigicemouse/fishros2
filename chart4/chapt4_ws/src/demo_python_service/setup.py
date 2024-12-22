# 导入必要的模块
from setuptools import find_packages, setup
from glob import glob

# 定义包的名称
package_name = 'demo_python_service'

# 调用setup函数进行包的配置和安装
setup(
    # 设置包的名称
    name=package_name,
    # 设置包的版本
    version='0.0.0',
    # 查找并包含所有非测试的包
    packages=find_packages(exclude=['test']),
    # 设置数据文件的安装路径和文件列表
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/resource", ['resource/default.jpg','resource/test1.jpg']),
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
    ],
    # 设置安装依赖项
    install_requires=['setuptools'],
    # 设置是否安全地使用zip
    zip_safe=True,
    # 设置包的维护者名称
    maintainer='mzebra',
    # 设置包的维护者邮箱
    maintainer_email='mzebra@foxmail.com',
    # 设置包的描述（TODO提示未完成）
    description='TODO: Package description',
    # 设置包的许可证类型
    license='Apache-2.0',
    # 设置测试依赖项
    tests_require=['pytest'],
    # 设置控制台脚本的入口点
    entry_points={
        'console_scripts': [
            'learn_face_detect=demo_python_service.learn_face_detect:main',
            'face_detect_node=demo_python_service.face_detect_node:main',
            'face_detect_client_node=demo_python_service.face_detect_client_node:main',
        ],
    },
)