import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'radar_web_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        ('share/' + package_name + '/web',
         [f for f in glob('web/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Radar Team',
    maintainer_email='rm@hnu.edu.cn',
    description='RM_radar_Cpp_2027 调试 Web UI (赛场地图/解析波/运行状态)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_ui_node = radar_web_ui.web_ui_node:main',
            'wave_mock_node = radar_web_ui.wave_mock_node:main',
        ],
    },
)
