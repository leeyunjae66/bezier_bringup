from setuptools import setup
from glob import glob

package_name = 'bezier_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # bringup 전용 (파이썬 모듈 없음)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        # ★ config 폴더의 모든 파일(yaml/pgm/png 등)을 함께 설치
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Bringup for rviz2 + nav2 map_server(lifecycle) + path_planner3/path_generator',
    license='MIT',
    entry_points={'console_scripts': []},
)
