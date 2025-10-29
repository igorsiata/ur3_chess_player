from setuptools import find_packages, setup

package_name = 'computer_opponent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igorsiata',
    maintainer_email='igor.siata33@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stockfish_player = computer_opponent.stockfish_player:main',
            'human_player = computer_opponent.human_player:main',
            'pose_listener_moveit2 = my_robot_control.pose_listener_moveit2:main',
            'simple_client = computer_opponent.simple_client:main',
            'test_ik = computer_opponent.test_ik:main'
        ],
    },
)
