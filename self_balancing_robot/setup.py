from setuptools import find_packages, setup

package_name = 'self_balancing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/hw.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='yash_soni_in11',
    maintainer_email='yash_soni_in11@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_bridge = self_balancer_nodes.imu_bridge:main',
            'balance_controller = self_balancer_nodes.balance_controller:main',
            'motor_comm = self_balancer_nodes.motor_comm:main',
        ],
    },
)
