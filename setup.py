from setuptools import find_packages, setup

package_name = 'strawberry_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qys',
    maintainer_email='qys@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = strawberry_vision.camera_publisher:main',
            'vision_to_arm = strawberry_vision.vision_to_arm:main',
            'robot_driver = strawberry_vision.robot_driver:main',
            'result_monitor = strawberry_vision.result_monitor:main',
        ],
    },
)
