from setuptools import setup, find_packages

package_name = 'create_behavior_tree'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/behavior_tree_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/behavior_tree_config.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Behavior tree for Create2 robot sensor-based autonomous behavior',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'refactored_behavior_tree_node = create_behavior_tree.refactored_behavior_tree_node:main',
        ],
    },
)
