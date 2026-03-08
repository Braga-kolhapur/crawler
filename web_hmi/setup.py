from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'web_hmi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install templates
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*') if os.path.exists('config') else []),
        # Install scripts to lib/{package_name}/ for ROS2 launch compatibility
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=[
        'setuptools',
        'flask>=2.3',
        'flask-socketio>=5.3',
        'Pillow>=9.0',
        'pyyaml>=6.0',
    ],
    zip_safe=True,
    maintainer='Pratik',
    maintainer_email='pratik@todo.todo',
    description='Mobile-first web interface for monitoring and controlling the coverage robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi_server = web_hmi.hmi_server:main',
        ],
    },
)
