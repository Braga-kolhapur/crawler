from setuptools import setup

package_name = 'simple_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='maintainer',
    author_email='maintainer@example.com',
    description='Simple keyboard teleop publishing TwistStamped',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop = simple_teleop.teleop:main'
        ],
    },
)
