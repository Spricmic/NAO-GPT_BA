from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_stt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib/python3.10/site-packages', package_name), glob('temp_storage/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nao',
    maintainer_email='nao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_stt_node = ros_stt.ros_stt:main'
        ],
    },
)
