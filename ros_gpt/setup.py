from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_gpt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib/python3.10/site-packages', package_name), glob('config/*.json')),
        (os.path.join('lib/python3.10/site-packages', package_name), glob('config/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nao',
    maintainer_email='nao@todo.todo',
    description='This package provides an API tool to use the OpenAI API with GPT 3.5-Turbo.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_gpt_node = ros_gpt.ros_gpt:main',
            'start_gpt_node_run = ros_gpt.ros_gpt_running:main',
        ],
    },
)
