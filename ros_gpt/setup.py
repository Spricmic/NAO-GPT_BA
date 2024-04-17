from setuptools import find_packages, setup

package_name = 'ros_gpt'

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
    maintainer='nao',
    maintainer_email='nao@todo.todo',
    description='This package provides an API tool to use the OpenAI API with GPT 3.5-Turbo.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_gpt_node = ros_gpt.gpt_publisher:main',
            'start_gpt_node_run = ros_gpt.gpt_publisher_running:main',
        ],
    },
)
