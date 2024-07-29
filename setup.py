from setuptools import setup

package_name = 'voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Voice control node',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_node = voice_control.voice_command_node:main',
            'command_mapper_node = voice_control.command_mapper_node:main'
        ],
    },
)
