from setuptools import setup

package_name = 'turtle_bot_2'  # Cambia el 1 por tu número de grupo

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu.email@uniandes.edu.co',
    description='Taller 1 - Control de TurtleBot2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_bot_teleop = turtle_bot_2.turtle_bot_teleop:main',
            'turtle_bot_interface = turtle_bot_2.turtle_bot_interface:main',
            'turtle_bot_recorder = turtle_bot_2.turtle_bot_recorder:main',
            'turtle_bot_player = turtle_bot_2.turtle_bot_player:main',
        ],
    },
)
