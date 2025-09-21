from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bug1_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registro do pacote
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Instala arquivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Instala arquivos de mundo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luisvviana',
    maintainer_email='luisvviana@todo.todo',
    description='Pacote com launch e mundos customizados do TurtleBot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aqui definimos o nome do executável ROS 2
            'bug1_node = bug1_turtle.bug1_node:main',
        ],
    },
)
