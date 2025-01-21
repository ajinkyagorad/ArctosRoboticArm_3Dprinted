from setuptools import setup
import os
from glob import glob

package_name = 'arctos_moveit'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='MoveIt interface for Arctos arm',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = arctos_moveit.interface:main',
            'transform = arctos_moveit.transform:main',
            'objrec_publisher = arctos_moveit.moveo_objrec_publisher:main',
        ],
    },
)
