import os
from setuptools import find_packages, setup

package_name = 'manipulator_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'data'),['data/trajectory.csv'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hellhound',
    maintainer_email='hellhound@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_publisher = manipulator_planning.trajectory_publisher:main',
            'trajectory_service = manipulator_planning.trajectory_service:main',
            'trajectory_action_client = manipulator_planning.trajectory_action_client:main',
        ],
    },
)
