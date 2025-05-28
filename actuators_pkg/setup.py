from setuptools import setup
import os
from glob import glob

package_name = 'actuators_pkg'
submodule = str(package_name + "/vesc_info_submodule") #added VESC info submodule -sebbb

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),	
	(os.path.join('share', package_name), glob('actuators_pkg/vesc_clientside.py'))
    ], #TODO - launch/config files
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'twist_vesc_LLM = actuators_pkg.twist_vesc_LLM:main',
	'twist_servo_LLM = actuators_pkg.twist_servo_LLM:main'
        ],
    },
)
