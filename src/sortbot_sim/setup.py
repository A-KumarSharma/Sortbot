from glob import glob  
from setuptools import find_packages, setup


setup(
    name='sortbot_sim',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/sortbot_sim']),
        ('share/sortbot_sim', ['package.xml']),
        *[('share/sortbot_sim/'+d, glob (d+'/*')) for d in ('launch','urdf','config')]],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
        ],
    },
)
