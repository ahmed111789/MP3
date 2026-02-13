from setuptools import find_packages, setup

package_name = 'intruder_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/alert_example.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/alert.yaml'
        ]),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='batata',
    maintainer_email='ahmedkaffel2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'intruder_detection = intruder_detection.intruder_detection:main'
        ],
    },
)
