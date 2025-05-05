from setuptools import find_packages, setup

package_name = 'ur5_wave'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bwilab',
    maintainer_email='madhanimansi56@gmail.com',
    description='Make UR5 robot wave',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wave_node = ur5_wave.wave:main',
        ],
    },
)
