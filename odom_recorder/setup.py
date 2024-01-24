from setuptools import find_packages, setup

package_name = 'odom_recorder'

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
    maintainer='angel',
    maintainer_email='angeloob11@gmail.com',
    description='Odom Recorder',
    license='Apache licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_recorder = odom_recorder.odom_recorder:main',
        ],
    },
)
