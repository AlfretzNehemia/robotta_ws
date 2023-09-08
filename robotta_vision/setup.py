from setuptools import setup

package_name = 'robotta_vision'

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
    maintainer='alfretz',
    maintainer_email='alfretznehemia@gmail.com',
    description='object detection with YOLOv8 and sending cmd_vel data for human following',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detection_node = robotta_vision.object_following:main',
        ],
    },
)
