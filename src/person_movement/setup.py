from setuptools import setup

package_name = 'person_movement'

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
    maintainer='dimash',
    maintainer_email='your_email@example.com',
    description='A package to move a person model in Gazebo and publish its position',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_and_publish_person = person_movement.move_and_publish_person:main',
        ],
    },
)