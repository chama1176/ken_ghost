from setuptools import setup

package_name = 'ken_motion_sound_py'

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
    maintainer='kaede',
    maintainer_email='kaede6120@gmail.com',
    description='ken_motion_sound',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = ken_motion_sound_py.subscriber_member_function:main',
        ],
    },
)
