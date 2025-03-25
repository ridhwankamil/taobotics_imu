from setuptools import find_packages, setup

package_name = 'taobotics_imu'

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
    maintainer='ridhwan',
    maintainer_email='ridhwan.kamil.rk@gmail.com',
    description='taobotics hfi_a9 imu nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_a9_node = taobotics_imu.hfi_a9_node:main'
        ],
    },
)
