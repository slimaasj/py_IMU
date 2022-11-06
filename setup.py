from setuptools import setup

package_name = 'py_imu'

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
    maintainer='slim',
    maintainer_email='Salimjbara@gmail.com',
    description='Publishes raw IMU and Mag',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyIMUpub = py_imu.pyIMUpub:main'
        ],
    },
)
