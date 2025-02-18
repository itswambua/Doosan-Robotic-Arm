from setuptools import find_packages, setup

package_name = 'hum_pkg'

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
    maintainer='wam',
    maintainer_email='wam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'r_controller = hum_pkg.r_controller:main',
=======
>>>>>>> 77f3d32 (initial doosan packages and rviz)
        ],
    },
)
