from setuptools import setup

package_name = 'turtlesim_catch_turtles'

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
    maintainer='natalia',
    maintainer_email='natalia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = turtlesim_catch_turtles.controller:main",
            "spawner = turtlesim_catch_turtles.spawner:main",
        ],
    },
)
