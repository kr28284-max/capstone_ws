from setuptools import find_packages, setup

package_name = 'control_pkg'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node1 = control_pkg.test_node1:main',
            'test_node2 = control_pkg.test_node2:main',
            'test_node3 = control_pkg.test_node3:main',
            'test_node4 = control_pkg.test_node4:main',
            'test_node5 = control_pkg.test_node5:main',
        ],
    },
)
