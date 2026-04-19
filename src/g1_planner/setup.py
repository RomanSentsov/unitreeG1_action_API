from setuptools import find_packages, setup

package_name = 'g1_planner'

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
    maintainer='egorlekomtsev',
    maintainer_email='egorlekomtsev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'high_level_planner = g1_planner.high_level_planner:main',
            'mock_manipulate_server = g1_planner.mock_manipulate_server:main',
        ],
    },
)
