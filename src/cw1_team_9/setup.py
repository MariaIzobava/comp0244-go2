from setuptools import find_packages, setup

package_name = 'cw1_team_9'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/run_solution_task_1.launch.py', 
             'launch/run_solution_task_2.launch.py',
             'launch/run_solution_task_3.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maryia',
    maintainer_email='maryia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug0 = cw1_team_9.bug0_walker:main',
            'bug1 = cw1_team_9.bug1_walker:main',
        ],
    },
)
