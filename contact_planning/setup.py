from setuptools import find_packages, setup

package_name = 'contact_planning'

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
    maintainer='zolkin',
    maintainer_email='zach.olkin@gmail.com',
    description='Contact Planning Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_planner = contact_planning.contact_planner:main',
            'mujoco_geom_reader = contact_planning.mujoco_geom_reader:main'
        ],
    },
)
