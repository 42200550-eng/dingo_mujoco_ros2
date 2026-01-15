from setuptools import setup

package_name = 'dingo_mujoco'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='MuJoCo-based simulator node for Dingo, keeping CHAMP/ROS2 topics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_dingo_sim = dingo_mujoco.mujoco_dingo_sim:main',
        ],
    },
)
