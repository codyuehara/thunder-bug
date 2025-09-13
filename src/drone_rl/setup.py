from setuptools import find_packages, setup

package_name = 'drone_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'stable-baselines3', 'torch'],
    zip_safe=True,
    maintainer='buggy',
    maintainer_email='codyuehara@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_node = drone_rl.run_env:main',
        ],
    },
)
