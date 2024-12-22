from setuptools import find_packages, setup

package_name = 'simple_world_models'

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
    maintainer='shashank',
    maintainer_email='shswami@mit.edu',
    description='Contains services and nodes for accessing and simulating simple world models',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = simple_world_models.two_dim_world:main',
            'client = simple_world_models.two_dim_world_req:main',
        ],
    },
)
