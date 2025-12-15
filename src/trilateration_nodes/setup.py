from setuptools import find_packages, setup

package_name = 'trilateration_nodes'

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
    maintainer='root',
    maintainer_email='cryptonian007@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "rangefinder = trilateration_nodes.rangefinder:main",
            "trilateration_node = trilateration_nodes.trilateration:main",
            "visualizernode = trilateration_nodes.visualizer:main"
        ],
    },
)
