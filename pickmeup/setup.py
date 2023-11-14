from setuptools import find_packages, setup

package_name = 'pickmeup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         [
             'package.xml',
             'launch/pickup.launch.xml',
             'launch/april_tag.launch.xml',
             'config/tags1.yaml'
         ]
         ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Graham Clifford',
    maintainer_email='grahamclifford2024@u.northwestern.edu',
    description='This package controls the Emika Franka Panda robot and gets \
    it to pick up a piece of paper.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "picker = pickmeup.picker:main",
            "tags = pickmeup.tags:Tags_entry",
        ],
    },
)
