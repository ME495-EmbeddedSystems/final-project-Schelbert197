from setuptools import find_packages, setup

package_name = 'ocr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
<<<<<<< HEAD
        ('share/' + package_name, ['package.xml', "launch/ocr_game.launch.xml"]),
=======
        ('share/' + package_name, ['package.xml']),
>>>>>>> 25da7c6 (Working opencv example inside a ros node.)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi2001',
    maintainer_email='241abhishek@gmail.com',
    description='ROS package for OCR on a whiteboard',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            "ocr = ocr.ocr:main",
            "hangman = ocr.hangman:main"
=======
            "ocr = ocr.ocr:main"
>>>>>>> 25da7c6 (Working opencv example inside a ros node.)
        ],
    },
)
