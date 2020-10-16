from setuptools import setup

package_name = 'test_video_subscriber'

setup(
    name=package_name,
    version='0.10.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jari Nippula',
    author_email='jarix@ssrc.tii.ae',
    maintainer='Jari Nippula',
    maintainer_email='jarix@ssrc.tii.ae',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='video subscribers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_video_subscriber ='
            ' test_video_subscriber.test_video_subscriber:main',
        ],
    },
)
