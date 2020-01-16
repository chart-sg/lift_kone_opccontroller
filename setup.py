from setuptools import find_packages
from setuptools import setup

package_name = 'kone_opc_controller'

setup(
    name=package_name,
    version='0.5.1',
    packages=find_packages(),
    # py_modules=['kone_client'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Marc-Antoine Testier',
    author_email='marc.testier@gmail.com',
    maintainer='Marc-Antoine Testier',
    maintainer_email='marc.testier@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes to control kone lift through an OPC server'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'kone_client = kone_opc_controller.main:main',
        ],
    },
)
