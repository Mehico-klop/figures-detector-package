from setuptools import setup
from glob import glob
import os

package_name = 'figure_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faust',
    maintainer_email='vitosik3000@gmail.com',
    entry_points={
        'console_scripts': [
            'figure_node = figure_detector.figure_node:main',
        ],
    },
)

