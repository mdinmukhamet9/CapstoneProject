from setuptools import find_packages
from setuptools import setup

setup(
    name='clearpath_generator_common',
    version='0.2.11',
    packages=find_packages(
        include=('clearpath_generator_common', 'clearpath_generator_common.*')),
)
