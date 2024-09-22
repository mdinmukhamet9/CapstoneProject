from setuptools import find_packages
from setuptools import setup

setup(
    name='clearpath_generator_gz',
    version='0.2.6',
    packages=find_packages(
        include=('clearpath_generator_gz', 'clearpath_generator_gz.*')),
)
