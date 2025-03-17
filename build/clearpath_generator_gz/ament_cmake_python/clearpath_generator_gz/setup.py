from setuptools import find_packages
from setuptools import setup

setup(
    name='clearpath_generator_gz',
    version='0.3.0',
    packages=find_packages(
        include=('clearpath_generator_gz', 'clearpath_generator_gz.*')),
)
