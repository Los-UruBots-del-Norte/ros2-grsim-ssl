from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='ssl-python-clients',
    version='1.0.0',
    url='https://gitlab.com/diploma_2021/robo_football.git',
    author='Mikhail Lipkovich',
    author_email='lipkovich.mikhail@gmail.com',
    description='Client library for GrSim and ErForce simulators',
    packages=find_packages(),
    install_requires=requirements,
)
