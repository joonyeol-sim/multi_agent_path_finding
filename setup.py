#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('README.rst') as readme_file:
    readme = readme_file.read()

with open('HISTORY.rst') as history_file:
    history = history_file.read()

requirements = ['Click>=7.0', ]

test_requirements = ['pytest>=3', ]

setup(
    author="Joonyeol Sim",
    author_email='jysim@u.sogang.ac.kr',
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description="Implement of Multi-Agent Path Finding",
    entry_points={
        'console_scripts': [
            'multi_agent_path_finding=multi_agent_path_finding.cli:main',
        ],
    },
    install_requires=requirements,
    license="MIT license",
    long_description=readme + '\n\n' + history,
    include_package_data=True,
    keywords='multi_agent_path_finding',
    name='multi_agent_path_finding',
    packages=find_packages(include=['multi_agent_path_finding', 'multi_agent_path_finding.*']),
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/joonyeolsim/multi_agent_path_finding',
    version='0.1.0',
    zip_safe=False,
)
