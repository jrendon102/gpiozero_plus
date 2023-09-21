#!/usr/bin/env python3

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as readme_file:
    long_description = readme_file.read()

setup(
    name="gpiozero_plus",
    version="0.0.1",
    url="https://github.com/jrendon102/gpiozero_plus.git",
    author="Julian A Rendon",
    author_email="julianrendon514@gmail.com",
    description="A Python Library providing enhanced features and utilities for gpiozero.",
    long_description=long_description,
    packages=find_packages(),
    license="MIT",
    install_requires=["gpiozero"],
    keywords=["python", "gpiozero", "rpi", "pigpio"],
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
        "Operating System :: Unix",
    ],
)
