#!/usr/bin/env python3

from setuptools import setup

setup(
    name="yb_expansion_board",
    version="1.0.0",
    author="Julian A. Rendon",
    author_email="julianrendon514@gmail.com",
    description="A Python Library for the various Yahboom expansion boards.",
    license="MIT",
    packages=["yb_expansion_board"],
    install_requires=["gpiozero"],
)
