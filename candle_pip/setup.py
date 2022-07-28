import setuptools


setuptools.setup(
    name="pyCandleMAB",
    version="1.1.1",
    author="Piotr Wasilewski",
    author_email="support.md80@mabrobotics.pl",
    description="Python package for controlling MD80-based actuators",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU Lesser General Public License v3 or later (LGPLv3+)",
        "Operating System :: POSIX :: Linux",
    ],
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
    package_data={'mab': ['pyCandle.cpython-38-x86_64-linux-gnu.so']},
)
