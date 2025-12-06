from setuptools import find_packages, setup

package_name = "jazzy_py_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=[],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "say_hello=jazzy_py_demo.talker:main",
        ],
    },
)
