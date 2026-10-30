from setuptools import find_packages, setup

PACKAGE = "td_bench"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Bench testing of localization with any mix of real, recorded and simulated inputs.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["loc_bench = td_bench.__main__:main"]},
)
