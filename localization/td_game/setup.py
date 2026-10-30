from setuptools import find_packages, setup

PACKAGE = "td_game"

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
    description="HSL GameController client and team-message packing/budget (LOC-08).",
    license="MIT",
    tests_require=["pytest"],
)
