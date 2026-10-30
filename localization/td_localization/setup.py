from setuptools import find_packages, setup

PACKAGE = "td_localization"

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
    description="Field localization: particle filter (LOC-05), side belief (LOC-06).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["loc_eval = td_localization.eval:main"]},
)
