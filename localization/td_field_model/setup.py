from setuptools import find_packages, setup

PACKAGE = "td_field_model"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={PACKAGE: ["fields/*.json", "fields/SOURCE.md"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="HSL soccer field geometry, landmarks and line likelihood field for localization (LOC-03).",
    license="MIT",
    tests_require=["pytest"],
)
