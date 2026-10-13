from glob import glob

from setuptools import setup

package_name = "rammp_button_press"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "rammp-alternative.yaml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/sheppy", glob("sheppy/*.yaml") + ["sheppy/env.sh"]),
        ("share/" + package_name + "/sheppy/profiles", glob("sheppy/profiles/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Yi",
    maintainer_email="dhyi@andrew.cmu.edu",
    description="Autonomous microwave button press on RAMMP's kinova_gen3_node arm driver.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "press_button = rammp_button_press.press_button:main",
            "measure_prepress_offset = rammp_button_press.measure_prepress_offset:main",
            "view_detection = rammp_button_press.view_detection:main",
            "calibration_tf = rammp_button_press.calibration_tf:main",
            "press_server = rammp_button_press.press_server:main",
            "press_client = rammp_button_press.press_client:main",
        ],
    },
)
