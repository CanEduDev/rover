from pathlib import Path

from setuptools import setup

package_name = "rover_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package.xml
        ("share/" + package_name, ["package.xml"]),
        # URDF / Xacro
        (
            str(Path("share") / package_name / "urdf"),
            [str(p) for p in Path("urdf").glob("*")],
        ),
        # Meshes (recursive)
        (
            str(Path("share") / package_name / "meshes"),
            [str(p) for p in Path("meshes").rglob("*") if p.is_file()],
        ),
        # sensors (recursive)
        (
            str(Path("share") / package_name / "sensors"),
            [str(p) for p in Path("sensors").rglob("*") if p.is_file()],
        ),
        # Static config (unused for now)
        (
            str(Path("share") / package_name / "config"),
            [str(p) for p in Path("config").glob("*")],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="None",
    maintainer_email="none@todo.todo",
    description="Robot description (URDF/Xacro) for rover",
    license="Apache-2.0",
)
