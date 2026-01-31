from setuptools import setup
from glob import glob
import os

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # URDF / Xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # Meshes (recursive)
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/**/*', recursive=True)),

        # sensors (recursive)
        (os.path.join('share', package_name, 'sensors'),
            glob('sensors/**/*', recursive=True)),

        # Static config (unused for now)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='None',
    maintainer_email='none@todo.todo',
    description='Robot description (URDF/Xacro) for rover',
    license='Apache-2.0',
)
