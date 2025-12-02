from setuptools import find_packages, setup

package_name = 'project_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ir-t11',
    maintainer_email='miguellopdelgado@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "project_first_node = project_py_pkg.project_first_node:main",
        "number_publisher = project_py_pkg.number_publisher:main",
        ],
    },
)
