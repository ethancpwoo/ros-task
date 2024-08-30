from setuptools import find_packages, setup

package_name = 'py_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name , ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ethan',
    maintainer_email='ecpwoo@uwaterloo.ca',
    description='TODO: Package description',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plotter_node = py_utils.plotter:main',
            'params_node = py_utils.params:main',
        ],
    },
    package_data={
        package_name: [
            'config/params.yaml',
        ],
    },
)
