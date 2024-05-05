from setuptools import setup

package_name = 'sequence_action_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sebastianmayorquin@gmail.com',
    description='Sequence Action Server for managing executions of packages in core',
    license='MIT',
)
