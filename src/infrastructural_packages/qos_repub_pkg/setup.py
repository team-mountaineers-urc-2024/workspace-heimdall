from setuptools import find_packages, setup

package_name = 'qos_repub_pkg'

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
    maintainer='jdb3',
    maintainer_email='jalen.beeman@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressed_repub = qos_repub_pkg.compressed_repub:main',
            'decompress_repub = qos_repub_pkg.decompress_repub:main',
        ],
    },
)
