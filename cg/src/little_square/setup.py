from setuptools import find_packages, setup

package_name = 'little_square'

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
    maintainer='lidia',
    maintainer_email='lidia.mariano@sou.inteli.edu.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_reativa = little_square.nav_reativa:main',
            'nav_mapa = little_square.nav_com_mapa:main',
        ],
    },
)
