from setuptools import find_packages, setup

package_name = 'kaitenyaki'

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
    maintainer='robo25',
    maintainer_email='e1x23099@oit.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sikaku_odm = kaitenyaki.sikaku_odm:main',
            'hatinozi_odm = kaitenyaki.hatinozi_odm:main',
            'yoppy_sikaku_odm = kaitenyaki.yoppy_sikaku_odm:main',
            'pnd_odm = kaitenyaki.pnd_odm:main',
            'odm_cpp = kaitenyaki.odm_cpp:main',
            'odm_cpp_8 = kaitenyaki.odm_cpp_8:main',
            
        ],
    },
)
