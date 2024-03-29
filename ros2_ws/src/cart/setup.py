from setuptools import setup

package_name = 'cart'

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
    maintainer='teatime77',
    maintainer_email='teatime77@live.jp',
    description='cart navigation',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = cart.publisher:main',
        ],
    },
)
