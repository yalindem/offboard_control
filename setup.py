from setuptools import setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Eğer ek veri dosyalarınız varsa, burada belirtin
        # Örneğin, launch dosyaları için:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yalin Demir',
    maintainer_email='your_email@example.com',
    description='desc,
    license='Lisansa dair bilgiler',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'your_node_name = offboard_control.start_px4:main',
        ],
    },
)