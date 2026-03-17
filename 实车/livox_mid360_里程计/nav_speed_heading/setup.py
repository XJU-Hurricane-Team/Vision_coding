from setuptools import setup

package_name = 'nav_speed_heading'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Extracts real-time velocity and heading from Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式：命令名称 = 包名.文件名:主函数名
            'tracker_node = nav_speed_heading.tracker_node:main',
            'preset_nav_node = nav_speed_heading.preset_nav_node:main'
        ],
    },
)