from setuptools import find_packages, setup

package_name = 'service_python_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/resource", ['resource/default.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyoveros',
    maintainer_email='liuy27@zju.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'learn_face_detect = service_python_detect.learn_face_detect:main',
            'face_detect_node = service_python_detect.face_detect_node:main',
            'face_detect_client_node = service_python_detect.face_detect_client_node:main',
        ],
    },
)
