from setuptools import setup
setup(
    name='orv_gui', version='0.1.0', packages=['orv_gui'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/orv_gui']),
                ('share/orv_gui', ['package.xml'])],
    install_requires=['setuptools'], extras_require={'qt': ['PySide6>=6.8,<6.9']},
    maintainer='ktj', maintainer_email='ktj@example.invalid', description='ORV motor tuning GUI',
    license='Apache-2.0', entry_points={'console_scripts': ['tuner = orv_gui.app:main']},
)
