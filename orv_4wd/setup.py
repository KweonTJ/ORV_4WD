from setuptools import setup

setup(
    name='orv_4wd', version='0.1.0', packages=['orv_4wd'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/orv_4wd']),
                ('share/orv_4wd', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ktj', maintainer_email='ktj@example.invalid',
    description='UNO four wheel differential driver', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['driver = orv_4wd.node:main']},
)
