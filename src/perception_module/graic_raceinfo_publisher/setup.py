from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['graic_raceinfo_publisher'],
    package_dir={'': 'src'},
)
