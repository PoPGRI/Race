from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['envobj_bb_publihser'],
    package_dir={'': 'src'},
)
