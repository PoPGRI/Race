from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['model_based_node'],
    package_dir={'': 'src'},
)
