# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cart_collection', 'cart_drop_off'],
    package_dir={'cart_collection': 'ros/src/cart_collection',
                 'cart_drop_off': 'ros/src/cart_drop_off'}
)

setup(**d)

