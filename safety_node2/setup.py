

#to be invoked only by catkin to run .py scripts

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args=generate_distutils_setup(packages=['safety_node2'],
                                    package_dir={'': 'scripts'},
                                    )
setup(**setup_args)
