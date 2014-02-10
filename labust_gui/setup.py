from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['labust_rqt','control'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'rqt_gui', 'rqt_gui_py'])

setup(**setup_args)
