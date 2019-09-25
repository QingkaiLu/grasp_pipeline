from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['grasp_pipeline'],
    package_dir={'': 'src'},
    requires=['std_msgs','message_generation','message_runtime', 'rospy', 'message_filters', 'sensor_msgs']
)

setup(**setup_args)
