from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mouse_controller','mouse_controller.leg_models','mouse_controller.motion',
    'mouse_controller.trajectory_generator','mouse_controller.state_machine','mouse_controller.spine_models'],
    package_dir={'': 'src'}
)

setup(**d)