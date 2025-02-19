# installing Yolov8

errors: 
Summary: 4 packages finished [11.2s]
  1 package failed: yolo_ros
  1 package had stderr output: yolo_ros
  1 package not processed
Expected ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', '--help-commands'] to pass: /home/robot/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py:270: UserWarning: Unknown distribution option: 'tests_require'
  warnings.warn(msg)
Traceback (most recent call last):
  File "/home/robot/ros2_ws/src/yolo_ros/yolo_ros/setup.py", line 5, in <module>
    setup(
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/__init__.py", line 117, in setup
    return distutils.core.setup(**attrs)
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/_distutils/core.py", line 173, in setup
    ok = dist.parse_command_line()
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py", line 480, in parse_command_line
    if self.handle_display_options(option_order):
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/dist.py", line 975, in handle_display_options
    return _Distribution.handle_display_options(self, option_order)
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py", line 694, in handle_display_options
    self.print_commands()
  File "/home/robot/.local/lib/python3.10/site-packages/setuptools/dist.py", line 751, in print_commands
    cmdclass = ep.load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 883, in exec_module
  File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
  File "/home/robot/.local/lib/python3.10/site-packages/sphinx/setup_command.py", line 21, in <module>
    from sphinx.application import Sphinx
  File "/home/robot/.local/lib/python3.10/site-packages/sphinx/application.py", line 31, in <module>
    from sphinx.config import Config
  File "/home/robot/.local/lib/python3.10/site-packages/sphinx/config.py", line 21, in <module>
    from sphinx.util import logging
  File "/home/robot/.local/lib/python3.10/site-packages/sphinx/util/__init__.py", line 41, in <module>
    from sphinx.util.typing import PathMatcher
  File "/home/robot/.local/lib/python3.10/site-packages/sphinx/util/typing.py", line 37, in <module>
    from types import Union as types_Union
ImportError: cannot import name 'Union' from 'types' (/usr/lib/python3.10/types.py)


try: 

pip install --upgrade sphinx
pip install --upgrade setuptools
pip install --upgrade wheel

errors: 
--- stderr: yolo_ros                   
/home/robot/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py:270: UserWarning: Unknown distribution option: 'tests_require'
  warnings.warn(msg)

try: cd ~/ros2_ws/src/yolo_ros/yolo_ros$ gedit setup.py 
comment out 'tests_require'



# IN YOLO - change topics 

