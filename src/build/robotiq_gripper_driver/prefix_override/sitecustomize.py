import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bwilab/fri_ws/src/install/robotiq_gripper_driver'
