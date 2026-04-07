import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jazzy/Desktop/dynominion_rmf/install/teleop_robot'
