import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jerry/AutoSweeperSystem/AutoSweeperSystem_ws/install/autosweeper_robot'
