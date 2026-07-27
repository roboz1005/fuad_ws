import sys
if sys.prefix == '/home/fuad/Downloads/fuad_ws/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fuad/Downloads/fuad_ws/install/integration_pkg'
