import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dhandavishal/aerostack2_ws/src/as2_ardu_msn/install/as2_ardu_msn'
