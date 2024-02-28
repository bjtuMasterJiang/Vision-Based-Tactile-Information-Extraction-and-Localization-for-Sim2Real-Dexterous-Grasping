import pypot.dynamixel

# initialize a DxlIO object
dxl_io = pypot.dynamixel.io.DxlIO(port='COM8', baudrate=1000000, timeout=0.1)

# power off
dxl_io.disable_torque([30, 31, 32, 33, 34, 35, 36, 37, 38])  # pass a list of motor IDs to power off

# communication close
dxl_io.close()