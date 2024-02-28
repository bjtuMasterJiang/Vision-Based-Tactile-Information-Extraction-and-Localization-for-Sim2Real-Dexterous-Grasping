import pypot.dynamixel
from pypot.dynamixel import DxlIO

ports = pypot.dynamixel.get_available_ports()
print('available ports:', ports)

if not ports:
    raise IOError('No port available.')

port = ports[0]
print('Using the first on the list', port)

dxl_io = pypot.dynamixel.DxlIO(port)
print('Connected!')
ids = [30, 31, 32, 33, 34, 35, 36, 37]
# 30 main-board
# 31 wrist rotation
# 32 wrist adduction
# 33 wrist flexion
# 34 thumb adduction
# 35 thumb flexion
# 36 index finger flexion
# 37 middle finger flexion
# 38 the fourth and fifth fingers flexion
dxl_io.enable_torque(ids)
def position_detect():
    print(dxl_io.get_present_position(ids))
def speed_detect():
    v_speed = dxl_io.get_present_speed(ids)
    print(v_speed)


# negative number: extend palms out
# positive number: draw palms in
# zero: the palm of the hand is about half clenched
def victory():
    # speed=180
    # dxl_io.set_moving_speed(ids,speed) # set the speed
    dxl_io.set_moving_speed({37: 0.01})
    dxl_io.set_goal_position({38: 105})
    dxl_io.set_goal_position({37: -155})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: 65})


def button_press():
    dxl_io.set_moving_speed({ids: 0.01})
    dxl_io.set_goal_position({38: 105})
    dxl_io.set_goal_position({37: 65})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: 65})

def palm_open():
    dxl_io.set_moving_speed({35: 0.00001})
    dxl_io.set_goal_position({38: -165})
    dxl_io.set_goal_position({37: -165})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: -115})
    dxl_io.set_goal_position({34: -105})
