from pypot import dynamixel


class DynamixelMotor:
    def __init__(self, motor_id, port='/dev/ttyUSB0'):
        self.motor_id = motor_id
        self.dxl_io = dynamixel.DxlIO(port, baudrate=57600)
        self.dxl_io.set_wheel_mode((self.motor_id,))
        self.direction = 1

    def scan(self, range):
        print self.dxl_io.scan(range)

    def get_position(self):
        return self.dxl_io.get_present_position((self.motor_id,))[0]

    def set_position(self, angle):
        self.dxl_io.set_goal_position({self.motor_id: angle})

    def set_speed(self, speed):
        return self.dxl_io.set_moving_speed({self.motor_id: speed})
