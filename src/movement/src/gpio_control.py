#!/usr/bin/env python3

# coding: utf-8
import smbus
import time
import math
import rospy
from std_msgs.msg import Float32MultiArray, String

class CarDriver(object):

    def get_i2c_device(self, address, i2c_bus):
        # Initialize the i2c driver of the car
        self._addr = address
        if i2c_bus is None:
            return smbus.SMBus(1)
        else:
            return smbus.SMBus(1)

    def __init__(self):
        # Create I2C device.
        self._device = self.get_i2c_device(0x16, 1)

        # Spped of the motor
        self.car_speed = 150

        # Subscribers
        rospy.Subscriber("/gpio/camera_servo", Float32MultiArray, self.camera_servo_control)
        rospy.Subscriber("/gpio/car_control_keyboard", String, self.car_control_keyboard)

    def car_control_keyboard(self, msg:String):
        """Choose a moving direction/speed of the robot given the specified message.

        Args:
            msg (String): String describing the change.
        """
        try:
            self.move_car(msg.data)
            # Process the change in car speed.
            if msg.data == 'speed_up':
                self.car_speed += 10
            if msg.data == 'speed_down':
                self.car_speed -= 10

        except Exception as e:
            print ('I2C error')
            print(e)

    def camera_servo_control(self, msg:Float32MultiArray):
        """Receive the control of the servo motor

        Args:
            msg (Float32MultiArray): Array of float32 values
        """
        servo = msg.data[0]
        angle = msg.data[1]

        self.control_servo(int(servo), int(angle))


    def write_u8(self, reg, data):
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except:
            print ('write_u8 I2C error')

    def write_reg(self, reg):
        try:
            self._device.write_byte(self._addr, reg)
        except:
            print ('write_u8 I2C error')

    def write_array(self, reg, data):
        try:
            # self._device.write_block_data(self._addr, reg, data)
            self._device.write_i2c_block_data(self._addr, reg, data)
        except:
            print ('write_array I2C error')

    def control_car(self, l_dir, l_speed, r_dir, r_speed):
        try:
            reg = 0x01
            data = [l_dir, int(math.fabs(l_speed)), r_dir, int(math.fabs(r_speed))]
            self.write_array(reg, data)
        except:
            print ('Ctrl_Car I2C error')

    def move_car(self, direction:String)->None:
        """Move the car given direction for a time period.

        Args:
            direction (String): Direction of the car.
        """
        # Process the  possible direction of the car.
        if direction == 'up':
            self.control_car(1, self.car_speed, 1, self.car_speed)
            rospy.sleep(0.35)
            self.car_stop()
        if direction == 'down':
            self.control_car(0, self.car_speed, 0, self.car_speed)
            rospy.sleep(0.35)
            self.car_stop()
        if direction == 'left':
            self.control_car(0, self.car_speed, 1, self.car_speed)
            rospy.sleep(0.35)
            self.car_stop()
        if direction == 'right':
            self.control_car(1, self.car_speed, 0, self.car_speed)
            rospy.sleep(0.35)
            self.car_stop()
        
        #rospy.sleep(0.01)

    def car_stop(self):
        try:
            reg = 0x02
            self.write_u8(reg, 0x00)
        except:
            print ('Car_Stop I2C error')

    def Car_Back(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 0, speed2)
        except:
            print ('Car_Back I2C error')

    def Car_Left(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 1, speed2)
        except:
            print ('Car_Spin_Left I2C error')

    def Car_Right(self, speed1, speed2):
        try:
            self.Ctrl_Car(1, speed1, 0, speed2)
        except:
            print ('Car_Spin_Left I2C error')

    def control_servo(self, id, angle):

        try:
            reg = 0x03
            data = [id, angle]
            if angle < 0:
                angle = 0
            elif angle > 180:
                angle = 180
            self.write_array(reg, data)
        except:
            print('Control servo of the camera does not work')

if __name__ == "__main__":
    rospy.init_node('gpio_control')
    car = CarDriver()
    rospy.spin()
