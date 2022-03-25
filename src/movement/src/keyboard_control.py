#!/usr/bin/env python3

# coding: utf-8
import rospy
import time
from pynput import keyboard
from std_msgs.msg import String 


class KeyboardListener():

    def __init__(self) -> None:

        # Listener for the keyboard events
        self.listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release)
        self.listener.start()

        self.start = time.time()
        # Publishers
        self.keyboard_publisher = rospy.Publisher("/gpio/car_control_keyboard", String, queue_size=1)

    def on_press(self, key:keyboard.KeyCode)->None:
        """Callback hook on key press.

        Args:
            msg (keyboard.Key): the pressed key
        """
        try:
            
            msg = String()
            if key.char == 'w':
                msg = String('up')
            elif key.char == 's':
                msg = String('down')
            elif key.char == 'a':
                msg = String('left')
            elif key.char == 'd':
                msg = String('right')
            
            if msg.data != '':
                if time.time() - self.start > 0.35:
                    self.keyboard_publisher.publish(msg)
                    self.start = time.time()
        except:
            print('Keyboard listener Error occured ')


    def on_release(self, msg:keyboard.Key)->None:
        """Callback hook on key release.

        Args:
            msg (keyboard.Key): the released key
        """
        pass

    
if __name__ == "__main__":
    rospy.init_node('keyboard_listener')
    keyboard_listener = KeyboardListener()
    rospy.spin()
