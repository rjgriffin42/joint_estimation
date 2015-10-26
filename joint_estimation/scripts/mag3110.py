#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32 
from Adafruit_I2C import Adafruit_I2C

address = 0x60

i2c = Adafruit_I2C(address)

P9_19: I2C2, SCL
P9_20: I2C2, SDA

def read_and_publish():
  # initialize publishers
  pub_x = rospy.Publisher('magnet_x', Float32, queue_size=10)
  pub_y = rospy.Publisher('magnet_y', Float32, queue_size=10)
  pub_z = rospy.Publisher('magnet_z', Float32, queue_size=10)
  rospy.init_node('mag3110_driver', anonymous=True)
  rate = rospy.Rate(10) # 10 hz


  while not rospy.is_shutdown():
    # read data from board
    byte = i2c.readU8()
    x = (byte[0]<<8) + byte[1]
    y = (byte[2]<<8) + byte[3]
    z = (byte[4]<<8) + byte[5]

    # publish data out
    pub_x.publish(x)
    pub_y.publish(y)
    pub_z.publish(z)
    rate.sleep()
