import rospy
import numpy as np
import paho.mqtt.client as mqtt

from std_msgs.msg import Empty, _String
from rospy.numpy_msg import numpy_msg
from mouse_controller.msg import Floats, mouse_sensors

class MQTT_CLIENT_CONTROLLER:

    def __init__(self):
        print("Starting MQTT on robot side")
        self.q_values = np.array([0]*12)
        self.setup_mqtt_client()
        self.mqtt_listener_loop(rate = 100)

    def on_publish(self, client, userdata, mid) -> None:
        """ Callback for when publish commands are sent via mqtt"""
        print("Message published")

    def on_connect(self, client, userdata, flags, rc) -> None:
        print("Connected with result code "+str(rc))

    
    def setup_mqtt_client(self) -> None:
        """ Setup for mqtt client. Connect to mqtt broker"""
        self.client = mqtt.Client(client_id="mqtt_controller", clean_session=True, 
                                    userdata=None, transport="tcp")

        self.client.username_pw_set(username="crazy_lamp", password="Lamp1234")
        self.client.on_publish = self.on_publish
        self.client.on_connect = self.on_connect

        print("Now trying to connect")
        self.client.connect(host="64.227.112.163", port=1883, keepalive=60)

        self.topic_name = "q_values"

        hello = "Hello World! mqtt publisher here"
        self.client.publish(self.topic_name, hello,qos=0, retain=False)

    def callback_q_values(self, data) -> None:
        """ Callback to receive q_values from ROS"""
        self.q_values = np.array((data.data))

    def mqtt_listener_loop(self, rate: int) -> None:
        """ Main loop of the MQTT client to publish q_values into MQTT"""
        rospy.init_node("mqtt_client_controller", anonymous=True)
        r = rospy.Rate(rate)

        rospy.Subscriber('q_values', Floats, self.callback_q_values, queue_size = 1)
        count = 0
        while(not rospy.is_shutdown()):
            self.client.publish(self.topic_name, count,qos=0, retain=False)
            self.client.loop_write()
            count += 1
            count %= 100
            r.sleep()


if __name__ == "__main__":
    mqtt_client_robot = MQTT_CLIENT_CONTROLLER()    