#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests
import random
from qt_interaction.srv import RasaService

"""
ROS node for switching between emotions displayed during interactive game session.
This node subscribes to the /vivek/next_emotion_trigger topic, and publishes to the /qt_robot/emotion/show topic on the QTrobot. 
"""

class EmotionSwitcher:
    def __init__(self):
        self.node = rospy.init_node('emotion_switcher')
        self.emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
        self.sub = rospy.Subscriber('/vivek/next_emotion_trigger', String, self.callback)
        self.emotions = ["QT/afraid", "QT/angry", "QT/sad", "QT/confused", "QT/cry", "QT/disgusted", "QT/sad", "QT/shy", "QT/surprise", "QT/happy"]
        self.count = 0
        self.max_count = 3  # Maximum number of times to set emotion
        self.rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"  # Rasa server endpoint

    def callback(self, msg):
        if msg.data == "next_emotion" and self.count < self.max_count:
            # Choose a random emotion
            emotion = random.choice(self.emotions)
            # Publish the emotion
            self.emotionShow_pub.publish(emotion)
            self.count += 1
            rospy.loginfo(f"count: {self.count}")
            if self.count >= self.max_count:
                # If the maximum count is reached, send a final message to the Rasa server
                rospy.loginfo("Sending activity_finished to Rasa server")
                rasa_service = rospy.ServiceProxy('rasa_service', RasaService)
                rasa_service("system: activity_finished")
                self.count = 0
            else:
                # Send a POST request to Rasa server
                emotion_name = emotion.split("/")[1]  # Extract the name of the emotion
                rospy.loginfo(f"Sending emotion {emotion_name} to Rasa server")
                emotional_msg = f"system: current emotion displayed is {emotion_name}"
                rasa_service = rospy.ServiceProxy('rasa_service', RasaService)
                rasa_service(emotional_msg)



    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    es = EmotionSwitcher()
    es.spin()
