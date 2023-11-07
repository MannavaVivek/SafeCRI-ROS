#!/usr/bin/env python
import rospy
from qt_interaction.srv import BehaviorTalkText, TextToSpeech

"""
ROS service node for text-to-speech. Interacts with QTrobot's talkText service for TTS.
"""
def text_to_speech(request):
    rospy.loginfo(f"Received request: {request.response_text}")
    response_text = request.response_text

    try:
        # Call the service
        service_response = qt_talk_text(response_text)
        # Process the response
        rospy.loginfo(f"Service response: {service_response.status}")
    except rospy.ServiceException as exc:
        rospy.logerr(f"Service call failed: {exc}")
    
    return {'success': True}

def service_node():
    global qt_talk_text
    rospy.init_node('service_node', anonymous=True)

    # # Alternate server to test TTS in QTrobot
    # s = rospy.Service('text_to_speech', TextToSpeech, text_to_speech)
    # rospy.wait_for_service('/qt_robot/speech/say')

    qt_talk_text = rospy.ServiceProxy("/qt_robot/behavior/talkText",BehaviorTalkText)
    rospy.wait_for_service("/qt_robot/behavior/talkText")
    
    print("Service ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        pass
