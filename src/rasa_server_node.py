#!/usr/bin/env python3

import rospy
from qt_interaction.srv import RasaService
import requests

"""
Common node for handling Rasa requests from multiple clients. 
This is to ensure consistency in the Rasa server endpoint.
"""
rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook" # Rasa server endpoint

def handle_rasa_request(req):
    # print(f"User: {req.message}")
    response = requests.post(rasa_endpoint, json={"sender": "user", "message": req.message})
    try:
        data = response.json()
        response_text = ""
        for message in data:
            response_text += message['text'] + "\n"
        if response_text == "":
            print("Empty response from Rasa server.")
            raise Exception
    except Exception as e:
        print("Received exception: ", e)
        response_text = "Sorry, I didn't understand that. Can you please repeat?"
    print(f"Robbie: {response_text}")
    return response_text

def rasa_node():
    rospy.init_node('rasa_node')
    service = rospy.Service('rasa_service', RasaService, handle_rasa_request)
    rospy.spin()

if __name__ == "__main__":
    rasa_node()
