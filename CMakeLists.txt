cmake_minimum_required(VERSION 3.0.2)
project(qt_interaction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  TextToSpeech.srv
  RasaService.srv
  BehaviorTalkText.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  src/emotion_game_node.py
  src/qt_speech_server.py
  src/rasa_server_node.py
  src/transcriber_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
