import py_trees
import rospy
import rospkg
import json

from robot_toolkit_msgs.msg import speech_msg, animation_msg
from robot_toolkit_msgs.srv import speech_recognition_sr
from navigation_msgs.msg import simple_feedback_msg
from navigation_msgs.srv import  go_to_place_srv ,get_route_guidance_srv
from speech_recognition_msgs.srv import q_a_speech_srv, q_a_speech_srvRequest, talk_speech_srv, talk_speech_srvRequest
from perception_msgs.srv import start_recognition_srv, look_for_object_srv , get_labels_srv

from std_msgs.msg import Int32

# Comportamiento que verifica la ubicación de equipaje
start_place = "living_room"
end_place = "outside_area"

# comportamiento para reconocer operador/persona
class RecognizeOperator(py_trees.behaviors.Behavior):
    def setup(self, timeout, **kwargs):
        self.timeout = timeout
    def initialise(self):
        self.start_time = py_trees.blackboard.Blackboard().current_time
        self.feedback_message = "Recognizing operator"
        rospy.wait_for_service('/perception_utilities/start_recognition_srv')
        self.startRecognitionServiceClient = rospy.ServiceProxy('/perception_utilities/start_recognition_srv', start_recognition_srv)
    def update(self):
        current_time = py_trees.blackboard.Blackboard().current_time
        time_elapsed = current_time - self.start_time
        if time_elapsed > self.timeout:
            self.feedback_message = "Unable to recognize operator"
            return py_trees.Status.FAILURE
        # Reconoce al operador
        if "operator_recognized" in py_trees.blackboard.Blackboard().keys():
            self.feedback_message = "Operator recognized"
            return py_trees.Status.SUCCESS

        return py_trees.Status.RUNNING

class KnowLuggageLocation(py_trees.behaviors.Behavior):
    def setup(self, timeout, **kwargs):
        self.timeout = timeout
    def initialise(self):
        self.start_time = py_trees.blackboard.Blackboard().current_time
        self.feedback_message = "Checking if I know the location of my luggage"
    def update(self):
        current_time = py_trees.blackboard.Blackboard().current_time
        time_elapsed = current_time - self.start_time
        if time_elapsed > self.timeout:
            self.feedback_message = "I have no information about my luggage"
            return py_trees.Status.FAILURE

        # Checa si sabes la ubicación de tu equipaje
        if "luggage_location" in py_trees.blackboard.Blackboard().keys():
            self.feedback_message = "I know the location of my luggage"
            return py_trees.Status.SUCCESS

        return py_trees.Status.RUNNING

# Comportamiento que busca tu equipaje en una ubicación conocida
class SearchLuggage(py_trees.behaviors.Behavior):
    def setup(self, timeout, **kwargs):
        self.timeout = timeout

    def initialise(self):
        self.start_time = py_trees.blackboard.Blackboard().current_time
        self.feedback_message = "Searching for my luggage"

    def update(self):
        current_time = py_trees.blackboard.Blackboard().current_time
        time_elapsed = current_time - self.start_time
        if time_elapsed > self.timeout:
            self.feedback_message = "Unable to find my luggage"
            return py_trees.Status.FAILURE

        # Busca tu equipaje en la ubicación conocida
        if "luggage_found" in py_trees.blackboard.Blackboard().keys():
            self.feedback_message = "Found my luggage"
            return py_trees.Status.SUCCESS

        return py_trees.Status.RUNNING
