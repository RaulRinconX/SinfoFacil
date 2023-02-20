import py_trees
import rospy
import time
from robot_toolkit_msgs.msg import animation_msg
from speech_recognition_msgs.srv import talk_speech_srv
from perception_msgs.srv import (
    start_recognition_srv,
    look_for_object_srv,
    read_qr_srv,
    turn_camera_srv
)
from std_msgs.msg import Int32

# Introducte event task
class Initial(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.talkSpeechClient = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)

    def setup(self, timeout):
        self.talkSpeechClient.call("I am going to do the Sisandes task", 2)
        return True

    def update(self):
        return py_trees.common.Status.SUCCESS


# Waits for a person to be detected
class Wait4Guest(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.startRecognitionServiceClient = rospy.ServiceProxy('/perception_utilities/start_recognition_srv', start_recognition_srv)
        self.lookForSomethingServiceClient = rospy.ServiceProxy('/perception_utilities/look_for_object_srv', look_for_object_srv)
        self.talkSpeechClient = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)
        self.turnCameraServiceClient = rospy.ServiceProxy('/perception_utilities/turn_camera_srv', turn_camera_srv)
        self.simpleFeedbackSubscriber = rospy.Subscriber('/perception_utilities/look_for_object_publisher', Int32, self.callback_look_for_object)
        self.found = 0

    def setup(self, timeout):
        self.turnCameraServiceClient.call('front_camera', 'enable', 0, 0) 
        rospy.sleep(2)
        reqStart = start_recognition_srvRequest()
        reqStart.camera_name = "front_camera"
        self.startRecognitionServiceClient.call(reqStart)
        reqLook = look_for_object_srvRequest()
        reqLook.object = "person"
        self.lookForSomethingServiceClient.call(reqLook)
        return True

    def update(self):
        if self.found == 1:
            reqStart = start_recognition_srvRequest()
            reqStart.camera_name = "off"
            self.startRecognitionServiceClient.call(reqStart)
            self.talkSpeechClient.call("Hello guest", 2)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def callback_look_for_object(self, msg):
        self.found = msg.data


# Receive people to the event
class Receive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.talkSpeechClient = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)

    def setup(self, timeout):
        self.animationPublisher.publish(animation_msg("animations", "Gestures/Maybe_1"))
        return True

    def update(self):
        self.talkSpeechClient.call("I am going to receive you", 2)
        return py_trees.common.Status.SUCCESS


# Check if people belong to the event
class Check(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.talkSpeechClient = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)
        self.readQrCode
