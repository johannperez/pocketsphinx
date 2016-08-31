#!/usr/bin/env python

"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
    ~mic_name - set the pulsesrc device name for the microphone input.
           e.g. a Logitech G35 Headset has the following device name:
           alsa_input.usb-Logitech_Logitech_G35_Headset-00-Headset_1.analog-mono
           To list audio device info on your machine, in a terminal type:
           pacmd list-sources
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import os

from gi import pygtkcompat
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()
Gst.init(None)

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from audio_common_msgs.msg import AudioData

import commands

class recognizer(object):
    """ GStreamer based speech recognizer. """
    
    _device_name_param = "~mic_name"
    _lm_param = "~lm"
    _dic_param = "~dict"
    _audio_topic_param = "~audio_msg_topic"
    
    _ros_audio_topic = None
    _app_source = None

    asr = None
    bus = None
    bus_id = None
    
    started = False
    
    def __init__(self):
        rospy.loginfo('#########')
        rospy.init_node("recognizer")

        if not self.valid_parameters():
            return

        self.init_launch_config()
        self.configure_ros()
        self.init_pipeline()
        
        self.asr = self.pipeline.get_by_name('asr')
        
        if self._ros_audio_topic:
            if not self.suscribe_to_audio_topic():
                return;

        self.asr.set_property('lm', self.lm)
        self.asr.set_property('dict', self.dic)
        
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message::element', self.element_message)

        self.pipeline.set_state(Gst.State.PAUSED)
        #self.start_recognizer()
                          
    def suscribe_to_audio_topic(self):
        # returns True if it is able to subscribe succesfully
        self._app_source = self.pipeline.get_by_name('appsrc')
        
        if not self._app_source:
            rospy.logerr('Error getting the appsrc element.')
            return  False
        
        rospy.loginfo('Subscribing to AudioData on topic: {}'.format(self._ros_audio_topic))
        rospy.Subscriber(self._ros_audio_topic, AudioData, self.on_audio_message)
        
        return True 
                               
    def valid_parameters(self):
        if rospy.has_param(self._lm_param):
            self.lm = rospy.get_param(self._lm_param)
            if not os.path.isfile(self.lm):
                rospy.logerr(
                    'Language model file does not exist: {}'.format(self.lm))
                return False
        else:
            rospy.logerr('Recognizer not started. Please specify a '
                         'language model file.')
            return False

        if rospy.has_param(self._dic_param):
            self.dic = rospy.get_param(self._dic_param)
            if not os.path.isfile(self.dic):
                rospy.logerr(
                    'Dictionary file does not exist: {}'.format(self.dic))
                return False
        else:
            rospy.logerr('Recognizer not started. Please specify a dictionary.')
            return False
        
        return True
            
    def init_pipeline(self):
        rospy.loginfo('Starting recognizer... pipeline: {}'.format(self.launch_config))
        self.pipeline = Gst.parse_launch(self.launch_config)
        if not self.pipeline:
            rospy.logerr('Could not create gstreamer pipeline.')
            return
        rospy.loginfo('gstreamer pipeline created.')

    def configure_ros(self):
        rospy.on_shutdown(self.shutdown)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)
        self.pub = rospy.Publisher('~output', String, queue_size=10)

    def init_launch_config(self):
        if rospy.has_param(self._device_name_param):
            self.device_name = rospy.get_param(self._device_name_param)
            self.device_index = self.pulse_index_from_name(self.device_name)
            self.launch_config = "pulsesrc device=" + str(self.device_index)
            rospy.loginfo("Using: pulsesrc device=%s name=%s", self.device_index, self.device_name)
        elif rospy.has_param('~source'):
            # common sources: 'alsasrc'
            self.launch_config = rospy.get_param('~source')
        elif rospy.has_param(self._audio_topic_param):
            # Use ROS audio messages as input: Use an appsrc to pass AudioData
            # messages to the gstreamer pipeline. Use 'mad' plugin to decode
            # mp3-formatted messages.
            self.launch_config = 'appsrc name=appsrc ! mad'
            self._ros_audio_topic = rospy.get_param(self._audio_topic_param)
            rospy.loginfo('Using ROS audio messages as input. Topic: {}'.
                          format(self._ros_audio_topic))
        else:
            self.launch_config = 'alsasrc device=plughw:0,0'# 'alsasrc device=plughw:0,0'#'pulsesrc'#'autoaudiosrrc'

        rospy.loginfo("Audio input: {}".format(self.launch_config))

        self.launch_config += " ! audioconvert ! audioresample " \
                            + '! pocketsphinx name=asr ! fakesink' 
        
    def element_message(self, bus, msg):
        rospy.loginfo('Message recrecieved')
        msgtype = msg.get_structure().get_name()
        if msgtype != 'pocketsphinx':
            return

        if msg.get_structure().get_value('final'):
            self.final_result(msg.get_structure().get_value('hypothesis')) #TODO: it would be nice to publish to a topic including confidence, msg.get_structure().get_value('confidence'))
        elif msg.get_structure().get_value('hypothesis'):
            self.partial_result(msg.get_structure().get_value('hypothesis'))
           
    def start_recognizer(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        rospy.loginfo('Recognizer started!')

    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput(
            ("pacmd list-sources | grep -B 1 'name: <" + name +
             ">' | grep -o -P '(?<=index: )[0-9]*'"))

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: {}".
                            format(name))

    def stop_recognizer(self):
        self.pipeline.set_state(Gst.State.PAUSED)

    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._device_name_param, self._lm_param, self._dic_param,
                      self._audio_topic_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

    def start(self, req):
        self.start_recognizer()
        rospy.loginfo("recognizer started")
        return EmptyResponse()

    def stop(self, req):
        self.stop_recognizer()
        rospy.loginfo("recognizer stopped")
        return EmptyResponse()

    def partial_result(self, hyp):
        rospy.loginfo("Partial: " + hyp)

    def final_result(self, hyp):
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo('Final result: {}'.format(msg.data))
        self.pub.publish(msg)

    def on_audio_message(self, audio):
        # Callback for ROS audio messages -- emits the audio data to the
        # gstreamer pipeline through the appsrc.
        rospy.logdebug('Received audio packet of length {}'.format(
            len(audio.data)))

        if self._app_source:
            self._app_source.emit('push-buffer',
                                  Gst.Buffer(str(bytearray(audio.data))))


if __name__ == "__main__":
    r = recognizer()
    try:
        GObject.MainLoop().run()
    except (KeyboardInterrupt):
        r.shutdown()
    
