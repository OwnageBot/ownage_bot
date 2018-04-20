#!/usr/bin/env python
import rospy
import pyaudio
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from std_msgs.msg import String
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

MODEL_DIR = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model/"

class SpeechProcessor(object):
    """Processes natural speech to/from the syntax in DialogManager."""

    def __init__(self):

        self.speech_cmd_pub = rospy.Publisher(, String, queue_size=10)
        self.hmm_path = rospy.get_param("~hmm_path", MODEL_DIR + "en-us")
        self.lm_path = rospy.get_param("~lm_path", "../params/speech.lm")
        self.dict_path = rospy.get_param("~dict_path", "../params/speech.dic")

        # initialize pocketsphinx
        config = Decoder.default_config()
        config.set_string('-hmm', self.hmm_path)
        config.set_string('-lm', self.lm_path)
        config.set_string('-dict', self.dict_path)

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                                        rate=16000, input=True,
                                        frames_per_buffer=1024)
        stream.start_stream()

        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_result()

    def parse_result(self):
        """Parse decoded segments."""

        if self.decoder.hyp() != None:
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()

            # TODO: Fill in logic
            if seg.word.find("i am xuan") > -1:
                msg = "i am xuan"
    
        self.speech_cmd_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('speech_processor')
    speech_processor = speech_processor()
    rospy.spin()

