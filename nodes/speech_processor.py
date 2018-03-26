#!/usr/bin/env python
import rospy
import pyaudio
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from std_msgs.msg import String
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *


class SpeechProcessor(object):
    """Processes natural speech to/from the syntax in DialogManager."""

    def __init__(self, model, lexicon, kwlist, pub):

        self.speech_cmd_pub = rospy.Publisher(, String, queue_size=10)

        # initialize pocketsphinx
        config = Decoder.default_config()
        config.set_string('-hmm', model)
        config.set_string('-dict', lexicon)
        config.set_string('-kws', kwlist)

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

