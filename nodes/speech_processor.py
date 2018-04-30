#!/usr/bin/env python
import rospy
import rospkg
import pyaudio
import re
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from std_msgs.msg import String
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class SpeechProcessor(object):
    """Processes natural speech to/from the syntax in DialogManager."""

    def __init__(self):
        # Load structured corpus for parsing decoded strings
        self.corpus = rospy.get_param("~corpus")

        # Pocketsphinx parameters
        pkg_path = rospkg.RosPack().get_path('ownage_bot')
        self.hmm_path = rospy.get_param("~hmm_path",
                                        pkg_path + "/speech/hmm")
        self.lm_path = rospy.get_param("~lm_path",
                                       pkg_path + "/speech/speech.lm")
        self.dict_path = rospy.get_param("~dict_path",
                                         pkg_path + "/speech/speech.dic")

        # PyAudio parameters
        self.audio_device_name = rospy.get_param("~audio_device_name",
                                                 "Samson RXD")
        self.audio_sample_rate = rospy.get_param("~audio_sample_rate",
                                                 16000)
        self.audio_buffer_size = rospy.get_param("~audio_buffer_size",
                                                 1024)

        # Parsing related variables
        self.pre_parse_re = "\S* ?\S* ?" # Match spurious words at utt. start
        self.post_parse_re = " ?\S*$" # Match spurious word at utt. end
        self.arg_parse_re = " (.+)" # Match argument at utt. end


        # Parse error message and string constants
        self.err_parse = ""
        self.NO_MATCH = "Could not match utterance to any command."
        
        # Initialize Pocketsphinx decoder
        config = Decoder.default_config()
        config.set_string('-hmm', self.hmm_path)
        config.set_string('-lm', self.lm_path)
        config.set_string('-dict', self.dict_path)
        config.set_string('-logfn', '/dev/null') # Suppress log msgs
        self.decoder = Decoder(config)
       
        # Initialize PyAudio and list device names
        pa = pyaudio.PyAudio()
        dev_names = [pa.get_device_info_by_index(i)['name'] for i in
                    range(pa.get_device_count())]
        # Default to first audio device
        dev_id = 0
        # Try to find input device that matches self.audio_device_name
        if self.audio_device_name is not None:
            for i, d in enumerate(dev_names):
                if self.audio_device_name.lower() in d.lower():
                    dev_id = i
        # Open audio stream from device
        self.stream = pa.open(format=pyaudio.paInt16, channels=1,
                              input=True, rate=self.audio_sample_rate,
                              frames_per_buffer=self.audio_buffer_size,
                              input_device_index=dev_id)

        # Publisher for commands recognized by DialogManager
        self.speech_cmd_pub = rospy.Publisher("speech_cmd", 
                                              String, queue_size=10)

    def process_stream(self):
        """Continuously process audio stream until shutdown."""
        self.stream.start_stream()
        self.decoder.start_utt()
        speech_detected = False
        while not rospy.is_shutdown():
            buf = self.stream.read(self.audio_buffer_size,
                                   exception_on_overflow=False)
            if buf:
                self.decoder.process_raw(buf, False, False)
                # Check if speech detection status has changed
                if self.decoder.get_in_speech() != speech_detected:
                    speech_detected = self.decoder.get_in_speech()
                    # Terminate and process utterance upon silence
                    if not speech_detected:
                        self.decoder.end_utt()
                        self.parse_utterance()
                        self.decoder.start_utt()
            else:
                break

    def parse_utterance(self):
        """Parse decoded utterances."""
        if self.decoder.hyp() is None:
            return
        utt = self.decoder.hyp().hypstr.lower()
        print utt

        for parse_f in [self.parse_as_introduction, self.parse_as_claim,
                        self.parse_as_permission,
                        self.parse_as_action, self.parse_as_task]:
            msg = parse_f(utt)
            if msg is None:
                continue
            print msg
            if type(msg) is list:
                for m in msg:
                    self.speech_cmd_pub.publish(m)
                    rospy.sleep(0.01)
            else:
                self.speech_cmd_pub.publish(msg)

    def parse_as_number(self, utt):
        """Parse utterance as number, e.g. 'zero one one' -> 11"""
        digits = ["zero", "one", "two", "three", "four",
                  "five", "six", "seven", "eight", "nine"]
        if len(utt) == 0:
            self.err_parse = "Cannot parse empty string as number."
            return None
        num = 0
        for w in utt.split():
            if w not in digits:
                # Fail to parse if not all words are digits
                self.err_parse = "Word does not correspond to digit."
                return None
            num = num*10 + digits.index(w)
        return num

    def parse_as_introduction(self, utt):
        """Parse utterance as agent introduction."""
        syn_list = self.corpus.get("introductions", "").splitlines()
        agt_list = self.corpus.get("agents", "").splitlines()
        for syn in syn_list:
            pattern = self.pre_parse_re + syn + self.arg_parse_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            agent = match.group(1)
            if agent not in agt_list:
                self.err_parse = "Agent name not recognized."
                return None
            return "i am " + agent
        self.err_parse = self.NO_MATCH
        return None

    def parse_as_claim(self, utt):
        """Parses utterance as ownership claim."""
        syn_db = self.corpus.get("claims", dict())
        current_db = syn_db.get("current", dict())
        others_db = syn_db.get("others", dict())
        agt_list = self.corpus.get("agents", "").splitlines()

        # Parse claims made on behalf of current agent
        # e.g. 'that's not mine' -> 'not ownedBy current current'
        for val, syn_list in current_db.iteritems():
            if val not in ["positive", "negative"]:
                continue
            prefix = "not " if val == "negative" else ""
            syn_list = syn_list.splitlines()
            for syn in syn_list:
                pattern = self.pre_parse_re + syn + self.post_parse_re
                match = re.match(pattern, utt)
                if match is None:
                    continue
                return prefix + "ownedBy current current"

        # Parse claims made on behalf of other agents
        # e.g. 'this belongs to jake' -> 'ownedBy current jake'
        for val, syn_list in others_db.iteritems():
            if val not in ["positive", "negative"]:
                continue
            prefix = "not " if val == "negative" else ""
            syn_list = syn_list.splitlines()
            for syn in syn_list:
                pattern = (self.pre_parse_re + syn +
                           " ?(?:agent)?" + self.arg_parse_re) 
                match = re.match(pattern, utt)
                if match is None:
                    continue
                agent = None
                if syn.split()[-1] == "agent":
                    # Parse agent by ID
                    agent = self.parse_as_number(match.group(1))
                elif match.group(1) in agt_list:
                    # Parse agent by name
                    agent = match.group(1)
                if agent is None:
                    self.err_parse = "Agent not recognized."
                    return None
                return prefix + "ownedBy current " + str(agent)

        self.err_parse = self.NO_MATCH
        return None

    def parse_as_reprimand(self, utt):
        """Parse utterance as reprimand."""
        reprimand_db = self.corpus.get("reprimands", dict())
        for rep, msgs in reprimand_db.iteritems():
            pattern = self.pre_parse_re + rep + self.post_parse_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            return msgs
        self.err_parse = self.NO_MATCH
        return None        
        
    def parse_as_action(self, utt):
        """Parse utterance as action command."""
        # Iterate through list of synonyms for each action command
        syn_db = self.corpus.get("actions", dict())
        for name, syn_list in syn_db.iteritems():
            syn_list = syn_list.splitlines()
            tgt_required = (actions.db[name].tgtype == Object)
            for syn in syn_list:
                if syn.split()[-1] == "object":
                    pattern = self.pre_parse_re + syn + self.arg_parse_re
                else:
                    pattern = self.pre_parse_re + syn + self.post_parse_re
                match = re.match(pattern, utt)
                if match is None:
                    continue
                if not tgt_required:
                    return name
                if len(match.groups()) < 1:
                    return name + " " + "current"
                target = self.parse_as_number(match.group(1))
                if target is None:
                    self.err_parse = "Action target not recognized."
                    return None
                else:
                    return name + " " + str(target)
        self.err_parse = self.NO_MATCH
        return None

    def parse_as_task(self, utt):
        """Parse utterance as a high-level task."""
        # Iterate through list of synonyms for each task
        syn_db = self.corpus.get("tasks", dict())
        for name, syn_list in syn_db.iteritems():
            syn_list = syn_list.splitlines()
            for syn in syn_list:
                pattern = self.pre_parse_re + syn + self.post_parse_re
                match = re.match(pattern, utt)
                if match is None:
                    continue
                return name
        self.err_parse = self.NO_MATCH
        return None

    def parse_as_permission(self, utt):
        """Parses utterance as object specific permission."""
        syn_db = self.corpus.get("permissions", dict())
        current_db = syn_db.get("current", dict())
        others_db = syn_db.get("others", dict())

        # Parse claims made about current action
        # e.g. 'you can't do that' -> 'forbid current on current'
        for val, syn_list in current_db.iteritems():
            if val not in ["forbid", "allow"]:
                continue
            syn_list = syn_list.splitlines()
            for syn in syn_list:
                pattern = self.pre_parse_re + syn + self.post_parse_re
                match = re.match(pattern, utt)
                if match is None:
                    continue
                return val + " current on current"

        # Parse claims made on specific action
        # e.g. 'you can't pick this up' -> 'forbid pickUp on current'
        for val, syn_list in others_db.iteritems():
            if val not in ["forbid", "allow"]:
                continue
            syn_list = syn_list.splitlines()
            for syn in syn_list:
                pattern = self.pre_parse_re + syn + self.arg_parse_re
                match = re.match(pattern, utt)
                if match is None:
                    continue
                # Parse remaining segment as action command
                act_s = self.parse_as_action(match.group(1))
                if act_s is None:
                    self.err_parse = "Action not recognized."
                    return None
                act_w = act_s.split()
                act = act_w[0]
                tgt = "current" if (len(act_w) < 2) else act_w[1]
                return "{} {} on {}".format(val, act, tgt)

        self.err_parse = self.NO_MATCH
        return None
        
    def parse_as_reset(self, utt):
        """Parse utterance as database reset instruction."""
        syn_list = self.corpus.get("reset", "").splitlines()
        for syn in syn_list:
            pattern = self.pre_parse_re + syn + self.arg_parse_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            return syn
        self.err_parse = self.NO_MATCH
        return None

    
if __name__ == '__main__':
    rospy.init_node('speech_processor')
    speech_processor = SpeechProcessor()
    speech_processor.process_stream()
