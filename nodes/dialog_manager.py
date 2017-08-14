#!/usr/bin/env python
import rospy
import nltk
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager:
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Databases of available actions and tasks
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))
        self.task_db = dict(zip([t.name for t in tasks.db], tasks.db))
        
        # Subscribers and publishers
        self.input_sub = rospy.Subscriber("text_input", String, self.inputCb)
        self.command_pub = rospy.Publisher("command", TaskMsg, queue_size=10)

        # Sentence type will be determined by the PoS of the initial word
        self.q_words_PoS = ["WP", "WP$", "WDT"]  # For questions
        self.c_words_PoS = ["VB"]  # For commands
        self.s_words_PoS = ["IN"]  # For statements

    def inputCb(self, msg):
        """Handles text input and publishes commands."""
        name = tasks.Idle.name

        args = msg.data.split()
        pos_tag_args = nltk.pos_tag(args)

        if len(args) == 0:
            return
        if args[0] == "list":
            # List available actions
            print "Available actions:"
            for a in self.action_db.iterkeys():
                print a
            return
        elif args[0] in self.action_db:
            # Try one of the higher-level tasks
            name = args[0]
            tgt = args[1]
            cmd = TaskMsg(name=name, oneshot=False, interrupt=True, target=tgt)
            self.command_pub.publish(cmd)
            return
        elif args[0] in self.task_db:
            # Try one of the higher-level tasks
            name = args[0]
            cmd = TaskMsg(name=name, oneshot=False, interrupt=True, target="")
            self.command_pub.publish(cmd)
            return
        elif pos_tag_args[0][1] in self.c_words_PoS
        # Expects command of the form:
        # "if x true and owns a b true then z false"

        # Chunks predicates and action

            grammar = r"""
            PREDS: {<.*>+}
                    }<RB|IN|CC>{
            """
            cp = nltk.RegexpParser(grammar)
            result = cp.parse(tok_sent)
            preds = [] # list of list of predicates
            action = [] # Action associated w predicate
            print(result)
            for i in result.subtrees():
                if i.label() == 'PREDS':
                    chunks = nltk.chunk.tree2conlltags(i)
                    pred = [p[0] for p in chunks if "and" not in p]
                    preds.append(pred)
            action = preds.pop()
            print(action)
            # TODO Update predicate/rule db
            return
        if name == tasks.Idle.name:
            print "Could not parse input, defaulting to idle task."

if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
