#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import *
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager(object):
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Handle input and output to/from users
        self.text_sub = rospy.Subscriber("text_in", String,
                                          self.cmdInputCb)
        self.text_pub = rospy.Publisher("text_out", String,
                                          queue_size=10)
        self.speech_sub = rospy.Subscriber("speech_in", String,
                                          self.cmdInputCb)
        self.speech_pub = rospy.Publisher("speech_out", String,
                                          queue_size=10)

        # Handle input and output from task manager node
        self.task_sub = rospy.Subscriber("task_out", String,
                                         self.taskOutCb)
        self.task_pub = rospy.Publisher("task_in", TaskMsg,
                                           queue_size=10)

        # Sends instructions to rule manager node
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)

        # Sends agent introductions to agent tracker node
        self.agent_pub = rospy.Publisher("agent_input", AgentMsg,
                                         queue_size=10)
        # Sends ownership claims to object tracker node
        self.owner_pub = rospy.Publisher("owner_input", PredicateMsg,
                                         queue_size=10)
        
        # Looks up rule database from rule manager
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

        # Services that reset various databases
        self.reset = dict()
        self.reset["perms"] = rospy.ServiceProxy("reset_perms", Trigger)
        self.reset["rules"] = rospy.ServiceProxy("reset_rules", Trigger)
        self.reset["claims"] = rospy.ServiceProxy("reset_claims", Trigger)
        self.reset["objects"] = rospy.ServiceProxy("reset_objects", Trigger)
        self.reset["agents"] = rospy.ServiceProxy("reset_agents", Trigger)
        self.reset["simulation"] = rospy.ServiceProxy("simulation/reset",
                                                      Trigger)
        
        # Services that freeze/unfreeze various databases
        self.freeze = dict()
        self.freeze["perms"] = rospy.ServiceProxy("freeze_perms", SetBool)
        self.freeze["rules"] = rospy.ServiceProxy("freeze_rules", SetBool)

        # Services that disable/enable various functions
        self.disable = dict()
        self.disable["inference"] = \
            rospy.ServiceProxy("disable_inference", SetBool)
        self.disable["extrapolate"] = \
            rospy.ServiceProxy("disable_extrapolate", SetBool)
        
        # Lookup simulated data
        self.simuObjects = rospy.ServiceProxy("simulation/all_objects",
                                              ListObjects)
        self.simuAgents = rospy.ServiceProxy("simulation/all_agents",
                                             ListAgents)
        
    def cmdInputCb(self, msg):
        """Handles dialog input and publishes commands."""
        args = msg.data.split()
        
        if len(args) == 0:
            return
        # List various entities (actions, rules, etc.)
        if args[0] == "list":
            self.handleList(args)
            return
        # Reset various entities (rules, permissions, simulation, etc.)
        if args[0] == "reset":
            self.handleReset(args)
            return
        # Freeze / unfreeze various databases (rules, permissions)
        if args[0] in ["freeze", "unfreeze"]:
            self.handleFreeze(args)
            return
        # Disable / enable various functions (owner inference, extrapolation)
        if args[0] in ["disable", "enable"]:
            self.handleDisable(args)
            return
        # Try parsing as agent introduction
        agent = cmd.parseAsAgent(msg.data)
        if agent:
            self.agent_pub.publish(agent)
            
            return
        # Try parsing a one-shot task (i.e. an action)
        task = cmd.parseAsAction(msg.data)
        if task:
            self.task_pub.publish(task)
            return
        # Try parsing a higher level task
        task = cmd.parseAsTask(msg.data)
        if task:
            self.task_pub.publish(task)
            return
        # Try to parse as ownership claim
        pred = cmd.parseAsPredicate(msg.data)
        if pred:
            if pred.predicate != predicates.OwnedBy.name:
                self.text_pub.publish("Only ownedBy predicate is handled.")
                return
            self.text_pub.publish(str(pred))
            self.owner_pub.publish(pred)
            return
        # Try to parse object-specific permissions
        perm = cmd.parseAsPerm(msg.data)
        if perm:
            self.text_pub.publish(str(perm))
            self.perm_pub.publish(perm)
            return
        # Try to parse rules that govern actions
        rule = cmd.parseAsRule(msg.data)
        if rule:
            self.text_pub.publish(str(rule))
            self.rule_pub.publish(rule)
            return

        # Error out if nothing works
        out = "Could not parse input: {}".format(cmd.error)
        self.text_pub.publish(out)

    def taskOutCb(self, msg):
        """Handles output from TaskManager node."""
        # Just print response for now
        self.text_pub.publish(msg)

    def handleList(self, args):
        """Handles list command."""
        if len(args) < 2:
            out = "\n".join(["List one of the following:"] +
                            ['objects', 'agents', 'predicates',
                             'rules', 'actions', 'tasks'])
        elif args[1] == "objects":
            if len(args) > 2 and args[2] == "simulated":
                objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
                objs.sort(key=lambda o : o.id)
                header_str = "Simulated objects:"
                props = args[3:]
            else:
                objs = sorted(list(Object.universe()), key=lambda o : o.id)
                header_str = "Tracked objects:"
                props = args[2:]
            if len(props) == 0:
                props = ["id", "color", "position", "ownership"]
            else:
                props = ["id"] + props
            obj_strs = [o.toPrint(props) for o in objs]
            out = "\n".join([header_str] + obj_strs)
        elif args[1] == "agents":
            if len(args) > 2 and args[2] == "simulated":
                agents = [Agent.fromMsg(m) for m in self.simuAgents().agents]
                agents.sort(key=lambda a : a.id)
                header_str = "Simulated agents:"
            else:
                agents = sorted(list(Agent.universe()), key=lambda a : a.id)
                header_str = "Tracked agents:"
            agent_strs = ["{:3d} {:10}".format(a.id, a.name) for a in agents]
            out = "\n".join([header_str] + agent_strs)
        elif args[1] == "predicates":
            out = "\n".join(["Available predicates:"] + predicates.db.keys())
        elif args[1] == "rules":
            act_names = args[2:]
            rule_msgs = []
            if len(act_names) == 0:
                act_names = actions.db.keys()
            for a in act_names:
                rule_msgs += self.lookupRules(a).rule_set
            rule_strs = [Rule.fromMsg(m).toPrint() for m in rule_msgs]
            out = "\n".join(["Active rules:"] + rule_strs)
        elif args[1] == "actions":
            out = "\n".join(["Available actions:"] + actions.db.keys())
        elif args[1] == "tasks":
            out = "\n".join(["Available tasks:"] + tasks.db.keys())
        else:
            out = "Keyword not recognized."
        self.text_pub.publish(out)

    def handleReset(self, args):
        """Handles reset command."""
        if len(args) < 2:
            out = "\n".join(["Reset one of the following:"] +
                            self.reset.keys() + ["all"])
            self.text_pub.publish(out)
        elif args[1] in self.reset:
            key = args[1]
            try:
                self.reset[key].wait_for_service(timeout=0.5)
                self.reset[key]()
                return
            except rospy.ROSException:
                out = "Reset service for {} is unavailable.".format(key)
                self.text_pub.publish(out)
                return
        elif args[1] == "all":
            # Try to reset all databases
            for reset_f in self.reset.values():
                try:
                    reset_f.wait_for_service(timeout=0.5)
                    reset_f()
                except rospy.ROSException:
                    # Fail silently for unavailable services
                    pass
            return
        else:
            out = "Keyword not recognized."
            self.text_pub.publish(out)

    def handleFreeze(self, args):
        """Handles freeze command."""
        val = True if args[0] == "freeze" else False
        if len(args) < 2:
            out = "\n".join(["(Un)Freeze one of the following databases:"] +
                            self.freeze.keys() + ["all"])
            self.text_pub.publish(out)
        elif args[1] in self.freeze:
            key = args[1]
            try:
                self.freeze[key].wait_for_service(timeout=0.5)
                self.freeze[key](val)
                return
            except rospy.ROSException:
                out = "Freeze service for {} is unavailable.".format(key)
                self.text_pub.publish(out)
                return
        elif args[1] == "all":
            # Try to freeze all databases
            for freeze_f in self.freeze.values():
                try:
                    freeze_f.wait_for_service(timeout=0.5)
                    freeze_f(val)
                except rospy.ROSException:
                    # Fail silently for unavailable services
                    pass
            return
        else:
            out = "Keyword not recognized."
            self.text_pub.publish(out)

    def handleDisable(self, args):
        """Handles disable command."""
        val = True if args[0] == "disable" else False
        if len(args) < 2:
            out = "\n".join(["Enable/disable one of the following:"] +
                            self.disable.keys() + ["all"])
            self.text_pub.publish(out)
        elif args[1] in self.disable:
            key = args[1]
            try:
                self.disable[key].wait_for_service(timeout=0.5)
                self.disable[key](val)
                return
            except rospy.ROSException:
                out = "Disable service for {} is unavailable.".format(key)
                self.text_pub.publish(out)
                return
        elif args[1] == "all":
            # Try to disable all functions
            for disable_f in self.disable.values():
                try:
                    disable_f.wait_for_service(timeout=0.5)
                    disable_f(val)
                except rospy.ROSException:
                    # Fail silently for unavailable services
                    pass
            return
        else:
            out = "Keyword not recognized."
            self.text_pub.publish(out)
            
if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
