#!/usr/bin/env python
import os
import rospy
import random
import csv
from collections import defaultdict
from std_srvs.srv import *
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleInstructor(object):
    """Automatically teaches rules by example or direct instruction."""

    def __init__(self):
        # Various rates
        self.pub_rate = rospy.Rate(rospy.get_param("~pub_rate", 10))
        self.iter_wait = rospy.get_param("~iter_wait", 1.0)

        # Path to save results
        self.results_path = rospy.get_param("~results_path",
                                            os.getcwd() + "/results.csv")
        
        # Which set of objects to evaluate ('all', 'train' or 'test')
        self.eval_set = rospy.get_param("~eval_set", "all")
        
        # Fraction of objects for which permissions are given
        self.perm_frac = rospy.get_param("~perm_frac", 1.0)
        # Fraction of objects for which permissions and owners are given
        self.perm_own_frac = rospy.get_param("~perm_own_frac", 0.5)
        # Fraction of objects whose owner labels are given
        self.own_frac = rospy.get_param("~own_frac", 1.0)
        # Average ownership probability for the training examples
        self.own_mean = rospy.get_param("~own_mean", 1.0)
        # Half-range of ownership probability for the training examples
        self.own_dev = rospy.get_param("~own_dev", 0.0)

        # Whether to give permissions as feedback in online mode
        self.online_perms = rospy.get_param("~online_perms", True)
        # Whether to give rules as feedback in online mode
        self.online_rules = rospy.get_param("~online_rules", False)
        # Whether to give owners as feedback in online mode
        self.online_owners = rospy.get_param("~online_owners", True)
        # Whether to cancel forbidden actions in online mode
        self.online_cancel = rospy.get_param("~online_cancel", True)        

        # Flag whether online task is complete
        self.online_done = False
        # Count of number of online errors
        self.online_errors = 0
        
        # Handle input and output from task manager node
        self.task_sub = None
        self.task_pub = rospy.Publisher("task_in", TaskMsg,
                                        queue_size=10)
        
        # Publish ownership, permission and rule data
        self.owner_pub = rospy.Publisher("owner_input", PredicateMsg,
                                         queue_size=10)
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)

        # Servers
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        self.listAgents = rospy.ServiceProxy("list_agents", ListAgents)
        self.simuObjects = rospy.ServiceProxy("simulation/all_objects",
                                              ListObjects)
        self.simuAgents = rospy.ServiceProxy("simulation/all_agents",
                                             ListAgents)
        self.setAgents = rospy.ServiceProxy("set_agents", SendAgents)
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

        # Services that reset various databases
        self.reset = dict()
        self.reset["perms"] = rospy.ServiceProxy("reset_perms", Trigger)
        self.reset["rules"] = rospy.ServiceProxy("reset_rules", Trigger)
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

    def resetAll(self):
        """Resets all database to prepare for next instruction trial."""
        for reset_f in self.reset.values():
            try:
                reset_f.wait_for_service(timeout=0.5)
                reset_f()
            except rospy.ROSException:
                # Fail silently for unavailable services
                pass
                
    def loadRules(self):
        """Loads rules from parameter server."""
        rule_strs = rospy.get_param("~rules", [])
        rule_msgs = [parse.cmd.asRule(s) for s in rule_strs]
        if None in rule_msgs:
            raise SyntaxError("Rules were in the wrong syntax")
        rules = [Rule.fromMsg(m) for m in rule_msgs]
        rule_db = dict()
        for r in rules:
            if r.action.name not in rule_db:
                rule_db[r.action.name] = set()
            rule_db[r.action.name].add(r)
        self.rule_db = rule_db

    def loadScript(self):
        """Loads script from parameter server."""
        script_strs = rospy.get_param("~script", [])
        script_msgs = []
        for s in script_strs:
            msg = parse.cmd.asPerm(s)
            if msg:
                script_msgs.append(msg)
                continue
            msg = parse.cmd.asRule(s)
            if msg:
                script_msgs.append(msg)
                continue
            if msg is None:
                raise SyntaxError("Script was in the wrong syntax")
        self.script_msgs = script_msgs
        
    def initializeOnline(self):
        """Register subscribers for online instruction."""
        self.introduceAgents()
        self.loadRules()
        self.object_db = {m.id: Object.fromMsg(m) for m in
                          self.simuObjects().objects}
        self.agent_db = {m.id: Agent.fromMsg(m) for m in
                         self.simuAgents().agents}
        self.task_sub = rospy.Subscriber("task_out", FeedbackMsg,
                                         self.onlineCb)
        self.online_done = False
        self.online_errors = 0

    def shutdownOnline(self):
        """Deregister subscribers for online instruction."""
        self.task_sub.unregister()
        self.task_sub = None
        self.online_done = False
        self.online_errors = 0
        self.object_db.clear()
        self.agent_db.clear()

    def onlineCb(self, msg):
        """Provides commands and permissions in response to actions."""
        # Check if task is complete
        if msg.complete:
            self.online_done = True
            return

        # Give feedback if action allowed and ongoing, or forbidden by rules
        allowed = msg.allowed and not msg.success and msg.failtype != "error"
        forbidden = (msg.failtype == "rule")
        if not allowed and not forbidden:
            return

        action = actions.db[msg.action]
        if action.tgtype is type(None):
            # Ignore actions without targets
            return
        
        target = action.tgtype.fromStr(msg.target)
        if type(target) == Object:
            # Lookup simulated object properties
            target = self.object_db[target.id]
        Object.use_inference = False

        # Evaluate rules for action and all its dependencies
        perms, violations, all_rules = [], [], []
        violated = False
        for a in (action.dependencies + [action]):
            rule_set = list(self.rule_db.get(a.name, []))
            # Check target types
            if a.tgtype != type(target):
                continue
            truth = Rule.evaluateOr(rule_set, target)
            perms.append((a.name, truth))
            all_rules += rule_set
            if truth >= 0.5:
                violated = True
                violations += sorted(rule_set, reverse=True,
                                     key=lambda r : r.evaluate(target))

        # Update number of corrections so far
        incorrect = (allowed and violated) or (forbidden and not violated)
        self.online_errors += 1 if incorrect else 0
                
        if self.online_perms:
            # Publish permissions for the action and all dependencies
            for act_name, truth in perms:
                perm = PredicateMsg(predicate=act_name,
                                    bindings=[msg.target],
                                    truth=truth)
                self.perm_pub.publish(perm)
                rospy.sleep(0.05) # Delay to avoid overloading                

        if self.online_rules:
            # Publish rules that were violated
            for r in violations:
                self.rule_pub.publish(r.toMsg())
                rospy.sleep(0.05) # Delay to avoid overloading                

        if self.online_owners and incorrect:
            # Check for relevant owners if correction was required
            owners_relevant = set()
            for r in all_rules:
                for c in r.conditions:
                    if c.name != predicates.OwnedBy.name:
                        continue
                    agents = c.bindings[1]
                    if agents == objects.Any:
                        owners_relevant.update(self.agent_db.keys())
                    elif type(agents) == list:
                        owners_relevant.update([a.id for a in agents])
                    elif type(c.bindings[1]) == Agent:
                        owners_relevant.add(agents.id)
            # Publish ownership of the object
            for a_id in owners_relevant:
                p_owned = target.getOwnership(a_id)
                ownedBy = PredicateMsg(predicate=predicates.OwnedBy.name,
                                       bindings=[target.toStr(), str(a_id)],
                                       negated=False, truth=p_owned)
                self.owner_pub.publish(ownedBy)
                rospy.sleep(0.05) # Delay to avoid overloading                

        if self.online_cancel:
            if allowed and violated:
                # Skip the current action
                task = TaskMsg(name="skip", oneshot=False, interrupt=False,
                               skip=True, force=False, target="")
                self.task_pub.publish(task)
            elif forbidden and not violated:
                # Force through the current action
                task = TaskMsg(name="force", oneshot=False, interrupt=False,
                               skip=False, force=True, target="")
                self.task_pub.publish(task)
            
    def introduceAgents(self):
        """Looks up all simulated agents and introduces them to the tracker."""
        agents = self.simuAgents().agents
        self.setAgents(agents)

    def introduceOwners(self, objs=None):
        """Provides ownership labels for a subset of the objects.

        objs - List of objects from which the fraction will be sampled.
        """
        # Load objects from simulator if not provided
        if objs is None:
            objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
            objs = [o for o in objs if not o.is_avatar]
        # Sample random fraction objects
        objs = random.sample(objs, int(self.own_frac * len(objs)))
        # Load agents
        agents = self.simuAgents().agents
        # Make sure to look up non-inferred ownership values
        Object.use_inferred = False
        for o in objs:
            for a in agents:
                p_owned = o.getOwnership(a.id)
                # Make ownership labels uncertain
                p_owned = self.own_mean if p_owned > 0.5 else (1-self.own_mean)
                p_owned += random.uniform(-self.own_dev, +self.own_dev)
                p_owned = min(max(p_owned, 0.0), 1.0)
                msg = PredicateMsg(predicate=predicates.OwnedBy.name,
                                   bindings=[o.toStr(), str(a.id)],
                                   negated=False, truth=p_owned)
                self.pub_rate.sleep()
                self.owner_pub.publish(msg)
        return objs
                        
    def instructPerms(self, objs=None):
        """Provides object-specific permissions in one batch.
        
        objs - List of objects from which the fraction will be sampled.
        """
        # Load objects from simulator if not provided
        if objs is None:
            objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
            objs = [o for o in objs if not o.is_avatar]
        # Sample random fraction of (given objects)
        objs = random.sample(objs, int(self.perm_frac * len(objs)))
        # Make sure to look up non-inferred ownership values
        Object.use_inferred = False
        # Feed permissions for each object and action
        for o in objs:
            for act_name, rule_set in self.rule_db.iteritems():
                truth = Rule.evaluateOr(rule_set, o)
                perm = PredicateMsg(predicate=act_name,
                                    bindings=[o.toStr()],
                                    truth=truth)
                self.pub_rate.sleep()
                self.perm_pub.publish(perm)
        return objs

    def instructPermsWithOwners(self, objs=None):
        """Provides both owner labels and permissions for a subest of objects.

        objs - List of objects from which the fraction will be sampled.
        """
        # Load objects from simulator if not provided
        if objs is None:
            objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
            objs = [o for o in objs if not o.is_avatar]
        # Sample random fraction of (given objects)
        objs = random.sample(objs, int(self.perm_own_frac * len(objs)))
        agents = self.simuAgents().agents
        # Make sure to look up non-inferred ownership values
        Object.use_inferred = False
        # Iterate through objects
        for o in objs:
            # Give ownership labels for each agent
            for a in agents:
                # Default to unowned if agent not in ownership database
                p_owned = o.getOwnership(a.id)
                # Make ownership labels uncertain
                p_owned = self.own_mean if p_owned > 0.5 else (1-self.own_mean)
                p_owned += random.uniform(-self.own_dev, +self.own_dev)
                p_owned = min(max(p_owned, 0.0), 1.0)
                msg = PredicateMsg(predicate=predicates.OwnedBy.name,
                                   bindings=[o.toStr(), str(a.id)],
                                   negated=False, truth=p_owned)
                self.pub_rate.sleep()
                self.owner_pub.publish(msg)
            # Give permissions for each action
            for act_name, rule_set in self.rule_db.iteritems():
                truth = Rule.evaluateOr(rule_set, o)
                perm = PredicateMsg(predicate=act_name,
                                    bindings=[o.toStr()],
                                    truth=truth)
                self.pub_rate.sleep()
                self.perm_pub.publish(perm)
        return objs
                
    def instructRules(self):
        """Publish all known rules in one batch."""
        for rule_set in self.rule_db.values():
            for r in rule_set:
                self.pub_rate.sleep()
                self.rule_pub.publish(r.toMsg())

    def instructScript(self):
        """Publish instruction messages in exact order of the script."""
        for msg in self.script_msgs:
            self.pub_rate.sleep()
            if isinstance(msg, PredicateMsg):
                self.perm_pub.publish(msg)
            elif isinstance(msg, RuleMsg):
                self.rule_pub.publish(msg)

    def evaluateRules(self, training_ids=None):
        """Evaluates performance of learned rules against actual rules."""
        objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
        objs = [o for o in objs if not o.is_avatar]
        if training_ids is not None:
            if self.eval_set == "train":
                # Evaluate on training examples
                objs = [o for o in objs if o.id in training_ids]
            elif self.eval_set == "test":
                # Evaluate on testing examples
                objs = [o for o in objs if o.id not in training_ids]
        acts = self.rule_db.keys()
        
        metrics = defaultdict(lambda : defaultdict(float))
        headers = ["accuracy", "precision", "recall", "f1"]

        guard_div = lambda x, y, z: z if (y == 0) else x/y
        f1 = lambda x, y: guard_div(2*x*y, (x+y), 0.0)

        # Compute performance metrics
        Object.use_inferred = False
        for act in acts:
            actual_rules = self.rule_db[act]
            learned_rules = [Rule.fromMsg(m) for m in
                             self.lookupRules(act).rule_set]
            n_pos_actual = 0
            n_pos_learned = 0
            for o in objs:
                actual_perm = Rule.evaluateOr(actual_rules, o)
                learned_perm = Rule.evaluateOr(learned_rules, o)
                correct = (actual_perm-0.5)*(learned_perm-0.5) >= 0
                metrics[act]["accuracy"] += correct
                if actual_perm >= 0.5:
                    n_pos_actual += 1
                    metrics[act]["recall"] += correct
                if learned_perm >= 0.5:
                    n_pos_learned += 1
                    metrics[act]["precision"] += correct

            metrics[act]["accuracy"] =\
                guard_div(metrics[act]["accuracy"], len(objs), 1.0)
            metrics[act]["recall"] =\
                guard_div(metrics[act]["recall"], n_pos_actual, 1.0)
            metrics[act]["precision"] =\
                guard_div(metrics[act]["precision"], n_pos_learned, 1.0)
            metrics[act]["f1"] =\
                f1(metrics[act]["precision"], metrics[act]["recall"]) 

        # Average across actions
        for k in headers:
            metrics["average"][k] =\
                sum([metrics[act][k] for act in acts]) / len(acts)

        # Print metrics
        self.printResults(headers, metrics, topleft="action")
                                        
        return metrics

    def evaluateOwnership(self, training_ids=None):
        """Evaluates accuracy of predicted against actual ownership."""
        true_objs = [Object.fromMsg(m) for m in self.simuObjects().objects]
        true_objs = [o for o in true_objs if not o.is_avatar]

        pred_objs = [Object.fromMsg(m) for m in self.listObjects().objects]
        pred_objs = [o for o in pred_objs if not o.is_avatar]

        if training_ids is not None:
            if self.eval_set == "train":
                # Evaluate on training examples
                true_objs = [o for o in true_objs if o.id in training_ids]
                pred_objs = [o for o in pred_objs if o.id in training_ids]
            elif self.eval_set == "test":
                # Evaluate on testing examples
                true_objs = [o for o in true_objs if o.id not in training_ids]
                pred_objs = [o for o in pred_objs if o.id not in training_ids]
        true_objs = dict(zip([o.id for o in true_objs], true_objs))
        pred_objs = dict(zip([o.id for o in pred_objs], pred_objs))

        agents = self.simuAgents().agents

        metrics = defaultdict(lambda : defaultdict(float))
        headers = ["accuracy", "precision", "recall", "f1",
                   "p-accuracy", "p-precision", "p-recall", "p-f1"]

        guard_div = lambda x, y, z: z if (y == 0) else x/y
        f1 = lambda x, y: guard_div(2*x*y, (x+y), 0.0)

        # Make sure to evaluate using inferred ownership values
        Object.use_inferred = True
        # Compute performance metrics
        for a in agents:
            n_pos_true, f_pos_true = 0, 0
            n_pos_pred, f_pos_pred = 0, 0
            for o_id, true_obj in true_objs.iteritems():
                pred_obj = pred_objs[o_id]
                true_own = true_obj.getOwnership(a.id)
                pred_own = pred_obj.getOwnership(a.id)
                correct = (true_own-0.5)*(pred_own-0.5) > 0
                # Compute deterministic metrics (with cutoff=0.5)
                metrics[a.id]["accuracy"] += correct
                if pred_own >= 0.5:
                    n_pos_pred += 1
                    metrics[a.id]["precision"] += correct
                if true_own >= 0.5:
                    n_pos_true += 1
                    metrics[a.id]["recall"] += correct
                # Compute probabilistic metrics
                f_pos_true += true_own
                f_pos_pred += pred_own
                true_pos = min(true_own, pred_own)
                true_neg = min(1-true_own, 1-pred_own)
                metrics[a.id]["p-accuracy"] += true_pos + true_neg
                metrics[a.id]["p-precision"] += true_pos
                metrics[a.id]["p-recall"] += true_pos
                    
            metrics[a.id]["accuracy"] =\
                guard_div(metrics[a.id]["accuracy"], len(true_objs), 1.0)
            metrics[a.id]["precision"] =\
                guard_div(metrics[a.id]["precision"], n_pos_pred, 1.0)
            metrics[a.id]["recall"] =\
                guard_div(metrics[a.id]["recall"], n_pos_true, 1.0)
            metrics[a.id]["f1"] =\
                f1(metrics[a.id]["precision"], metrics[a.id]["recall"]) 

            metrics[a.id]["p-accuracy"] =\
                guard_div(metrics[a.id]["p-accuracy"], len(true_objs), 1.0)
            metrics[a.id]["p-precision"] =\
                guard_div(metrics[a.id]["p-precision"], f_pos_pred, 1.0)
            metrics[a.id]["p-recall"] =\
                guard_div(metrics[a.id]["p-recall"], f_pos_true, 1.0)
            metrics[a.id]["p-f1"] =\
                f1(metrics[a.id]["p-precision"], metrics[a.id]["p-recall"]) 
            
        # Average across agents
        for k in headers:
            metrics["average"][k] =\
                sum([metrics[a.id][k] for a in agents]) / len(agents)

        # Print metrics
        self.printResults(headers, metrics, topleft="owner")

        return metrics

    def toggleOwnerPrediction(self, def_infer, def_extra):
        inference = rospy.get_param("~disable_inference", -1)
        inference = def_infer if (inference < 0) else bool(inference)
        extrapolate = rospy.get_param("~disable_extrapolate", -1)
        extrapolate = def_extra if (extrapolate < 0) else bool(extrapolate)
        self.disable["inference"](inference)
        self.disable["extrapolate"](extrapolate)
    
    def testRuleLearning(self, n_iters):
        """Test rule learning by providing labelled examples."""
        # Disable ownership inference and extrapolation by default
        self.toggleOwnerPrediction(def_infer=True, def_extra=True)
        
        # Run trials
        avg_metrics = defaultdict(lambda : defaultdict(float))
        for i in range(n_iters):
            print "-- Trial {} --".format(i+1)
            self.resetAll()
            rospy.sleep(self.iter_wait/2.0)
            self.introduceAgents()
            self.loadRules()
            objs = self.introduceOwners()
            objs = self.instructPerms(objs)
            metrics = self.evaluateRules([o.id for o in objs])
            acts = self.rule_db.keys()
            for act in acts:
                for k, v in metrics[act].iteritems():
                    avg_metrics[act][k] += metrics[act][k]
            rospy.sleep(self.iter_wait/2.0)

        # Compute averages
        headers = ["accuracy", "precision", "recall", "f1"]
        for act in avg_metrics.keys():
            for k in avg_metrics[act].keys():
                avg_metrics[act][k] /= n_iters
        for k in headers:
            avg_metrics["average"][k] =\
                sum([avg_metrics[act][k] for act in acts]) / len(acts)

        # Print averages
        print "== Overall performance after {} trials ==".format(n_iters)
        self.printResults(headers, avg_metrics, topleft="action")

        # Save averages
        print "Saving to {}".format(self.results_path)
        self.saveResults(headers, avg_metrics, "action", n_iters)

    def testOwnerInference(self, n_iters):
        """Test ownership inference by providing permissions."""
        # Enable ownership inference, disable extrapolation by default
        self.toggleOwnerPrediction(def_infer=False, def_extra=True)
        # Enable rule instruction, disable rule induction from permissions
        self.freeze["rules"](False)
        self.freeze["perms"](True)

        # Run trials
        agents = self.simuAgents().agents
        avg_metrics = defaultdict(lambda : defaultdict(float))
        for i in range(n_iters):
            print "-- Trial {} --".format(i+1)
            self.resetAll()
            rospy.sleep(self.iter_wait/2.0)
            self.introduceAgents()
            self.loadRules()
            self.instructRules()
            objs = self.instructPerms()
            metrics = self.evaluateOwnership([o.id for o in objs])
            for a in agents:
                for k, v in metrics[a.id].iteritems():
                    avg_metrics[a.id][k] += metrics[a.id][k]
            rospy.sleep(self.iter_wait/2.0)

        # Compute averages
        headers = ["accuracy", "precision", "recall", "f1",
                   "p-accuracy", "p-precision", "p-recall", "p-f1"]
        for a_id in avg_metrics.keys():
            for k in avg_metrics[a_id].keys():
                avg_metrics[a_id][k] /= n_iters
        for k in headers:
            avg_metrics["average"][k] =\
                sum([avg_metrics[a.id][k] for a in agents]) / len(agents)

        # Print averages
        print "== Overall performance after {} trials ==".format(n_iters)
        self.printResults(headers, avg_metrics, topleft="owner")

        # Save averages
        print "Saving to {}".format(self.results_path)
        self.saveResults(headers, avg_metrics, "owner", n_iters)

    def testOwnerPrediction(self, n_iters):
        """Test ownership prediction by providing owners and permissions."""
        # Enable ownership inference and extrapolation by default
        self.toggleOwnerPrediction(def_infer=False, def_extra=False)
        # Whether a set of rules is initially provided
        rules_given = rospy.get_param("~rules_given", False)
        # Whether rule learning from permisssions is enabled
        rule_learning = rospy.get_param("~rule_learning", True)
        # Freeze/unfreeze rule and permission databases accordingly
        self.freeze["rules"](False)
        self.freeze["perms"](not rule_learning)
        
        # Run trials
        agents = self.simuAgents().agents
        avg_metrics = defaultdict(lambda : defaultdict(float))
        for i in range(n_iters):
            print "-- Trial {} --".format(i+1)
            self.resetAll()
            rospy.sleep(self.iter_wait/2.0)
            self.introduceAgents()
            self.loadRules()
            if rules_given:
                self.instructRules()
            objs = self.instructPermsWithOwners()
            metrics = self.evaluateOwnership([o.id for o in objs])
            for a in agents:
                for k, v in metrics[a.id].iteritems():
                    avg_metrics[a.id][k] += metrics[a.id][k]
            rospy.sleep(self.iter_wait/2.0)

        # Compute averages
        headers = ["accuracy", "precision", "recall", "f1",
                   "p-accuracy", "p-precision", "p-recall", "p-f1"]
        for a_id in avg_metrics.keys():
            for k in avg_metrics[a_id].keys():
                avg_metrics[a_id][k] /= n_iters
        for k in headers:
            avg_metrics["average"][k] =\
                sum([avg_metrics[a.id][k] for a in agents]) / len(agents)

        # Print averages
        print "== Overall accuracy after {} trials ==".format(n_iters)
        self.printResults(headers, avg_metrics, topleft="owner")

        # Save averages
        print "Saving to {}".format(self.results_path)
        self.saveResults(headers, avg_metrics, "owner", n_iters)

    def testOnlinePerformance(self, n_iters):
        """Test online performance."""
        # Enable ownership inference and extrapolation by default
        self.toggleOwnerPrediction(def_infer=False, def_extra=False)
        # Unfreeze rule and permission databases accordingly
        self.freeze["rules"](False)
        self.freeze["perms"](False)

        # Set up task to perform
        task = rospy.get_param("~online_task", "collectAll")
        msg = TaskMsg(name=task, oneshot=False, interrupt=True,
                      skip=False, force=False, target="")
        
        # Run trials
        agents = self.simuAgents().agents
        rule_metrics = defaultdict(lambda : defaultdict(float))
        own_metrics = defaultdict(lambda : defaultdict(float))
        avg_error_frac = 0.0
        for i in range(n_iters):
            print "-- Trial {} --".format(i+1)
            self.resetAll()
            rospy.sleep(self.iter_wait/4.0)
            self.initializeOnline()
            rospy.sleep(self.iter_wait/4.0)
            self.online_done = False
            self.task_pub.publish(msg)
            while not rospy.is_shutdown():
                rospy.sleep(self.iter_wait/2.0)
                if self.online_done:
                    break
            n_objects = len([o for o in self.object_db.itervalues() if
                             not o.is_avatar])
            error_frac = float(self.online_errors) / n_objects
            avg_error_frac += error_frac
            print "No. of online errors: {}".format(self.online_errors)
            print "Online error fraction: {}".format(error_frac)
            metrics = self.evaluateRules(self.object_db.keys())
            acts = self.rule_db.keys()
            for act in acts:
                for k, v in metrics[act].iteritems():
                    rule_metrics[act][k] += metrics[act][k]
            metrics = self.evaluateOwnership(self.object_db.keys())
            for a in agents:
                for k, v in metrics[a.id].iteritems():
                    own_metrics[a.id][k] += metrics[a.id][k]
            self.shutdownOnline()
                    
        # Compute averages
        avg_error_frac /= n_iters
        rule_headers = ["accuracy", "precision", "recall", "f1"]
        for act in rule_metrics.keys():
            for k in rule_metrics[act].keys():
                rule_metrics[act][k] /= n_iters
        for k in rule_headers:
            rule_metrics["average"][k] =\
                sum([rule_metrics[act][k] for act in acts]) / len(acts)
        own_headers = ["accuracy", "precision", "recall", "f1",
                       "p-accuracy", "p-precision", "p-recall", "p-f1"]
        for a_id in own_metrics.keys():
            for k in own_metrics[a_id].keys():
                own_metrics[a_id][k] /= n_iters
        for k in own_headers:
            own_metrics["average"][k] =\
                sum([own_metrics[a.id][k] for a in agents]) / len(agents)

        # Print averages
        print "== Overall accuracy after {} trials ==".format(n_iters)
        print "Online error fraction: {}".format(avg_error_frac)
        self.printResults(rule_headers, rule_metrics, topleft="action")
        self.printResults(own_headers, own_metrics, topleft="owner")

        # Save averages
        print "Saving to {}".format(self.results_path)
        self.saveResults(rule_headers, rule_metrics,
                         topleft="action", n_iters=n_iters, append=False)
        self.saveResults(own_headers, own_metrics,
                         topleft="owner", n_iters=n_iters, append=True)
        with open(self.results_path, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow(["errors", avg_error_frac])
        
    def printResults(self, headers, metrics, topleft=""):
        """Print results to screen."""
        print "\t".join([topleft] + [h[:3] for h in headers])
        for row in metrics.keys():
            print "\t".join([str(row)] + [str(metrics[row][k])[:4]
                                          for k in headers])

    def saveResults(self, headers, metrics,
                    topleft="", n_iters=0, append=False):
        """Save results to results path."""
        with open(self.results_path, 'ab' if append else 'wb') as f:
            writer = csv.writer(f)
            if n_iters > 0:
                writer.writerow(["trials", n_iters])
            writer.writerow([topleft] + headers)
            for row in metrics.keys():
                writer.writerow([row] + [metrics[row][k] for k in headers])
            
if __name__ == '__main__':
    rospy.init_node('rule_instructor')
    rule_instructor = RuleInstructor()
    if rospy.get_param("~online", False):
        rule_instructor.initializeOnline()
        rospy.spin()
    else:
        n_iters = rospy.get_param("~n_iters", 1)
        test_mode = rospy.get_param("~test_mode", "rules")
        if test_mode == "rules":
            rule_instructor.testRuleLearning(n_iters)
        elif test_mode == "inference":
            rule_instructor.testOwnerInference(n_iters)
        elif test_mode == "prediction":
            rule_instructor.testOwnerPrediction(n_iters)
        elif test_mode == "online":
            rule_instructor.testOnlinePerformance(n_iters)
            
