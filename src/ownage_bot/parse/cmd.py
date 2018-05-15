"""Parses text commands as structured ROS messages."""
import re
from .. import objects
from .. import predicates
from .. import actions
from .. import rules
from .. import tasks
from .. import context
from ownage_bot.msg import *

error = ""

NO_MATCH = "Syntax does not match"
NO_ACTION = "Action not recognized"
NO_TASK = "Task not recognized"
NO_PREDICATE = "Predicate not recognized"
NO_ARGS_MATCH = "Number of arguments does not match"
NO_CURRENT_ACT = "Could not look up current action"
NO_CURRENT_TGT = "Could not look up current target"
NO_CURRENT_AGT = "Could not look up current agent"
NO_CURRENT_MATCH = "Could not match current keyword to target or agent"

def asAction(s):
    """Parse a one-shot task (i.e. action) to be performed.
    Syntax: 'ACTION [TARGET]' 
    Example: 'pickUp 5', 'scan'
    """
    global error
    args = s.split()
    if len(args) > 2:
        error = NO_MATCH
        return None
    name = args[0]
    if name not in actions.db:
        error = NO_ACTION
        return None
    args = args[1:]
    n_args = 0 if actions.db[name].tgtype is type(None) else 1
    if len(args) != n_args:
        error = NO_ARGS_MATCH
        return None
    tgt = "" if n_args == 0 else args[0]
    if tgt == "current":
        tgt = context.getCurrentTarget()
        if tgt is None:
            error = NO_CURRENT_TGT
            return None
    msg = TaskMsg(name=name, oneshot=True, interrupt=True, target=tgt)
    return msg

def asTask(s):
    """Parse a higher-level task to be performed.
    Syntax: 'TASK' 
    Example: 'collectAll', 'trashAll'
    """
    global error
    args = s.split()
    if len(args) > 1:
        error = NO_MATCH
        return None
    name = args[0]
    if name not in tasks.db:
        error = NO_TASK
        return None
    msg = TaskMsg(name=name, oneshot=False, interrupt=True, target="")
    return msg

def asPredicate(s, n_unbound=0):
    """Parse predicate bound to some arguments.
    Syntax: '[not] PREDICATE [ARGS]' 
    Example: 'isColored 2 red' (0 unbound), 'not isColored red' (1 unbound)
    """    
    global error
    negated = False
    args = s.split()
    if args[0] == "not":
        negated = True
        args = args[1:]    
    name = args[0]
    if name not in predicates.db:
        error = NO_PREDICATE
        return None
    args = args[1:]
    pred = predicates.db[name]
    if len(args) + n_unbound != pred.n_args:
        error = NO_ARGS_MATCH
    for i in range(len(args)):
        if args[i] == "any":
            args[i] = "_any_"
        elif args[i] == "current":
            if pred.argtypes[i+n_unbound] in actions.tgtypes:
                cur_tgt = context.getCurrentTarget()
                if cur_tgt is None:
                    error = NO_CURRENT_TGT
                    return None
                args[i] = cur_tgt
            elif pred.argtypes[i+n_unbound] == objects.Agent:
                cur_agt = context.getCurrentAgent()
                if cur_agt is None:
                    error = NO_CURRENT_AGT
                    return None
                args[i] = cur_agt.name
            else:
                error = NO_CUR_MATCH
                return None
    args = [objects.Nil.toStr()] * n_unbound + args
    msg = PredicateMsg(predicate=name, bindings=args,
                       negated=negated, truth=1.0)
    return msg

def asPerm(s):
    """Parse target-specific action permissions.
    Syntax: 'forbid|allow ACTION on ID|POSITION' 
    Example: 'allow pickUp on 5'
    """    
    global error
    match = re.match("(forbid|allow) (\S+) on (\S+)", s)
    if match is None:
        error = NO_MATCH
        return None
    name = match.group(2)
    if name == "current":
        name = context.getCurrentAction()
        if name is None:
            error = NO_CURRENT_ACT
            return None
    if name not in actions.db:
        error = NO_ACTION
        return None
    tgt = match.group(3)
    if tgt == "current":
        tgt = context.getCurrentTarget()
        if tgt is None:
            error = NO_CURRENT_TGT
            return None
    truth = float(match.group(1) == "forbid")
    msg = PredicateMsg(predicate=name, bindings=[tgt],
                       negated=False, truth=truth)
    return msg

def asRule(s):
    """Parse deontic rules about actions.
    Syntax: 'forbid|allow ACTION if PREDICATE [ARGS] [and PREDICATE ...]'
    Example: 'forbid trash if isColored red and ownedBy 1'
    """
    global error
    match = re.match("(forbid|allow) (\S+) if (.+)", s)
    if match is None:
        error = NO_MATCH
        return None
    name = match.group(2)
    if name == "current":
        name = context.getCurrentAction()
        if name is None:
            error = NO_CURRENT_ACT
            return None
    if name not in actions.db:
        error = NO_ACTION
        return None
    truth = float(match.group(1) == "forbid")
    preds = match.group(3).strip().split(" and ")
    conditions = [asPredicate(p, n_unbound=1) for p in preds]
    if any([c is None for c in conditions]):
        error = NO_PREDICATE
        return None
    msg = RuleMsg(name, conditions, "forbid", truth)
    return msg
    
def asAgent(s):
    """Parse agent introductions.
    Syntax: 'i am NAME'
    Example: 'i am jake
    """
    args = s.split()
    if args[0] != "i" or args[1] != "am":
        return None
    name = args[2]
    msg = AgentMsg(-1, name, -1)
    return msg
