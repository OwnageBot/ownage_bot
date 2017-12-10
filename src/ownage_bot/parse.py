import re
import objects
import predicates
import actions
import rules
import tasks
from ownage_bot.msg import *

error = ""

NO_MATCH = "Syntax does not match"
NO_ACTION = "Action not recognized"
NO_TASK = "Task not recognized"
NO_PREDICATE = "Predicate not recognized"
NO_ARGS_MATCH = "Number of arguments does not match"

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
    task = TaskMsg(name=name, oneshot=True, interrupt=True, target=tgt)
    return task

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
    task = TaskMsg(name=name, oneshot=False, interrupt=True, target="")
    return task

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
    if len(args) + n_unbound != predicates.db[name].n_args:
        error = NO_ARGS_MATCH
    for i in range(len(args)):
        if args[i] == "any":
            args[i] = "_any_"
    args = [objects.Nil.toStr()] * n_unbound + args
    pred = PredicateMsg(predicate=name, bindings=args,
                        negated=negated, truth=1.0)
    return pred

def asPerm(s):
    """Parse target-specific action permissions.
    Syntax: 'forbid|allow ACTION on ID|POSITION' 
    Example: 'allow pickUp on 5'
    """    
    global error
    match = re.match("(forbid|allow) (\S+) on (\d*)", s)
    if match is None:
        error = NO_MATCH
        return None
    name = match.group(2)
    if name not in actions.db:
        error = NO_ACTION
        return None
    tgt = match.group(3)
    truth = float(match.group(1) == "forbid")
    perm = PredicateMsg(predicate=name, bindings=[tgt],
                        negated=False, truth=truth)
    return perm

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
    if name not in actions.db:
        error = NO_ACTION
        return None
    truth = float(match.group(1) == "forbid")
    preds = match.group(3).strip().split(" and ")
    conditions = [asPredicate(p, n_unbound=1) for p in preds]
    rule = RuleMsg(name, conditions, "forbidden", truth)
    return rule
    
def asAgent(s):
    """Parse agent introductions.
    Syntax: 'i am NAME'
    Example: 'i am jake
    """
    args = s.split()
    if args[0] != "i" or args[1] != "am":
        return None
    name = args[2]
    agent = AgentMsg(-1, name, -1)
    return agent
