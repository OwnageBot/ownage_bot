"""Parses natural speech as text commands."""
import re
import rospy
from .. import objects
from .. import predicates
from .. import actions
from .. import rules
from .. import tasks

error = ""

# Error message constants
NO_MATCH = "Could not match utterance to command"

# Load structured corpus for parsing decoded strings
corpus = rospy.get_param("speech_corpus", dict())
# Load structured corpus for speech responses
replies = rospy.get_param("reply_corpus", dict())

# Regular expressions for parsing
pre_re = "\S* ?\S* ?" # Match two spurious words at utterance start
post_re = " ?\S*$"    # Match one spurious word at utterance end
arg_re = " (.+)"      # Match argument at utterance end

def asNumber(utt):
    """Parse utterance as integer, e.g. 'zero one one' -> 11"""
    global error
    digits = ["zero", "one", "two", "three", "four",
              "five", "six", "seven", "eight", "nine"]
    if len(utt) == 0:
        error = "Cannot parse empty string as number."
        return None
    num = 0
    for w in utt.split():
        if w not in digits:
            # Fail to parse if not all words are digits
            error = "Word does not correspond to digit."
            return None
        num = num*10 + digits.index(w)
    return num

def asIntroduction(utt):
    """Parse utterance as agent introduction, return AgentMsg."""
    syn_list = corpus.get("introductions", "").splitlines()
    agt_list = corpus.get("agents", "").splitlines()
    reply = replies.get("introductions", "hello")
    for syn in syn_list:
        pattern = pre_re + syn + arg_re
        match = re.match(pattern, utt)
        if match is None:
            continue
        agent = match.group(1)
        if agent not in agt_list:
            error = "Agent name not recognized."
            return (None, None)
        return ("i am " + agent, reply)
    error = NO_MATCH
    return (None, None)

def asClaim(utt):
    """Parses utterance as ownership claim."""
    syn_db = corpus.get("claims", dict())
    current_db = syn_db.get("current", dict())
    others_db = syn_db.get("others", dict())
    agt_list = corpus.get("agents", "").splitlines()
    reply = replies.get("claims", "noted")

    # Parse claims made on behalf of current agent
    # e.g. 'that's not mine' -> 'not ownedBy current current'
    for val, syn_list in current_db.iteritems():
        if val not in ["positive", "negative"]:
            continue
        prefix = "not " if val == "negative" else ""
        syn_list = syn_list.splitlines()
        for syn in syn_list:
            pattern = pre_re + syn + post_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            return (prefix + "ownedBy current current", reply)

    # Parse claims made on behalf of other agents
    # e.g. 'this belongs to jake' -> 'ownedBy current jake'
    for val, syn_list in others_db.iteritems():
        if val not in ["positive", "negative"]:
            continue
        prefix = "not " if val == "negative" else ""
        syn_list = syn_list.splitlines()
        for syn in syn_list:
            pattern = (pre_re + syn +
                       " ?(?:agent)?" + arg_re) 
            match = re.match(pattern, utt)
            if match is None:
                continue
            agent = None
            if syn.split()[-1] == "agent":
                # Parse agent by ID
                agent = asNumber(match.group(1))
            elif match.group(1) in agt_list:
                # Parse agent by name
                agent = match.group(1)
            if agent is None:
                error = "Agent not recognized."
                return (None, None)
            return (prefix + "ownedBy current " + str(agent), reply)

    error = NO_MATCH
    return (None, None)

def asReprimand(utt):
    """Parse utterance as reprimand."""
    reprimand_db = corpus.get("reprimands", dict())
    reply = replies.get("reprimands", "sorry")
    for rep, msgs in reprimand_db.iteritems():
        pattern = pre_re + rep + post_re
        match = re.match(pattern, utt)
        if match is None:
            continue
        return (msgs, reply)
    error = NO_MATCH
    return (None, None)        

def asAction(utt):
    """Parse utterance as action command."""
    # Iterate through list of synonyms for each action command
    syn_db = corpus.get("actions", dict())
    reply = replies.get("actions", "certainly")
    for name, syn_list in syn_db.iteritems():
        syn_list = syn_list.splitlines()
        tgt_required = (actions.db[name].tgtype == objects.Object)
        for syn in syn_list:
            if syn.split()[-1] == "object":
                pattern = pre_re + syn + arg_re
            else:
                pattern = pre_re + syn + post_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            if not tgt_required:
                return (name, reply)
            if len(match.groups()) < 1:
                return (name + " " + "current", reply)
            target = asNumber(match.group(1))
            if target is None:
                error = "Action target not recognized."
                return (None, None)
            else:
                return (name + " " + str(target), reply)
    error = NO_MATCH
    return (None, None)

def asTask(utt):
    """Parse utterance as a high-level task."""
    # Iterate through list of synonyms for each task
    syn_db = corpus.get("tasks", dict())
    reply = replies.get("tasks", "certainly")
    for name, syn_list in syn_db.iteritems():
        syn_list = syn_list.splitlines()
        for syn in syn_list:
            pattern = pre_re + syn + post_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            return (name, reply)
    error = NO_MATCH
    return (None, None)

def asPermission(utt):
    """Parses utterance as object specific permission."""
    syn_db = corpus.get("permissions", dict())
    current_db = syn_db.get("current", dict())
    others_db = syn_db.get("others", dict())
    reply = replies.get("permissions", "noted")

    # Parse claims made about current action
    # e.g. 'you can't do that' -> 'forbid current on current'
    for val, syn_list in current_db.iteritems():
        if val not in ["forbid", "allow"]:
            continue
        syn_list = syn_list.splitlines()
        for syn in syn_list:
            pattern = pre_re + syn + post_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            return (val + " current on current", reply)

    # Parse claims made on specific action
    # e.g. 'you can't pick this up' -> 'forbid pickUp on current'
    for val, syn_list in others_db.iteritems():
        if val not in ["forbid", "allow"]:
            continue
        syn_list = syn_list.splitlines()
        for syn in syn_list:
            pattern = pre_re + syn + arg_re
            match = re.match(pattern, utt)
            if match is None:
                continue
            # Parse remaining segment as action command
            act_s = asAction(match.group(1))
            if act_s is None:
                error = "Action not recognized."
                return (None, None)
            act_w = act_s.split()
            act = act_w[0]
            tgt = "current" if (len(act_w) < 2) else act_w[1]
            return ("{} {} on {}".format(val, act, tgt), reply)

    error = NO_MATCH
    return (None, None)

def asReset(utt):
    """Parse utterance as database reset instruction."""
    syn_list = corpus.get("reset", "").splitlines()
    reply = replies.get("reset", "resetting database")
    for syn in syn_list:
        pattern = pre_re + syn + post_re
        match = re.match(pattern, utt)
        if match is None:
            continue
        return (syn, reply)
    error = NO_MATCH
    return (None, None)
