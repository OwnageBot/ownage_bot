# Name of current/last task for which feedback is being given
string task

# Name of current/last action for which feedback is being given
string action

# String representation of current/last target
string target

# Whether or not the task is complete
bool complete

# Whether or not the action was allowed
bool allowed

# Whether or not the last action was successful
bool success

# Whether the action failed because of a 'perm', 'rule', or 'error'
string failtype

# Error message if action failed because of an error
string error

# List of rules violated (if any), ordered by degree of violation
ownage_bot/RuleMsg[] violations
