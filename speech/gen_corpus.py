#!/usr/bin/env python
"""Generates numeric part of the corpus."""

import yaml

entities = ["object", "agent"]
digits = ["zero", "one", "two", "three", "four",
          "five", "six", "seven", "eight", "nine"]

def write_numbers(f):
    """Write numeric portion of corpus."""
    for n in range(100):
        s = str(n)
        s = " ".join([digits[int(c)] for c in s])
        f.write(s + "\n")
    f.write("\n")
    for e in entities:
        for d in digits:
            s = "{} {}".format(e, d)
            f.write(s + "\n")
        f.write("\n")
        
def write_introductions(corpus, f):
    for s in corpus["introductions"].splitlines():
        for a in corpus["agents"].splitlines():
            f.write(s + " " + a + "\n")
    f.write("\n")

def write_claims(corpus, f):
    for l in corpus["claims"]["current"].values():
        f.write(l)
    for l in corpus["claims"]["others"].values():
        for s in l.splitlines():
            words = s.split()
            if words[-1] == "to":
                for a in corpus["agents"].splitlines():
                    f.write(s + " " + a + "\n")
            elif words[-1] == "agent":
                for n in digits:
                    f.write(s + " " + n + "\n")                    
    f.write("\n")

def write_whose(corpus, f):
    f.write(corpus["whose"]["current"])
    for s in corpus["whose"]["others"].splitlines():
        for n in digits:
            f.write(s + " " + n + "\n")                    
    f.write("\n")

def write_actions(corpus, f):
    for l in corpus["actions"].values():
        for s in l.splitlines():
            words = s.split()
            if words[-1] == "object":
                for n in digits:
                    f.write(s + " " + n + "\n")
            else:
                f.write(s + "\n")
    f.write("\n")

def write_tasks(corpus, f):
    for l in corpus["tasks"].values():
        f.write(l)
    f.write("\n")

def write_permissions(corpus, f):
    for l in corpus["permissions"]["current"].values():
        f.write(l)
    for l in corpus["permissions"]["others"].values():
        for s in l.splitlines():
            for act_l in corpus["actions"].values():
                for act in act_l.splitlines():
                    f.write(s + " " + act +"\n")
    f.write("\n")

def write_rules(corpus, f):
    for l in corpus["permissions"]["current"].values():
        for s in l.splitlines():
            for pred_d in corpus["predicates"].values():
                for pred in pred_d["positive"].splitlines():
                    f.write(s + " if " + pred +"\n")
                for pred in pred_d["negative"].splitlines():
                    f.write(s + " if " + pred +"\n")
    for l in corpus["permissions"]["others"].values():
        for s in l.splitlines():
            for act_n, act_l in corpus["actions"].items():
                if act_n not in ["pickUp", "putDown", "find",
                                 "release", "trash", "collect"]:
                    continue
                for act in act_l.splitlines():
                    if act.split()[-1] == "object":
                        continue
                    for pred_d in corpus["predicates"].values():
                        for pred in pred_d["positive"].splitlines():
                            f.write(s + " " + act + " if " + pred +"\n")
                        for pred in pred_d["negative"].splitlines():
                            f.write(s + " " + act + " if " + pred +"\n")
    f.write("\n")

def write_why(corpus, f):
    f.write(corpus["why"]["positive"])
    f.write(corpus["why"]["negative"])
    f.write("\n")
    
if __name__ == '__main__':
    with open("structured_corpus.yaml", "r") as f:
        corpus = yaml.load(f)

    with open("corpus.txt", "w+") as f:
        write_numbers(f)
        for k in ["colors", "categories", "agents"]:
            f.write(corpus[k])
            f.write("\n")

        write_introductions(corpus, f)
        write_claims(corpus, f)
        write_whose(corpus, f)

        f.write(corpus["reprimands"])
        f.write("\n")

        write_actions(corpus, f)
        write_tasks(corpus, f)
        write_permissions(corpus, f)
        write_rules(corpus, f)
        write_why(corpus, f)
        
        f.write(corpus["reset"])
        f.write("\n")
