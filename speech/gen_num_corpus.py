#!/usr/bin/env python
"""Generates numeric part of the corpus."""

entities = ["object", "agent"]
digits = ["zero", "one", "two", "three", "four",
          "five", "six", "seven", "eight", "nine"]

if __name__ == '__main__':
    with open("num_corpus.txt", "w+") as f:
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
