#!/usr/bin/env python3
"""Run every test suite with a bare Python — no deps, no radio, no ROS.

    python run_tests.py

Mirrors go2-rf-survey's discipline: the parsing logic is pure so it can be
pinned before any hardware exists.
"""

import sys
import unittest

if __name__ == "__main__":
    suite = unittest.TestLoader().discover("tests")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
