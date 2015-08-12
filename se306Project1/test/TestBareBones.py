#!/usr/bin/env python
PKG = 'se306Project1'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest


# A sample python unit test
class TestBareBones(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1 Doesn't Equal 1")

    def test_one_equals_two(self):
        self.assertEquals(1, 2, "1 Doesn't Equal 2")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)