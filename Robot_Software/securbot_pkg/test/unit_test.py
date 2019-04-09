#!/usr/bin/env python

PKG = 'datboy'

import sys
import unittest


class UnitTest (unittest.TestCase):
    def test_run(self):
        self.assertEquals(1,2,"1!=1")

    def test_go(self):
        self.assertEquals(1,1,"1!=1")

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'dat_bot', UnitTest)
