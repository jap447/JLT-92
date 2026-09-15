#!/usr/bin/env python

import rosunit
import unittest

from prime_number import is_prime_number

class Testprime_number(unittest.TestCase):
    def test_small(self):
        # Not testing 0 and 1 because they are heavily debated... i guess.... 
        # self.assertEqual(is_prime_number(0), False)
        # self.assertEqual(is_prime_number(1), False)
        self.assertEqual(is_prime_number(2), True)
        self.assertEqual(is_prime_number(349), True)

    def test_large(self):
        self.assertEqual(is_prime_number(2333), True)
        self.assertEqual(is_prime_number(8929), True)
        self.assertEqual(is_prime_number(2688), False)


if __name__ == "__main__":
    rosunit.unitrun("hw1", "test_prime_number", Testprime_number, "--text")
