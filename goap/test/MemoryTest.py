'''
Created on Jul 4, 2013

@author: felix
'''
import unittest

from goap.inheriting import Memory


class MemoryTest(unittest.TestCase):

    def setUp(self):
        self.mem = Memory()

    def test_fresh(self):
        self.assertRaises(KeyError, self.mem.get_value, 'undef_var')

    def test_unset_var(self):
        name = 'declared_only'
        self.mem.declare_variable(name)
        self.assertIsNone(self.mem.get_value(name), 'Undeclared variable should give None?')

    def test_consistency(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem.declare_variable(name, value)
        self.assertEqual(self.mem.get_value(name), value, 'Memory variable not consistent')

    def test_declare_none_after_value(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem.declare_variable(name, value)
        self.mem.declare_variable(name)
        self.assertEqual(self.mem.get_value(name), value, 'Memory variable not consistent')


@unittest.skip('removed feature')
class MemoryTestSingletons(unittest.TestCase):

    def setUp(self):
        self.mem1 = Memory()
        self.mem2 = Memory()

    def test_Memory_singleton_inst(self):
        self.assertIs(self.mem1, self.mem2, 'Memory objects not same object')

    def test_Memory_singleton_mem_inst(self):
        self.assertIs(self.mem1._memory, self.mem2._memory, 'Memory objects dicts not same object')

    def test_Memory_singleton_values(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem1.declare_variable(name, value)
        self.assertEqual(self.mem2.get_value(name), value, 'Nonempty Memory objects not equal')

    def test_Memory_singleton_itm_equ(self):
        self.mem1.declare_variable('a', '1')
        self.mem1.declare_variable('b', '2')
        self.mem2.declare_variable('c', '3')
        self.mem2.declare_variable('d', '4')
        self.assertDictEqual(self.mem1._memory, self.mem2._memory, 'Nonempty Memory objects not equal')


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testMemory']
    unittest.main()
