import unittest
import subprocess
import os.path
from se306Project import run

class TestStringMethods(unittest.TestCase):

  def test_worldGeneration(self):
      process = run.main()
      self.assertEqual(os.path.isfile("./world/myworld.world"), True)
      process.kill()

  def test_isupper(self):
      self.assertTrue('FOO'.isupper())
      self.assertFalse('Foo'.isupper())

  def test_split(self):
      s = 'hello world'
      self.assertEqual(s.split(), ['hello', 'world'])
      # check that s.split fails when the separator is not a string
      with self.assertRaises(TypeError):
          s.split(2)

if __name__ == '__main__':
    unittest.main()
