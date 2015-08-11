import unittest
import subprocess
import os.path

class TestStringMethods(unittest.TestCase):

  def test_worldGeneration(self):
      process = subprocess.Popen("bash -c 'sleep 2 && python run.py -d'", shell=True)
      process.wait()
      self.assertEqual(os.path.isfile("./world/myworld.world"), True)

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
