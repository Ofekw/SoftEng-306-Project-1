import unittest
import subprocess
import os.path
import run
import generateRobotFile

class TestStringMethods(unittest.TestCase):
    def setUp(self):
        self.list = run.main(['-t'])

    def test_worldIsWritten(self):
        self.assertEqual(os.path.isfile("./world/myworld.world"), True)

    def test_worldContainsTemplate(self):
        world = open('world/myworld.world').read()
        worldTemplate = open('world/templates/myworld.template').read()
        self.assertTrue(worldTemplate in world)

    def test_worldContainsPicker(self):
        world = open('world/myworld.world').read()
        self.assertTrue("picker( pose" in world)

    def test_worldContainsCarrier(self):
        world = open('world/myworld.world').read()
        self.assertTrue("carrier( pose" in world)

    def test_worldContainsVistor(self):
        world = open('world/myworld.world').read()
        self.assertTrue("visitor( pose" in world)

    def test_worldContainsRightTree(self):
        world = open('world/myworld.world').read()
        self.assertTrue('name "treeRight"' in world)

    def test_worldContainsLeftTree(self):
        world = open('world/myworld.world').read()
        self.assertTrue('name "treeLeft"' in world)

    def tearDown(self):
        generateRobotFile.delete_files(self.list)

if __name__ == '__main__':
    unittest.main()

