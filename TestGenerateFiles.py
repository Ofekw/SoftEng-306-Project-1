import unittest
import re
import os.path
import run
import generateRobotFile

class TestGenerateFiles(unittest.TestCase):
    def setUp(self):
        self.list = run.main(['-t'])
        self.config = {}
        with open("config.properties", "r") as f:
            for line in f:
                property = line.split('=')
                self.config[property[0]] = property[1]

    def test_worldIsWritten(self):
        self.assertTrue(os.path.isfile("./world/myworld.world"))

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

    def test_PickerRobotGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Picker\d.py',i):
                number += 1
        self.assertEqual(str(number), self.config.get('picker.number').rstrip())

    def test_CarrierRobotGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Carrier\d.py',i):
                number += 1
        self.assertEqual(str(number), self.config.get('carrier.number').rstrip())

    def test_AnimalGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Animal\d.py',i):
                number += 1
        self.assertEqual(str(number), self.config.get('animal.number').rstrip())

    def test_VisitorGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Visitor\d.py',i):
                number += 1
        self.assertEqual(str(number), self.config.get('visitor.number').rstrip())

    def tearDown(self):
        generateRobotFile.delete_files(self.list)

if __name__ == '__main__':
    unittest.main()

