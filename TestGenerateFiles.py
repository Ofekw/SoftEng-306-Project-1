import unittest
import re
import os.path
import run
import generateEntity


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
        self.assertTrue(worldTemplate in world, "Testing worldfile exists")

    def test_worldContainsPicker(self):
        world = open('world/myworld.world').read()
        self.assertTrue("picker( pose" in world, "Testing world file has picker")

    def test_worldContainsCarrier(self):
        world = open('world/myworld.world').read()
        self.assertTrue("carrier( pose" in world,"Testing world file has carrier")

    def test_worldContainsVistor(self):
        world = open('world/myworld.world').read()
        self.assertTrue("visitor( pose" in world,"Testing world file has visitor")

    def test_worldContainsRightTree(self):
        world = open('world/myworld.world').read()
        self.assertTrue('name "treeRight"' in world, "Testing world file has right tree entity")

    def test_worldContainsLeftTree(self):
        world = open('world/myworld.world').read()
        self.assertTrue('name "treeLeft"' in world, "Testing world file has left tree enttity")

    def test_PickerRobotGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Picker\d.py',i):
                number += 1
        config_number = int(self.config.get('picker.number').rstrip())
        if config_number > 4:
            config_number = 4
        self.assertEqual(number, config_number, "Testing picker files generated")

    def test_CarrierRobotGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Carrier\d.py',i):
                number += 1
        config_number = int(self.config.get('carrier.number').rstrip())
        if config_number > 4:
            config_number = 4
        self.assertEqual(number, config_number, "Testing carrier files generated")

    def test_AnimalGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Animal\d.py',i):
                number += 1
        config_number = int(self.config.get('animal.number').rstrip())
        if config_number > 4:
            config_number = 4
        self.assertEqual(number, config_number, "Testing animal files generated")

    def test_VisitorGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Visitor\d.py',i):
                number += 1
        config_number = int(self.config.get('visitor.number').rstrip())
        if config_number > 4:
            config_number = 4
        self.assertEqual(number, config_number, "Testing visitor files generated")

    def test_WorkerGenerated(self):
        number = 0
        path = "./se306Project1/src/"
        files = []
        for i in os.listdir(path):
            if os.path.isfile(os.path.join(path, i)) and re.match('Visitor\d.py',i):
                number += 1
        config_number = int(self.config.get('worker.number').rstrip())
        if config_number > 4:
            config_number = 4
        self.assertEqual(number, config_number, "Testing worker files generated")

    def tearDown(self):
        generateEntity.exit_process(self.list)

if __name__ == '__main__':
    # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', TestGenerateFiles)