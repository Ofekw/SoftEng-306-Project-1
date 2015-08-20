#!/usr/bin/python
import random

def main(row_values, config):
    number = int(config.get('obstacle.number'))
    obstacle = open('world/templates/obstacle.template','r').read()
    myworld = open('world/myworld.world','a+')
    string = ""
    for i in range(0, number):
        rand = random.randint(0, 1)
        if rand == 1:
            temp = obstacle.replace("obstacle", "rock")
        else:
            temp = obstacle.replace("obstacle", "weed")
        invalid_value = True
        while invalid_value == True:
            collide_tree = False
            x = random.randint(-40, 40)
            for value in row_values:
                if value in range(x-3, x+3):
                    collide_tree = True
            if collide_tree == False:
                temp = temp.replace("x", str(x))
                invalid_value = False
            temp = temp.replace("y", str(random.randint(-12, 46)))

        string = string + '\n' + temp

    myworld.write(string)
    myworld.close()


