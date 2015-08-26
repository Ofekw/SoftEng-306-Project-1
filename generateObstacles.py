#!/usr/bin/python
import random


def main(row_values, config):
    number = int(config.get('obstacle.number'))
    obstacle = open('world/templates/obstacle.template','r').read()
    myworld = open('world/myworld.world','a+')
    string = ""
    x_values = []  # List of possible locations the obstacles can spawn
    for i in row_values:
        x_values.append(i-4.2)
        x_values.append(i+4.2)
    obstacle_coordinates = []  # List of obstacle coordinates in the world
    if number > 5:
        number = 5
    for i in range(0, number):
        temp = obstacle.replace("obstacle", "rock")

        obstacle_tooNear = True
        while obstacle_tooNear:
            x = x_values[random.randint(0, x_values.__len__()-1)]
            y = random.randint(-4, 33)
            obstacle_tooNear = False
            for j in obstacle_coordinates:
                if j[0] == x and abs(y - j[1]) < 16: # Check if the obstacle is too close in the same row
                    obstacle_tooNear = True
        obstacle_coordinates.append([x,y])

        temp = temp.replace("x", str(x))
        temp = temp.replace("y", str(y))

        string = string + '\n' + temp

    myworld.write(string)
    myworld.close()