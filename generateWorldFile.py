#!/usr/bin/python
import generateObstacles
import sys

def main(config):
    #read config file and save properties into hashmap
    left_tree = open('world/templates/leftTree.template').read()
    right_tree = open('world/templates/rightTree.template').read()
    world_template = open('world/templates/myworld.template').read()
    #get number of orchard rows
    rows = int(config.get('orchard.number'))

    #list of all the tree strings to append to world file
    all_tree_string = []
    #orchard size is 1 achre, which which is from -50 to 50 (100m*100m or 1 acre)
    #width of orchard row 7
    TREE_WIDTH = 8
    WORLD_WIDTH = 80
    #max number of orchards is 10
    if rows > 10:
        rows = 10
    elif rows < 1:
        rows = 1
    width_between_rows = WORLD_WIDTH/(rows)
    row_values = []
    #Always start from middle (0,0) and branch out if odd number else start from

    for x in range(-WORLD_WIDTH/2 + width_between_rows/2,WORLD_WIDTH/2 - width_between_rows/2 + 1, width_between_rows): #x co-cordinates step size
        row_values.append(x)
        tree_column = []
        for y in range(-10, 40, 2):  #safe zone is at -12, so orchard will go from 46 to -12
            temp_left_tree = left_tree.replace('x',str(x-3))
            temp_left_tree = temp_left_tree.replace('y',str(y-1))
            temp_right_tree = right_tree.replace('x',str(x+3))
            temp_right_tree = temp_right_tree.replace('y',str(y+2))
            tree_column.append(temp_left_tree)
            tree_column.append(temp_right_tree)
        all_tree_string.append(''.join(tree_column))

    myworld = open('world/myworld.world','a')

    myworld.write("".join(all_tree_string))
    myworld.close()
    generateObstacles.main(row_values, config)

if __name__ == "__main__":
    main(sys.argv[1:])