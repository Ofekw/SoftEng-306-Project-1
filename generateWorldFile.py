#!/usr/bin/python

#read config file and save properties into hashmap
config = {}
left_tree = open('world')

with open("config.properties", "r") as f:
    for line in f:
        print(line)
        property = line.split('=')
        config[property[0]] = property[1]

#get number of orchard rows
rows = config.get('orchard.number')

#do the math to calculate the x positions of each row
xpos = []
#orchard size is 1 achre, which which is from -48 to 48 (88)
#width of orchard row 7
TREE_WIDTH = 8
WORLD_WIDTH = 80
#max number of orchards is 10
if rows > 10:
    rows = 10
elif rows < 1:
    rows = 1
width_between_rows = WORLD_WIDTH/(rows)


#Always start from middle (0,0) and branch out if odd number else start from

