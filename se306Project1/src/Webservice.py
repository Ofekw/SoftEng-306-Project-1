__author__ = 'harry'
import os

def create_JSON_strings:
    directory = "./"
    i = 0
    for file in os.listdir(directory):
        if file.endswith("pic.sta")or file.endswith("car.sta"):
            with open(file) as f:
                for line in f:

                    i+=1
    i=0
    for file in os.listdir(directory):
        if file.endswith("vis.sta"):
            with open(file) as f:
                for line in f:
                    i+=1
    i=0
    for file in os.listdir(directory):
        if file.endswith("ani.sta"):
            with open(file) as f:
                for line in f:
                    i+=1
    i=0
