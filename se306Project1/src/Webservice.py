"""
Webservice program that creates JSON Strings
from Entity status files

"""

__author__ = 'harry'
import os


"""
Creates JSON strings for each entity
"""
def create_JSON_strings():
    directory = "./"
    JSON_string = ""
    JSON_string+="{ \"Robot_Pickers\": ["
    for file in os.listdir(directory):
        if file.endswith("pic.sta"):
            f=open(file)
            lines=f.readlines()
            JSON_string+="{\"Name\":\""+ lines[0].strip() +"\","
            JSON_string+="\"Type\":\"" +lines[1].strip() +"\","
            JSON_string+="\"Current_action\":\""+ lines[2].strip() +"\","
            JSON_string+="\"X_Position\":\""+ lines[3].strip() +"\","
            JSON_string+="\"Y_Position\":\"" + lines[4].strip() +"\","
            JSON_string+="\"Theta(Rads)\":\"" + lines[5].strip() +"\","
            JSON_string+="\"Current_capacity\":\"" + lines[6].strip() +"\"},"

    JSON_string = JSON_string[:-1]
    JSON_string+="],"

    JSON_string+="\"Robot_Carriers\": ["
    for file in os.listdir(directory):
        if file.endswith("car.sta"):
            f=open(file)
            lines=f.readlines()
            JSON_string+="{\"Name\":\""+ lines[0].strip() +"\","
            JSON_string+="\"Type\":\"" +lines[1].strip() +"\","
            JSON_string+="\"Current_action\":\""+ lines[2].strip() +"\","
            JSON_string+="\"X_Position\":\""+ lines[3].strip() +"\","
            JSON_string+="\"Y_Position\":\"" + lines[4].strip() +"\","
            JSON_string+="\"Theta(Rads)\":\"" + lines[5].strip() +"\","
            JSON_string+="\"Current_capacity\":\"" + lines[6].strip() +"\"},"


    JSON_string = JSON_string[:-1]
    JSON_string+="],"

    JSON_string+=" \"Visitors\": ["
    for file in os.listdir(directory):
        if file.endswith("vis.sta"):
            f=open(file)
            lines=f.readlines()
            JSON_string+="{\"Name\":\""+ lines[0].strip() +"\","
            JSON_string+="\"Type\":\"" +lines[1].strip() +"\","
            JSON_string+="\"Current_action\":\""+ lines[2].strip() +"\","
            JSON_string+="\"X_Position\":\""+ lines[3].strip() +"\","
            JSON_string+="\"Y_Position\":\"" + lines[4].strip() +"\","
            JSON_string+="\"Theta(Rads)\":\"" + lines[5].strip() +"\"},"


    JSON_string = JSON_string[:-1]
    JSON_string+="],"

    JSON_string+=" \"Animals\": ["
    for file in os.listdir(directory):
        if file.endswith("ani.sta"):
            f=open(file)
            lines=f.readlines()
            JSON_string+="{\"Name\":\""+ lines[0].strip() +"\","
            JSON_string+="\"Type\":\"" +lines[1].strip() +"\","
            JSON_string+="\"Current_action\":\""+ lines[2].strip() +"\","
            JSON_string+="\"X_Position\":\""+ lines[3].strip() +"\","
            JSON_string+="\"Y_Position\":\"" + lines[4].strip() +"\","
            JSON_string+="\"Theta(Rads)\":\"" + lines[5].strip() +"\"},"

    JSON_string = JSON_string[:-1]
    JSON_string+="]}"
    print(JSON_string)
    return JSON_string

#TODO WORKERS!


create_JSON_strings()
