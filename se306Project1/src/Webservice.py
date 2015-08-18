"""
Webservice program that creates JSON Strings
from Entity status files

"""

__author__ = 'harry'
import os
import ftplib



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

def send_JSON_via_FTP(JSON_strings):
    fn = os.path.join(os.path.dirname(__file__),"STATE_FILE.JSON")
    output_file = open(fn, "w")
    output_file.write(JSON_strings)
    output_file.close()
    session = ftplib.FTP('thinkscruffy.com','ros@thinkscruffy.com','network')
    session.cwd("/public_html/ros/")
    file = open("STATE_FILE.JSON", "rb")
    session.storbinary('STOR STATE_FILE.JSON', file)     # send the file
    file.close()                                    # close file and FTP
    session.quit()



JSON_strings = create_JSON_strings()
send_JSON_via_FTP(JSON_strings)
