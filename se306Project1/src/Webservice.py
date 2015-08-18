"""
Webservice program that creates JSON Strings
from Entity status files

"""

__author__ = 'harry'
import os
import ftplib
import time

class Webservice():


    """
    Creates JSON strings for each entity
    """
    def create_JSON_strings(self):
        count=0
        directory = "./"
        JSON_string = ""
        JSON_string+="aaData\": ["
        for file in os.listdir(directory):
            if file.endswith("pic.sta") or file.endswith("car.sta"):
                count+=1
                f=open(file)
                lines=f.readlines()
                if(lines and lines[0]!="" ):
                    JSON_string+="{\"0\":\""+ lines[0].strip() +"\","
                    JSON_string+="\"1\":\"" +lines[1].strip() +"\","
                    JSON_string+="\"2\":\""+ lines[2].strip() +"\","
                    JSON_string+="\"3\":\""+ lines[3].strip() +"\","
                    JSON_string+="\"4\":\"" + lines[4].strip() +"\","
                    JSON_string+="\"5\":\"" + lines[5].strip() +"\","
                    JSON_string+="\"6\":\"" + lines[6].strip() +"\"},"


        for file in os.listdir(directory):
            if file.endswith("vis.sta") or file.endswith("ani.sta"):
                count+=1
                f=open(file)
                lines=f.readlines()
                if(lines and lines[0]!="" ):
                    JSON_string+="{\"0\":\""+ lines[0].strip() +"\","
                    JSON_string+="\"1\":\"" +lines[1].strip() +"\","
                    JSON_string+="\"2\":\""+ lines[2].strip() +"\","
                    JSON_string+="\"3\":\""+ lines[3].strip() +"\","
                    JSON_string+="\"4\":\"" + lines[4].strip() +"\","
                    JSON_string+="\"5\":\"" + lines[5].strip() +"\","
                    JSON_string+="\"6\":\"""\"},"


        JSON_string = JSON_string[:-1]
        JSON_string+="]}"
        JSON_string = "{\"sEcho\": 1,\"iTotalRecords\": " + str(count)+","+"\"iTotalDisplayRecords\": 50,\"" + JSON_string
        print(JSON_string)
        return JSON_string


    #TODO WORKERS!

    def send_JSON_via_FTP(self,JSON_strings):
        fn = os.path.join(os.path.dirname(__file__),"state_file.json")
        output_file = open(fn, "w")
        output_file.write(JSON_strings)
        output_file.close()
        session = ftplib.FTP('thinkscruffy.com','ros@thinkscruffy.com','network')
        session.cwd("/public_html/ros/")
        file = open("state_file.json", "rb")
        session.storbinary('STOR state_file.json', file)     # send the file
        file.close()                                    # close file and FTP
        session.quit()


    def updater(self):
        print("UPDATER")
        JSON_strings = self.create_JSON_strings()
        self.send_JSON_via_FTP(JSON_strings)


if __name__ == '__main__':
    service = Webservice()
    while True:
       service.updater()
       time.sleep(0.5)





