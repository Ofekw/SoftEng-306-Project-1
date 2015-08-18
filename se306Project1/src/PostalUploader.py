import ftplib

session = ftplib.FTP('thinkscruffy.com/ros','ros@thinkscruffy.com','network')
file = open('0pic.sta','rb')                  # file to send
session.storbinary('STOR 0pic.sta', file)     # send the file
file.close()                                    # close file and FTP
session.quit()