import urllib2
from urllib import urlencode # new module and function

url = "http://thinkscruffy.com/ros/test.php"
data = {'test':'interesting...'}


encoded_data = urlencode(data)

website = urllib2.urlopen(url, encoded_data)
print website.read()


