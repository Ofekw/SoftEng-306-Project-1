import os    

def main():
      os.system("ps aux | grep rosecore | grep -v 'grep rosecore' | awk '{print $2}' | xargs kill -9")
      os.system("ps aux | grep stageros | grep -v 'grep stageros' | awk '{print $2}' | xargs kill -9")
      os.system("ps aux | grep python | grep -v 'grep python' | awk '{print $2}' | xargs kill -9")

if __name__ == "__main__":
    main()
