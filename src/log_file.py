
class LogFile:
    def __init__(self, filename, headers):
        self.filename = filename
        self.headers = headers
        with open(self.filename, "w") as file:
            file.write("time")
            for header in headers:
                file.write(",")
                file.write(header)

            file.write('\n')
    #call this every time you want to log, give it the ros timestamp
    def log(self, time, columns):
        try:
            assert(len(columns) == len(self.headers))
        except AssertionError:
            rospy.logwarn("Attempting to log an inconsistant number of values")
            return

        with open(self.filename, "a") as file:
            file.write(time)
            for col in columns:
                file.write(",")
                file.write(col)

            file.write('\n')

