bashCommand = "rostopic pub -1 /spray_onoff std_msgs/Float32 1.0"
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()