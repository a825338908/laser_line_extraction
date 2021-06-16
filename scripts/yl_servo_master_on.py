bashCommand = "rostopic pub -1 /spray_onoff std_msgs/Float32 1.0"
import redis


red = redis.Redis(host="127.0.0.1", port=6379)
red.set("main_switch_servo", "1")