import redis
import time

r = redis.Redis()
r.hset("Joy", "but_lb", 1.0)
r.hset("Joy", "spray_high", 0.0)
r.hset("Joy", "spray_mid", 0.0)
r.hset("Joy", "spray_low", 0.0)
time.sleep(1)
r.hset("Joy", "but_lb", 0.0)
