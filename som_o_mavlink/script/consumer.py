#!/usr/bin/env python
from __future__ import print_function
import pika
from pymavlink.dialects.v10 import common as mavlink
from pymavlink import mavutil

from builtins import object

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

dummyfile = fifo()
mav = mavlink.MAVLink(dummyfile)


connection = pika.BlockingConnection(pika.ConnectionParameters(
        host='localhost'))
channel = connection.channel()
channel.queue_declare(queue='myQueue', durable=False)

def callback(ch, method, properties, body):
    a = bytearray()
    a.extend(body)
    mavmsg = mav.decode(a)
    mavmsg_type = mavmsg.get_type()
    mavdict = mavmsg.to_dict()
    # print(mavdict)
    if('target_system' in mavdict.keys()):
        system_id_ = mavdict['target_system']
        print(str(system_id_))
        print(mavmsg)


channel.basic_consume(callback,
                      queue='myQueue',
                      no_ack=True)

print(' [*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()