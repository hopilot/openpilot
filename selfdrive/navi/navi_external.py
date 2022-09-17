#!/usr/bin/env python3
import numpy as np
import cereal.messaging as messaging
from common.params import Params

import zmq

# OPKR, this is for getting navi data from external device.
class ENavi():
  def __init__(self):
    self.navi_selection = int(Params().get("OPKRNaviSelect", encoding="utf8"))
    self.spd_limit = 0
    self.safety_distance = 0
    self.sign_type = 0
    self.turn_info = 0
    self.turn_distance = 0

    self.ip_add = Params().get("ExternalDeviceIP", encoding="utf8")

  def navi_data(self):

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
      socket.connect("tcp://" + str(self.ip_add) + ":5555")
    except:
      socket.connect("tcp://127.0.0.1:5555")
      pass
    socket.subscribe("")
    message = str(socket.recv(), 'utf-8')

    if "opkrspdlimit" in message:
      arr = message.split(': ')
      self.spd_limit = arr[1]
    if "opkrspddist" in message:
      arr = message.split(': ')
      self.safety_distance = arr[1]
    if "opkrsigntype" in message:
      arr = message.split(': ')
      self.sign_type = arr[1]
    if "opkrturninfo" in message:
      arr = message.split(': ')
      self.turn_info = arr[1]
    if "opkrdistancetoturn" in message:
      arr = message.split(': ')
      self.turn_distance = arr[1]

  def publish(self, pm):
    if self.navi_selection != 3:
      return

    navi_msg = messaging.new_message('liveENaviData')
    navi_msg.liveENaviData.speedLimit = int(self.spd_limit)
    navi_msg.liveENaviData.safetyDistance = float(self.safety_distance)
    navi_msg.liveENaviData.safetySign = int(self.sign_type)
    navi_msg.liveENaviData.turnInfo = int(self.turn_info)
    navi_msg.liveENaviData.distanceToTurn = float(self.turn_distance)
    pm.send('liveENaviData', navi_msg)

def navid_thread(pm=None):
  navid = ENavi()

  if pm is None:
    pm = messaging.PubMaster(['liveENaviData'])

  while True:
    navid.navi_data()
    navid.publish(pm)


def main(pm=None):
  navid_thread(pm)


if __name__ == "__main__":
  main()
