#!/usr/bin/env python

import serial
import time
import array

class _CommsEngine:
  def __init__(self, serialdevice):
    self.serialdevice = serialdevice
    self.serialdevice.setTimeout(0.25)

  def start(self, buf, bytes_expected, zigbeeAddr = 0):
    mymsg = bytearray(buf)
    sendbuf = bytearray([mymsg[0], mymsg[1]+5, (zigbeeAddr>>8)&0x00ff, zigbeeAddr&0x00ff, 0])
    sendbuf += bytearray(buf)
    self.recvbuf = bytearray([])
    # Try x times in case of errors
    # Send the packet
    # print "Send: {}".format(map(hex, sendbuf))
    #print "Sent: {}".format(bytes(sendbuf))
    for i in range(3):
      self.serialdevice.write(sendbuf)
      try:
        for _ in range(10):
          # Wait for a response
          recvbytes = bytearray(self.serialdevice.read((bytes_expected+6)-len(self.recvbuf)))
          self.recvbuf += recvbytes
          if len(self.recvbuf) < bytes_expected+5:
            if len(self.recvbuf) > 0 and self.recvbuf[0] == 0xff:
              # An error occured
              raise IOError("Received error response from robot.")
          else:
            break
        # print "Recv: {}".format(map(hex, self.recvbuf))
        return self.recvbuf[5:]
      except:
        if i == 2:
          raise
        else:
          pass

class Linkbot:
  def __init__(self):
    self.zigbeeAddr = 0
    pass

  def connectWithTTY(self, ttyfilename):
    self.serialdevice = serial.Serial(ttyfilename, baudrate=230400)
    self.comms = _CommsEngine(self.serialdevice)

  def getID(self):
    buf = bytearray([0x61, 0x03, 0x00])
    recv = self.comms.start(buf, 7, self.zigbeeAddr)
    r = array.array('B', recv[2:6]).tostring()
    return r

  def setBuzzerFrequency(self, freq):
    buf = bytearray([0x6A, 0x05, (freq>>8)&0xff, freq&0xff, 0x00])
    recv = self.comms.start(buf, 3, self.zigbeeAddr)

  def setLEDColor(self, r, g, b):
    buf = bytearray([0x67, 0x08, 0xff, 0xff, 0xff, r, g, b, 0x00])
    recv = self.comms.start(buf, 3, self.zigbeeAddr)

  def setID(self, idstr):
    if len(idstr) != 4:
      raise Exception("The ID String must be an alphanumeric string containing 4 characters.")
    buf = bytearray([0x62, 0x07])
    buf += bytearray(idstr)
    buf += bytearray([0x00])
    recv = self.comms.start(buf, 3, self.zigbeeAddr)

  def disconnect(self):
    self.serialdevice.close()
    
if __name__ == "__main__":
  l = Linkbot()
  l.connectWithTTY('/dev/ttyACM0')
  l.setLEDColor(255, 0, 0)
  time.sleep(1.0)
  print l.getID()
  l.setID(b'abcd')
  l.setBuzzerFrequency(440)
  time.sleep(0.5)
  l.setBuzzerFrequency(0)
