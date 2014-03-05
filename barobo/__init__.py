#!/usr/bin/env python

"""
The Barobo Python Module

This python module can be used to control Barobo robots. The easiest way to use
this package is in conjunction with BaroboLink. After connecting to the robots
you want to control in BaroboLink, the following python program will move
joints 1 and 3 on the first connected Linkbot in BaroboLink::

  from barobo import Linkbot
  linkbot = Linkbot()
  linkbot.connect()
  linkbot.moveTo(180, 0, -180)

You may also use this package to control Linkbots without BaroboLink. In that
case, a typical control program will look something like this::
  from barobo import Linkbot, BaroboCtx

  ctx = BaroboCtx()
  ctx.connectDongleTTY('COM3')  # where 'COM3' is the com port the Linkbot is 
                                # connected on. In Windows, the COM port of the
                                # Linkbot can be identified by inspecting the
                                # Device Manager. On a Mac, the com port will
                                # appear in the "/dev/" directory, usually as
                                # something like "/dev/cu.usbmodem1d11". In
                                # Linux, it should be something like 
                                # "/dev/ttyACM0".
  linkbot = ctx.getLinkbot() # or linkbot = ctx.getLinkbot('2B2C') where '2B2C'
                             # should be replaced with the serial ID of your 
                             # Linkbot. Note that the serial ID used here can
                             # be that of a nearby Linkbot that you wish to 
                             # connect to wirelessly. If no serial ID is 
                             # provided, the new linkbot will refer to the 
                             # Linkbot currently connected via USB.
  linkbot.moveTo(180, 0, -180)

For more documentation, please refer to the documentation under the
L{Linkbot<barobo.linkbot.Linkbot>} class.
"""

import struct
import sys
try:
  import Queue
except:
  import queue as Queue
import threading
import barobo._comms as _comms
from barobo.linkbot import Linkbot

ROBOTFORM_MOBOT=1
ROBOTFORM_I=2
ROBOTFORM_L=3
ROBOTFORM_T=4

ROBOT_NEUTRAL=0
ROBOT_FORWARD=1
ROBOT_BACKWARD=2
ROBOT_HOLD=3
ROBOT_POSITIVE=4
ROBOT_NEGATIVE=5

PINMODE_INPUT = 0x00
PINMODE_OUTPUT = 0x01
PINMODE_INPUTPULLUP = 0x02

AREF_DEFAULT = 0x00
AREF_INTERNAL = 0x01
AREF_INTERNAL1V1 = 0x02
AREF_INTERNAL2V56 = 0x03
AREF_EXTERNAL = 0x04

def _getSerialPorts():
  import serial
  if os.name == 'nt':
    available = []
    for i in range(256):
      try:
        s = serial.Serial(i)
        available.append('\\\\.\\COM'+str(i+1))
        s.close()
      except Serial.SerialException:
        pass
    return available
  else:
    from serial.tools import list_ports
    return [port[0] for port in list_ports.comports()]

# Check if a device connected at 'comport' is a Linkbot
def __checkLinkbotTTY(comport):
  import serial
  s = serial.Serial(comport, baudrate=230400)
  s.timeout = 2
  numtries = 0
  maxtries = 3
  while numtries < maxtries:
    try:
      s.write(bytearray([0x30, 0x03, 0x00]))
      r = s.recv(3)
      if r == [0x10, 0x03, 0x11]:
        break
    except:
      if numtries < maxtries:
        numtries+=1
      else:
        return True

def _unpack(fmt, buffer):
  if sys.version_info[0] == 2 and sys.version_info[1] == 6:
    return struct.unpack(fmt, bytes(buffer))
  else:
    return struct.unpack(fmt, str(buffer))


class BaroboException(Exception):
  def __init__(self, *args, **kwargs):
    Exception.__init__(self, *args, **kwargs)

class BaroboCtx():
  """
  The BaroboCtx (BaroboContext) is the entity which manages all of the Linkbots
  in a computational environment. If loosely represents a ZigBee dongle which 
  can communicate and with and control all Linkbots within its communication
  range. 
  """
  RESP_OK = 0x10
  RESP_END = 0x11
  RESP_ERR = 0xff
  RESP_ALREADY_PAIRED = 0xfe

  EVENT_BUTTON = 0x20
  EVENT_REPORTADDRESS = 0x21
  TWI_REGACCESS = 0x22
  EVENT_DEBUG_MSG = 0x23
  EVENT_JOINT_MOVED = 0x24
  EVENT_ACCEL_CHANGED = 0x25


  CMD_STATUS = 0x30
  CMD_DEMO = 0x31
  CMD_SETMOTORDIR = 0x32
  CMD_GETMOTORDIR = 0x33
  CMD_SETMOTORSPEED = 0x34
  CMD_GETMOTORSPEED = 0x35
  CMD_SETMOTORANGLES = 0x36
  CMD_SETMOTORANGLESABS = 0x37
  CMD_SETMOTORANGLESDIRECT = 0x38
  CMD_SETMOTORANGLESPID = 0x39
  CMD_GETMOTORANGLES = 0x3A
  CMD_GETMOTORANGLESABS = 0x3B
  CMD_GETMOTORANGLESTIMESTAMP = 0x3C
  CMD_GETMOTORANGLESTIMESTAMPABS = 0x3D
  CMD_SETMOTORANGLE = 0x3E
  CMD_SETMOTORANGLEABS = 0x3F
  CMD_SETMOTORANGLEDIRECT = 0x40
  CMD_SETMOTORANGLEPID = 0x41
  CMD_GETMOTORANGLE = 0x42
  CMD_GETMOTORANGLEABS = 0x43
  CMD_GETMOTORANGLETIMESTAMP = 0x44
  CMD_GETMOTORSTATE = 0x45
  CMD_GETMOTORMAXSPEED = 0x46
  CMD_GETENCODERVOLTAGE = 0x47
  CMD_GETBUTTONVOLTAGE = 0x48
  CMD_GETMOTORSAFETYLIMIT = 0x49
  CMD_SETMOTORSAFETYLIMIT = 0x4A
  CMD_GETMOTORSAFETYTIMEOUT = 0x4B
  CMD_SETMOTORSAFETYTIMEOUT = 0x4C
  CMD_STOP = 0x4D
  CMD_GETVERSION = 0x4E
  CMD_BLINKLED = 0x4F
  CMD_ENABLEBUTTONHANDLER = 0x50
  CMD_RESETABSCOUNTER = 0x51
  CMD_GETHWREV = 0x52
  CMD_SETHWREV = 0x53
  CMD_TIMEDACTION = 0x54
  CMD_GETBIGSTATE = 0x55
  CMD_SETFOURIERCOEFS = 0x56
  CMD_STARTFOURIER = 0x57
  CMD_LOADMELODY = 0x58
  CMD_PLAYMELODY = 0x59
  CMD_GETADDRESS = 0x5A
  CMD_QUERYADDRESSES = 0x5B
  CMD_GETQUERIEDADDRESSES = 0x5C
  CMD_CLEARQUERIEDADDRESSES = 0x5D
  CMD_REQUESTADDRESS = 0x5E
  CMD_REPORTADDRESS = 0x5F
  CMD_REBOOT = 0x60
  CMD_GETSERIALID = 0x61
  CMD_SETSERIALID = 0x62
  CMD_SETRFCHANNEL = 0x63
  CMD_FINDMOBOT = 0x64
  CMD_PAIRPARENT = 0x65
  CMD_UNPAIRPARENT = 0x66
  CMD_RGBLED = 0x67
  CMD_SETMOTORPOWER = 0x68
  CMD_GETBATTERYVOLTAGE = 0x69
  CMD_BUZZERFREQ = 0x6A
  CMD_GETACCEL = 0x6B
  CMD_GETFORMFACTOR = 0x6C
  CMD_GETRGB = 0x6D
  CMD_PLACEHOLDER201303291416 = 0x6E
  CMD_PLACEHOLDER201304121823 = 0x6F
  CMD_PLACEHOLDER201304152311 = 0x70
  CMD_PLACEHOLDER201304161605 = 0x71
  CMD_PLACEHOLDER201304181705 = 0x72
  CMD_PLACEHOLDER201304181425 = 0x73
  CMD_SET_GRP_MASTER = 0x74
  CMD_SET_GRP_SLAVE = 0x75
  CMD_SET_GRP = 0x76
  CMD_SAVE_POSE = 0x77
  CMD_MOVE_TO_POSE = 0x78
  CMD_IS_MOVING = 0x79
  CMD_GET_MOTOR_ERRORS = 0x7A
  CMD_MOVE_MOTORS = 0x7B
  CMD_TWI_SEND = 0x7C
  CMD_TWI_RECV = 0x7D
  CMD_TWI_SENDRECV = 0x7E
  CMD_PLACEHOLDER201306271044 = 0x7F
  CMD_SMOOTHMOVE = 0x80
  CMD_SETMOTORSTATES = 0x81
  CMD_SETGLOBALACCEL = 0x82
  CMD_PING = 0x89
  CMD_GET_HW_REV = 0x8A
  CMD_SET_HW_REV = 0x8B
  CMD_SET_JOINT_EVENT_THRESHOLD = 0x8C
  CMD_SET_ENABLE_JOINT_EVENT = 0x8D
  CMD_SET_ACCEL_EVENT_THRESHOLD = 0x8E
  CMD_SET_ENABLE_ACCEL_EVENT = 0x8F

  MOTOR_FORWARD = 1
  MOTOR_BACKWARD = 2

  TWIMSG_HEADER = 0x22
  TWIMSG_REGACCESS = 0x01
  TWIMSG_SETPINMODE = 0x02
  TWIMSG_DIGITALWRITEPIN = 0x03
  TWIMSG_DIGITALREADPIN = 0x04
  TWIMSG_ANALOGWRITEPIN = 0x05
  TWIMSG_ANALOGREADPIN = 0x06
  TWIMSG_ANALOGREF = 0x07

  def __init__(self):
    # Queue going to the robot
    self.writeQueue = Queue.Queue() 
    # Queue coming from the robot
    self.readQueue = Queue.Queue()
    # Queue coming from the robot intended for the content
    self.ctxReadQueue = Queue.Queue()
    self.link = None
    self.phys = None
    self.children = [] # List of Linkbots
    self.scannedIDs = {}
    self.scannedIDs_cond = threading.Condition()
    self.giant_lock = threading.Lock()
    pass

  def __init_comms(self):
    self.commsInThread = threading.Thread(target=self._commsInEngine)
    self.commsInThread.daemon = True
    self.commsInThread.start()
    self.commsOutThread = threading.Thread(target=self._commsOutEngine)
    self.commsOutThread.daemon = True
    self.commsOutThread.start()

  def addLinkbot(self, linkbot):
    self.children.append(linkbot)

  def __autoConnectWorker(self, comport):
    pass

  def autoConnect(self):
    import threading
    # Try to connect to all COM ports simultaneously.
    self.__foundComPort = None
    self.__foundComPortCond= threading.Condition()
    myports = _getSerialPorts()
    self.__workerThreads = []
    for port in myports:
      thread = threading.Thread(target=self.__autoConnectWorker, args=(port))
      self.__workerThreads.append(thread)
      thread.daemon = True
      thread.start()
    self.foundComPortCond.acquire()
    while self.__foundComPort == None:
      self.foundComPortCond.wait()

  def connect(self):
    """
    Connect the BaroboContext to BaroboLink.
    """
    # Try to connect to BaroboLink running on localhost
    self.phys = _comms.PhysicalLayer_Socket('localhost', 5768)
    self.link = _comms.LinkLayer_Socket(self.phys, self.handlePacket)
    self.link.start()
    self.__init_comms()

  def connectBluetooth(self, macaddr):
    """
    Connect the BaroboContext to a Bluetooth LinkPod.
    """
    self.phys = _comms.PhysicalLayer_Bluetooth(macaddr)
    #self.link = _comms.LinkLayer_Socket(self.phys, self.handlePacket)
    self.link = _comms.LinkLayer_TTY(self.phys, self.handlePacket)
    self.link.start()
    try:
      self.__init_comms()
      self.__checkStatus()
      self.__getDongleID()
    except:
      raise BaroboException('Could not connect to Bluetooth at {0}'.format(macaddr))

  def connectMobotBluetooth(self, macaddr):
    """
    Connect the BaroboContext to a Bluetooth Mobot or legacy Bluetooth Linkbot.
    """
    self.phys = _comms.PhysicalLayer_Bluetooth(macaddr)
    self.link = _comms.LinkLayer_Socket(self.phys, self.handlePacket)
    self.link.start()
    try:
      self.__init_comms()
    except:
      raise BaroboException('Could not connect to Bluetooth at {0}'.format(macaddr))


  def connectDongleTTY(self, ttyfilename):
    """
    Connect the BaroboCtx to a Linkbot that is connected with a USB cable.
    """
    self.phys = _comms.PhysicalLayer_TTY(ttyfilename)
    self.link = _comms.LinkLayer_TTY(self.phys, self.handlePacket)
    self.link.start()
    try:
      self.__init_comms()
      self.__checkStatus()
      self.__getDongleID()
    except:
      self.phys.close()
      self.link.stop()
      self.connectDongleSFP(ttyfilename)

  def connectDongleSFP(self, ttyfilename):
    """
    Connect the BaroboCtx to a Linkbot using libsfp that is connected with a USB cable.
    """
    self.phys = _comms.PhysicalLayer_TTY(ttyfilename)
    self.link = _comms.LinkLayer_SFP(self.phys, self.handlePacket)
    self.link.start()
    try:
      self.__init_comms()
      self.__checkStatus()
      self.__getDongleID()
    except:
      raise BaroboException('Could not connect to dongle at {0}'.format(ttyfilename))

  def disconnect(self):
    self.link.stop()
    self.phys.disconnect()
    self.children = []

  def handlePacket(self, packet):
    self.readQueue.put(packet)

  def scanForRobots(self):
    buf = [ self.CMD_QUERYADDRESSES, 3, 0x00 ]
    self.writePacket(_comms.Packet(buf, 0x0000))

  def getScannedRobots(self):
    return self.scannedIDs

  def getLinkbot(self, serialID=None, linkbotClass=None):
    if serialID is None:
      self.giant_lock.acquire()
      serialID = list(self.scannedIDs.keys())[0]
      self.giant_lock.release()

    if serialID not in self.scannedIDs:
      self.findRobot(serialID)
      self.waitForRobot(serialID)
    if linkbotClass is None:
      linkbotClass = Linkbot
    l = linkbotClass()
    l.zigbeeAddr = self.scannedIDs[serialID]
    l.serialID = serialID
    l.baroboCtx = self
    self.children.append(l)
    l.form = l.getFormFactor()
    if l.zigbeeAddr != self.zigbeeAddr:
      l._pairParent()
    return l

  def findRobot(self, serialID):
    if serialID in self.scannedIDs:
      return
    buf = bytearray([ self.CMD_FINDMOBOT, 7 ])
    buf += bytearray(serialID.encode('ascii'))
    buf += bytearray([0])
    self.writePacket(_comms.Packet(buf, 0x0000))

  def waitForRobot(self, serialID, timeout=2.0):
    self.scannedIDs_cond.acquire()
    numtries = 0
    while serialID not in self.scannedIDs:
      self.scannedIDs_cond.wait(2)
      numtries += 1
      if numtries >= 3:
        self.scannedIDs_cond.release()
        raise BaroboException('Robot {0} not found.'.format(serialID))
    self.scannedIDs_cond.release()
    return serialID in self.scannedIDs

  def writePacket(self, packet):
    self.writeQueue.put(packet)

  def _commsInEngine(self):
    while True:
      packet = self.readQueue.get(block=True, timeout=None)
      # First, see if these are "Report Address" events. We want to filter
      # those out and use them for our own purposes
      if packet.data[0] == self.EVENT_REPORTADDRESS:
        botid = _unpack('!4s', packet.data[4:8])[0]
        zigbeeAddr = _unpack('!H', packet[2:4])[0]
        if botid not in self.scannedIDs:
          self.scannedIDs_cond.acquire()
          self.scannedIDs[botid.decode('ascii')] = zigbeeAddr
          self.scannedIDs_cond.notify()
          self.scannedIDs_cond.release()
        continue
      elif packet.data[0] == self.EVENT_DEBUG_MSG:
        print (packet.data[2:])
        continue
      # Get the zigbee address from the packet 
      zigbeeAddr = packet.addr
      if 0 == zigbeeAddr:
        self.ctxReadQueue.put(packet)
        continue
      for linkbot in self.children:
        if zigbeeAddr == linkbot.zigbeeAddr:
          linkbot.readQueue.put(packet, block=True)
          break

  def _commsOutEngine(self):
    while True:
      packet = self.writeQueue.get()
      self.link.write(packet.data, packet.addr)

  def __checkStatus(self):
    numtries = 0
    maxtries = 3
    while True:
      buf = [ self.CMD_STATUS, 3, 0x00 ]
      self.writePacket(_comms.Packet(buf, 0x0000))
      try:
        response = self.ctxReadQueue.get(block=True, timeout=2.0)
        break
      except:
        if numtries < maxtries:
          numtries+=1
          continue
        else:
          raise

  def __getDongleID(self):
    numtries = 0
    maxtries = 3
    while True:
      buf = [ self.CMD_GETSERIALID, 3, 0x00 ]
      self.writePacket(_comms.Packet(buf, 0x0000))
      try:
        response = self.ctxReadQueue.get(block=True, timeout=2.0)
        break
      except:
        if numtries < maxtries:
          numtries+=1
          continue
        else:
          raise
    serialID = _unpack('!4s', response[2:6])[0]
    buf = [self.CMD_GETADDRESS, 3, 0x00]
    self.writePacket(_comms.Packet(buf, 0x0000))
    response = self.ctxReadQueue.get(block=True, timeout=2.0)
    zigbeeAddr = _unpack('!H', response[2:4])[0]
    self.zigbeeAddr = zigbeeAddr
    self.scannedIDs[serialID] = zigbeeAddr
   

