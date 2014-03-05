#!/usr/bin/env python

import threading
import time
import struct
import math
try:
  import Queue
except:
  import queue as Queue

import barobo
import barobo._comms as _comms
import barobo._util as _util
import barobo.mobot as mobot

class Linkbot(mobot.Mobot):
  """
  The Linkbot Class
  =================

  Each instance of this class can be used to represent a physical Linkbot. The
  member functions of this class can be used to get data, set motor angles,
  beep the buzzer, scan for button events, and more.  

  Blocking and Non-Blocking Member Functions
  ==========================================

  The member functions of the Linkbot class which are responsible for moving
  the joints of the Linkbot can be categorized into two general types of
  functions; "blocking" functions and "non-blocking" functions. A blocking
  function is a function that "hangs" until the complete motion is done,
  whereas a "non-blocking" function returns as soon as the motion begins,
  but does not wait until it is done. In the Linkbot class, all functions
  are blocking unless the have the suffix "NB", such as "Linkbot.moveNB()".

  For example, consider the following lines of code::
    linkbot.move(360, 0, 0)
    linkbot.setBuzzerFrequency(440)
  When the above lines of code are executed, the Linkbot will rotate joint 1
  360 degrees. Once the joint has rotated the full revolution, the buzzer will
  sound. Now consider the following code::
    linkbot.moveNB(360, 0, 0)
    linkbot.setBuzzerFrequency(440)
  For these lines of code, joint 1 also moves 360 degrees. The difference is 
  that with these lines of code, the buzzer will begin emitting a tone as soon
  as the joint begins turning, instead of waiting for the motion to finish.
  This is because the non-blocking version of move() was used, which returns
  immediately after the joint begins moving allowing the setBuzzerFrequency() 
  function to execute as the joint begins moving.

  The L{moveWait()<barobo.linkbot.Linkbot.moveWait>} function can be used to
  block until non-blocking motion functions are finished. For instance, the
  following two blocks of code will accomplish the same task::
    linkbot.move(360, 0, 0)
    linkbot.setBuzzerFrequency(440)

    linkbot.moveNB(360, 0, 0)
    linkbot.moveWait()
    linkbot.setBuzzerFrequency(440)

  """

  def connectBluetooth(self, bluetooth_mac_addr):
    """
    Connect to a bluetooth enabled Linkbot. Note that you must have a bluetooth
    enabled LinkPod connected to your Linkbot for this function to work. Most
    Linkbots do not come with BlueTooth, and instead use a ZigBee like protocol
    to communicate with each other.

    @type bluetooth_mac_addr: string
    @param bluetooth_mac_addr: The MAC address of the bluetooth Linkbot. Should
      be something like '00:06:66:6D:12:34'
    """
    self.zigbeeAddr = 0x0000
    if not self.baroboCtx:
      self.baroboCtx = barobo.BaroboCtx()
      self.baroboCtx.connectBluetooth(bluetooth_mac_addr)
      self.baroboCtx.addLinkbot(self)
      self.zigbeeAddr = self.baroboCtx.zigbeeAddr
    self.checkStatus()
    self.getSerialID()
    self.form = self.getFormFactor()

  def driveTo(self, angle1, angle2, angle3):
    self.driveToNB(angle1, angle2, angle3)
    self.moveWait()

  def driveToNB(self, angle1, angle2, angle3):
    mobot.Mobot.driveToNB(self, angle1, angle2, angle3, 0)

  def disableAccelEventCallback(self):
    buf = bytearray([barobo.BaroboCtx.CMD_SET_ENABLE_ACCEL_EVENT, 4, 0x00, 0x00])
    self._transactMessage(buf)
    self.accelCallbackEnabled = False

  def disableJointEventCallback(self):
    buf = bytearray([barobo.BaroboCtx.CMD_SET_ENABLE_JOINT_EVENT, 4, 0x00, 0x00])
    self._transactMessage(buf)
    self.jointCallbackEnabled = False

  def enableJointEventCallback(self, cb):
    """ 
    Enable the joint event callback. Whenever any joint on the robot moves, the 
    callback function 'cb' will be called. The callback function should be
    of the form cb(millis_timestamp, j1, j2, j3) where 'millis_timestamp' 
    will contain a timestamp from the robot, and j1, j2, and j3 will contain
    the new angles of all of the joints in degrees. 
    """
    buf = bytearray([barobo.BaroboCtx.CMD_SET_ENABLE_JOINT_EVENT, 4, 0x07, 0x00])
    self._transactMessage(buf)
    self.jointcallbackfunc = cb
    self.jointCallbackEnabled = True

  def enableAccelEventCallback(self, cb):
    """
    Enable the acceleration event callback. Whenever a change in acceleration
    happens on the robot, the callback function will be called.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_SET_ENABLE_ACCEL_EVENT, 4, 0x07, 0x00])
    self._transactMessage(buf)
    self.accelcallbackfunc = cb
    self.accelCallbackEnabled = True

  def _getADCVolts(self, adc):
    buf = bytearray([barobo.BaroboCtx.CMD_GETENCODERVOLTAGE, 4, adc, 0])
    response = self._transactMessage(buf)
    voltage = barobo._unpack('<f', response[2:6])[0]
    return voltage

  def getAccelerometerData(self):
    """
    Get the current accelerometer readings

    @rtype: [number, number, number]
    @return: A list of acceleration values in the x, y, and z directions.
      Accelerometer values are in units of "G's", where 1 G is standard earth
      gravitational acceleration (9.8m/s/s)
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETACCEL, 0x03, 0x00])
    response = self._transactMessage(buf)
    values = barobo._unpack('<3h', response[2:8])
    return list(map(lambda x: x/16384.0, values))

  def getBatteryVoltage(self):
    """
    Get the current battery voltage of the Linkbot.

    @rtype: number
    @return: Returns a value in Volts
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETBATTERYVOLTAGE, 0x03, 0x00])
    response = self._transactMessage(buf)
    voltage = barobo._unpack('<f', response[2:6])[0]
    return voltage

  def getBreakoutADC(self, adc):
    """
    Get the ADC (Analog-to-digital) reading from a breakout-board's ADC
    channel. 

    @type adc: number
    @param adc: The ADC channel to get the value from [0, 7]
    @rtype: number
    @return: Value from 0-1023
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_ANALOGREADPIN, adc])
    data = self.twiSendRecv(0x02, buf, 2)
    return barobo._unpack('!h', data)[0]

  def getBreakoutADCVolts(self, adc):
    """
    Get the voltage reading of an ADC pin.

    Note that the voltage is calibrated using the AVCC reference. If the
    reference is changed using the setBreakoutADCReference() function, the
    values reported by this function may no longer be accurate.

    @type adc: number
    @param adc: The ADC channel to get the value from [0, 7]
    @rtype: number
    @return: Floating point voltage from 0 to 5
    """
    return self.getBreakoutADC(adc)/1024.0 * 5

  def getBreakoutDigitalPin(self, pin):
    """
    Read value from digital I/O pin.

    @rtype: integer
    @return: Returns 0 if pin is grounded, or 1 in pin is high.
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_DIGITALREADPIN, pin])
    data = self.twiSendRecv(0x02, buf, 1)
    return data[0]
    
  def getColorRGB(self):
    """
    Get the current RGB values of the rgbled

    @rtype: [number, number, number]
    @return: The red, green, and blue values from 0 to 255
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETRGB, 0x03, 0x00])
    response = self._transactMessage(buf)
    return barobo._unpack('<3B', response[2:5])

  def getHWRev(self):
    """ 
    Get the hardware revision of the Linkbot

    @rtype: [major, minor, micro]
    @return: The major, minor, and micro version numbers
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GET_HW_REV, 0x03, 0x00])
    response = self._transactMessage(buf)
    return barobo._unpack('<3B', response[2:5])

  def getLinkbot(self, addr):
    """
    Use an instance of a Linkbot to get instances to other Linkbots. Note that
    this only works for Linkbots that are connected via Bluetooth or TTY, but
    does not work for Linkbots that are connected to BaroboLink.
    """
    return self.baroboCtx.getLinkbot(addr)

  def getSerialID(self):
    """
    Get the serial ID from the Linkbot

    @return: A four character string
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETSERIALID, 3, 0])
    response = self._transactMessage(buf) 
    botid = barobo._unpack('!4s', response[2:6])[0]
    self.serialID = botid
    return botid

  def isMoving(self):
    buf = bytearray([barobo.BaroboCtx.CMD_IS_MOVING, 3, 0])
    response = self._transactMessage(buf)
    return response[2]

  def move(self, angle1, angle2, angle3):
    self.moveNB(angle1, angle2, angle3)
    self.moveWait()

  def moveNB(self, angle1, angle2, angle3):
    angles = self.getJointAngles()
    self.moveToNB(angle1+angles[0], angle2+angles[1], angle3+angles[2])

  def moveContinuous(self, dir1, dir2, dir3):
    mobot.Mobot.moveContinuous(self, dir1, dir2, dir3, 0)

  def moveTo(self, angle1, angle2, angle3):
    self.moveToNB(angle1, angle2, angle3)
    self.moveWait()

  def moveToNB(self, angle1, angle2, angle3):
    mobot.Mobot.moveToNB(self, angle1, angle2, angle3, 0)

  def _pairParent(self):
    buf = bytearray([barobo.BaroboCtx.CMD_PAIRPARENT, 5])
    buf += bytearray(struct.pack('!H', self.baroboCtx.zigbeeAddr))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def ping(self, numbytes=4):
    import random
    now = time.time()
    buf = bytearray([barobo.BaroboCtx.CMD_PING, 0])
    randbytes = bytearray([random.randint(0, 255) for _ in range(numbytes)])
    buf += randbytes
    buf += bytearray([0x00])
    buf[1] = len(buf)
    response = self._transactMessage(buf, maxTries = 1, timeout = 0.5)
    if response[2:2+numbytes] != randbytes:
      raise barobo.BaroboException(
          'Ping did not receive correct bytes. Expected {0}, got {1}'.format(
            list(map(hex, randbytes)),
            list(map(hex, response)) ))
    return time.time() - now

  def reboot(self):
    """
    Reboot the connect robot. Note that communications with the robot will 
    not succeed while the robot is booting back up.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_REBOOT, 0x03, 0x00])
    self._transactMessage(buf)

  def recordAnglesBegin(self, delay=0.05):
    """
    Begin recording joint angles.

    @type delay: number
    @param delay: The number of seconds to delay between joint angles readings.
    """
    self.recordThread = _LinkbotRecordThread(self, delay)
    self.recordThread.start()

  def recordAnglesEnd(self):
    """ End recording angles and return a list consisting of [time_values,
    joint1angles, joint2angles, joint3angles]"""
    self.recordThread.runflag_lock.acquire()
    self.recordThread.runflag = False
    self.recordThread.runflag_lock.release()
    # Wait for recording to end
    while self.recordThread.isRunning:
      time.sleep(0.5)
    return [map(lambda x: x-self.recordThread.time[0], self.recordThread.time), 
        self.recordThread.angles[0], 
        self.recordThread.angles[1], 
        self.recordThread.angles[2]]

  def recordAnglesPlot(self):
    import pylab
    """Plot recorded angles.

    See recordAnglesBegin() and recordAnglesEnd() to record joint motions.
    """
    pylab.plot(
        self.recordThread.time, 
        self.recordThread.angles[0],
        self.recordThread.time, 
        self.recordThread.angles[1],
        self.recordThread.time, 
        self.recordThread.angles[2])
    pylab.show()

  def setAcceleration(self, accel):
    """
    Set the acceleration of all joints. Each joint motion will begin with this
    acceleration after calling this function. Set the acceleration to 0 to
    disable this feature. 
    """
    buf = bytearray([barobo.BaroboCtx.CMD_SETGLOBALACCEL, 0])
    buf += struct.pack('<f', _util.deg2rad(accel))
    buf += bytearray([0x00])
    buf[1] = len(buf)
    self._transactMessage(buf)

  def setBreakoutAnalogPin(self, pin, value):
    """
    Set an analog output pin (PWM) to a value between 0-255. This can be used
    to set the power of a motor, dim a LED, or more. 

    @type pin: integer from 0-7
    @param pin: The pin parameter must be a pin the supports analog output. 
      These pins are indicated by a tilde (~) symbol next to the pin number
      printed on the breakout board.
    @type value: integer from 0-255
    @param value: The amount to power the pin: 0 is equivalent to no power, 255
      is maximum power.
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_ANALOGWRITEPIN, pin, value])
    self.twiSend(0x02, buf)

  def setBreakoutAnalogRef(self, ref):
    """
    Set the reference voltage of an analog input pin. 

    @type ref: integer from 0-7
    @param ref: Valid values are barobo.AREF_DEFAULT, barobo.AREF_INTERNAL,
      barobo.AREF_INTERNAL1V1, barobo.AREF_INTERNAL2V56, and
      barobo.AREF_EXTERNAL.
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_ANALOGREF, ref])
    self.twiSend(0x02, buf)

  def setBreakoutDigitalPin(self, pin, value):
    """
    Set a digital I/O pin to either a high or low value. The pin will be set
    high if the parameter 'value' is non-zero, or set to ground otherwise.
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_DIGITALWRITEPIN, pin, value])
    self.twiSend(0x02, buf)

  def setBreakoutPinMode(self, pin, mode):
    """
    Set the mode of a digital I/O pin on the breakout board. Valid modes are
    barobo.PINMODE_INPUT, barobo.PINMODE_OUTPUT, and
    barobo.PINMODE_INPUTPULLUP.
    """
    buf = bytearray([barobo.BaroboCtx.TWIMSG_HEADER, barobo.BaroboCtx.TWIMSG_SETPINMODE, pin, mode])
    self.twiSend(0x02, buf)

  def setBuzzerFrequency(self, freq):
    """
    Set the buzzer to begin playing a tone.

    @type freq: number in Hz
    @param freq: The frequency in Hertz (Hz) for the buzzer to play. Set to
      zero to turn the buzzer off.
    """
    buf = bytearray([0x6A, 0x05, (freq>>8)&0xff, freq&0xff, 0x00])
    self._transactMessage(buf)

  def setAccelEventThreshold(self, g_force):
    """
    Set the minimum change in g-force required for the robot to report an
    acceleration event.

    See also: enableAccelEventCallback()

    @type g_force: floating point number
    @param g_force: The acceleration in standard earth gravity "G's".
    """
    assert g_force >= 0
    buf = bytearray([barobo.BaroboCtx.CMD_SET_ACCEL_EVENT_THRESHOLD,
        0])
    buf += bytearray(struct.pack('!H', g_force * 16384))
    buf += bytearray([0x0])
    self._transactMessage(buf)

  def setJointEventThreshold(self, angle):
    """
    Set the minimum amount the joint must move before a joint motion event is
    reported.

    Also see enableJointEventCallback()

    @type angle: floating point number
    @param angle: An angle in degrees.
    """
    assert angle >= 0
    buf = bytearray([barobo.BaroboCtx.CMD_SET_JOINT_EVENT_THRESHOLD,
        0])
    buf += bytearray(struct.pack('<f', angle*math.pi/180.0))
    buf += bytearray([0x0])
    self._transactMessage(buf)

  def setHWRev(self, major, minor, micro):
    """
    Set the HW revision of the Linkbot.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_SET_HW_REV, 0, major, minor, micro, 0x00])
    buf[1] = len(buf)
    self._transactMessage(buf)

  def setJointSpeeds(self, speed1, speed2, speed3):
    self.setJointSpeed(1, speed1)
    self.setJointSpeed(2, speed2)
    self.setJointSpeed(3, speed3)

  def setLEDColor(self, r, g, b):
    """
    Set the LED color

    @type r: number [0, 255]
    @type g: number [0, 255]
    @type b: number [0, 255]
    """

    buf = bytearray([barobo.BaroboCtx.CMD_RGBLED, 8, 0xff, 0xff, 0xff, r, g, b, 0x00])
    self._transactMessage(buf)

  def setMotorPowers(self, power1, power2, power3):
    mobot.Mobot.setMotorPowers(self, power1, power2, power3, 0)

  def setMovementState(self, state1, state2, state3):
    mobot.Mobot.setMovementState(self, state1, state2, state3, 0)

  def smoothMoveTo(self, joint, accel0, accelf, vmax, angle):
    """
    Move a joint to a desired position with a specified amount of starting and
    stopping acceleration.

    @type joint: number
    @param joint: The joint to move
    @type accel0: number
    @param accel0: The starting acceleration, in deg/sec/sec
    @type accelf: number
    @param accelf: The stopping deceleration, in deg/sec/sec
    @type vmax: number
    @param vmax: The maximum velocity for the joint during the motion, in deg/sec
    @type angle: number
    @param angle: The absolute angle to move the joint to, in degrees
    """
    _accel0 = _util.deg2rad(accel0)
    _accelf = _util.deg2rad(accelf)
    _vmax = _util.deg2rad(vmax)
    _angle = _util.deg2rad(angle)
    buf = bytearray([barobo.BaroboCtx.CMD_SMOOTHMOVE, 20, joint-1])
    buf += bytearray(struct.pack('<4f', _accel0, _accelf, _vmax, _angle))
    buf += bytearray([0x00])
    buf[1] = len(buf)
    self._transactMessage(buf)

  def _setSerialID(self, text):
    buf = bytearray([barobo.BaroboCtx.CMD_SETSERIALID, 0])
    buf += bytearray(text)
    buf += bytearray([0x00])
    buf[1] = len(buf)
    self._transactMessage(buf)

  def twiRecv(self, addr, size):
    """
    Receive data from a TWI device. See twiSend() for more details.

    @param addr: The TWI address to send data to.
    @rtype: bytearray
    """
    twibuf = bytearray(data)
    buf = bytearray([barobo.BaroboCtx.CMD_TWI_SEND, len(data)+5, addr, len(data)])
    buf += bytearray(data)
    buf += bytearray([0x00])
    response = self._transactMessage(buf)
    return bytearray(response[2:-1])
   
  def twiSend(self, addr, data):
    """ 
    Send data onto the Two-Wire Interface (TWI) (aka I2c) of the Linkbot.
    Many Linkbot peripherals are located on the TWI bus, including the
    accelerometer, breakout boards, and some sensors. The black phone-jack on
    top of the Linkbot exposes TWI pins where custom or prebuild peripherals
    may be attached.

    @param addr: The TWI address to send data to.
    @type data: iterable bytes
    @param data: The byte data to send to the peripheral
    """
    twibuf = bytearray(data)
    buf = bytearray([barobo.BaroboCtx.CMD_TWI_SEND, len(twibuf)+5, addr, len(twibuf)])
    buf += twibuf
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def twiSendRecv(self, addr, senddata, recvsize):
    """
    Send and receive data from a TWI device attached to the Linkbot. See
    twiSend() and twiRecv() for more details.

    @param addr: The TWI address to send data to.
    @type senddata: iterable
    @param senddata: The byte data to send to the peripheral
    @type recvsize: number
    @param recvsize: Number of bytes expected from TWI device
    @rtype: bytearray
    """
    twibuf = bytearray(senddata)
    buf = bytearray([barobo.BaroboCtx.CMD_TWI_SENDRECV, 0, addr, len(twibuf)])
    buf += twibuf
    buf += bytearray([recvsize, 0x00])
    buf[1] = len(buf)
    response = self._transactMessage(buf)
    return bytearray(response[2:-1])

class _LinkbotRecordThread(threading.Thread):
  def __init__(self, linkbot, delay):
    self.delay = delay
    self.linkbot = linkbot
    self.runflag = False
    self.isRunning = False;
    self.runflag_lock = threading.Lock()
    self.time = []
    self.angles = [ [], [], [] ]
    threading.Thread.__init__(self)
    self.daemon = True

  def run(self):
    self.runflag = True
    self.isRunning = True
    while True:
      self.runflag_lock.acquire()
      if self.runflag == False:
        self.runflag_lock.release()
        break
      self.runflag_lock.release()
      # Get the joint angles and stick them into our data struct
      try:
        numtries = 0
        data = self.linkbot.getJointAnglesTime()
        self.time.append(data[0])
        self.angles[0].append(data[1])
        self.angles[1].append(data[2])
        self.angles[2].append(data[3])
        time.sleep(self.delay)
      except IOError:
        numtries += 1
        if numtries >= 3:
          raise
          break
        continue
    self.isRunning = False
