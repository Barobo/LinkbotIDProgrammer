#!/usr/bin/env python

import math
import threading
import time
import struct
try:
  import Queue
except:
  import queue as Queue

import barobo
import barobo._comms as _comms
import barobo._util as _util

class Mobot:
  """
  The Mobot Class
  ===============

  Each instance of this class can be used to represent a physical Mobot. The
  member functions of this class can be used to get data, set motor angles,
  scan for button events, and more.  

  Blocking and Non-Blocking Member Functions
  ==========================================

  The member functions of the Mobot class which are responsible for moving
  the joints of the Mobot can be categorized into two general types of
  functions; "blocking" functions and "non-blocking" functions. A blocking
  function is a function that "hangs" until the complete motion is done,
  whereas a "non-blocking" function returns as soon as the motion begins,
  but does not wait until it is done. In the Mobot class, all functions
  are blocking unless the have the suffix "NB", such as "Mobot.moveNB()".

  For example, consider the following lines of code::
    mobot.move(360, 0, 0, 360)
    angles = mobot.getJointAngles()
  When the above lines of code are executed, the Mobot will rotate joint 1
  360 degrees. Once the joint has rotated the full revolution, the joint
  angles will be retrieved and stored in the variable "angles". Now 
  consider the following code::
    mobot.moveNB(360, 0, 0)
    angles = mobot.getJointAngles()
  For these lines of code, joint 1 also moves 360 degrees. The difference is 
  that with these lines of code, the joint angles will be retrieved as soon
  as the joint begins turning, instead of waiting for the motion to finish.
  This is because the non-blocking version of move() was used, which returns
  immediately after the joint begins moving allowing the getJointAngles() 
  function to execute as the joint begins moving.

  The L{moveWait()<barobo.mobot.Mobot.moveWait>} function can be used to
  block until non-blocking motion functions are finished. For instance, the
  following two blocks of code will accomplish the same task::
    mobot.move(360, 0, 0)
    angles = mobot.getJointAngles()

    mobot.moveNB(360, 0, 0)
    mobot.moveWait()
    angles = mobot.getJointAngles()

  """
  def __init__(self):
    self.responseQueue = Queue.Queue()
    self.eventQueue = Queue.Queue()
    self.readQueue = Queue.Queue()
    self.writeQueue = Queue.Queue()
    self.zigbeeAddr = None
    self.serialID = None
    self.baroboCtx = None
    self.messageThread = threading.Thread(target=self.__messageThread)
    self.messageThread.daemon = True
    self.messageThread.start()
    self.messageLock = threading.Lock()
    self.eventThread = threading.Thread(target=self.__eventThread)
    self.eventThread.daemon = True
    self.eventThread.start()

    self.callbackEnabled = False
    self.jointCallbackEnabled = False
    self.accelCallbackEnabled = False

  def checkStatus(self):
    """
    Check to see if the Linkbot is online. Raises an exception if the Linkbot
    is not online.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_STATUS, 3, 0])
    self._transactMessage(buf)

  def connect(self):
    """
    Connect to a Linkbot through BaroboLink
    """
    # Connect to a running instance of BaroboLink
    # First, make sure we have a BaroboCtx
    self.zigbeeAddr = 0x8000
    if not self.baroboCtx:
      self.baroboCtx = barobo.BaroboCtx()
      self.baroboCtx.connect()
      self.baroboCtx.addLinkbot(self)
    self.getSerialID()
    self.form = self.getFormFactor()

  def connectBluetooth(self, bluetooth_mac_addr):
    self.connectMobotBluetooth(bluetooth_mac_addr)

  def connectMobotBluetooth(self, bluetooth_mac_addr):
    """
    Connect to a legacy bluetooth Linkbot or Mobot.

    @type bluetooth_mac_addr: string
    @param bluetooth_mac_addr: The MAC address of the bluetooth Linkbot. Should
      be something like '00:06:66:6D:12:34'
    """
    self.zigbeeAddr = 0x8000
    if not self.baroboCtx:
      self.baroboCtx = barobo.BaroboCtx()
      self.baroboCtx.connectMobotBluetooth(bluetooth_mac_addr)
      self.baroboCtx.addLinkbot(self)
      #self.zigbeeAddr = self.baroboCtx.zigbeeAddr
    self.checkStatus()
    self.form = barobo.ROBOTFORM_MOBOT

  def disableButtonCallback(self):
    """
    Disable the button callback.

    See also: enableButtonCallback()
    """
    self.callbackEnabled = False
    buf = bytearray([barobo.BaroboCtx.CMD_ENABLEBUTTONHANDLER, 0x04, 0x00, 0x00])
    self._transactMessage(buf)

  def disconnect(self):
    """
    Disconnect from the Linkbot.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_UNPAIRPARENT, 3, 0])
    response = self._transactMessage(buf)
    self.baroboCtx.disconnect()
    self.baroboCtx = None

  def driveJointTo(self, joint, angle):
    """
    Drive a single joint to a position as fast as possible, using the on-board
    PID motor controller.

    @type joint: number [1,3]
    @param joint: The joint to move
    @type angle: number
    @param angle: The angle to move the joint to, in degrees
    """
    self.driveJointToNB(joint, angle)
    self.moveWait()

  def driveJointToNB(self, joint, angle):
    """
    Non-blocking version of driveJointTo(). Please see driveJointTo() for more
    details.
    """
    angle = _util.deg2rad(angle)
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORANGLEPID, 0x08, joint-1])
    buf += bytearray(struct.pack('<f', angle))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def driveTo(self, angle1, angle2, angle3, angle4):
    """
    Drive the joints to angles as fast as possible using on-board PID
    controller.

    @type angle1: number
    @param angle1: Position to move the joint, in degrees
    @type angle2: number
    @param angle2: Position to move the joint, in degrees
    @type angle3: number
    @param angle3: Position to move the joint, in degrees
    """
    self.driveToNB(angle1, angle2, angle3, angle4)
    self.moveWait()

  def driveToNB(self, angle1, angle2, angle3, angle4):
    """
    Non-blocking version of driveTo(). See driveTo() for more details
    """
    angle1 = _util.deg2rad(angle1)
    angle2 = _util.deg2rad(angle2)
    angle3 = _util.deg2rad(angle3)
    angle4 = _util.deg2rad(angle4)
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORANGLESPID, 0x13])
    buf += bytearray(struct.pack('<4f', angle1, angle2, angle3, angle4))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def enableButtonCallback(self, callbackfunc, userdata=None):
    """
    Enable button callbacks. This function temporarily disables the
    robot's default button actions. Instead, whenever a button is
    pressed or released, the function given as the parameter 'callbackfunc'
    is called.

    See also: disableButtonCallback()

    @type userdata: Anything
    @param userdata: This is data that will be passed into the callbackfunc
      whenever it is called.
    @type callbackfunc: function: func(buttonMask, buttonDown, userdata) . The 
      'buttonMask' parameter of the callback function will contain a bitmask
      indicating which buttons have changed. The buttonDown parameter
      is another bitmask, indicating the current state of each button.
    """
    self.callbackfunc = callbackfunc
    self.callbackUserData = userdata
    self.callbackEnabled = True  
    buf = bytearray([barobo.BaroboCtx.CMD_ENABLEBUTTONHANDLER, 0x04, 0x01, 0x00])
    self._transactMessage(buf)

  def getFormFactor(self):
    """
    Get the form factor.

    @rtype: Robot Form
    @return: Returns barobo.ROBOTFORM_MOBOT, barobo.ROBOTFORM_I, 
      barobo.ROBOTFORM_L, or barobo.ROBOTFORM_T
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETFORMFACTOR, 0x03, 0x00])
    response = self._transactMessage(buf)
    return response[2] 

  def getJointAngle(self, joint):
    """
    Get the current angle position of a joint.

    @type joint: number
    @param joint: Get the position of this joint. Can be 1, 2, or 3
    @rtype: number
    @return: Return the joint angle in degrees
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETMOTORANGLE, 0x04, joint-1, 0x00])
    response = self._transactMessage(buf)
    return _util.rad2deg(barobo._unpack('<f', response[2:6])[0])

  def getJointAngles(self):
    """
    Get the current joint angles.

    @rtype: [float, float, float]
    @return: The joint angles in degrees
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETMOTORANGLESABS, 3, 0])
    response = self._transactMessage(buf)
    angles = barobo._unpack('<4f', response[2:18])
    return list(map(_util.rad2deg, angles))

  def getJointState(self, joint):
    """
    Get the current state of a joint. Possible return values are
    ROBOT_NEUTRAL=0,
    ROBOT_FORWARD=1,
    ROBOT_BACKWARD=2,
    ROBOT_HOLD=3,
    ROBOT_POSITIVE=4,
    ROBOT_NEGATIVE=5
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETMOTORSTATE, 4, joint, 0x00])
    response = self._transactMessage(buf)
    return response[2]

  def getJointAnglesTime(self):
    """
    Get the joint angles with a timestamp. The timestamp is the number of
    seconds since the robot has powered on.

    @rtype: [numbers]
    @return: [seconds, angle1, angle2, angle3], all angles in degrees
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETMOTORANGLESTIMESTAMPABS, 0x03, 0x00])
    response = self._transactMessage(buf)
    millis = barobo._unpack('<L', response[2:6])[0]
    data = barobo._unpack('<4f', response[6:6+16])
    rc = [millis/1000.0]
    rc += list(map(_util.rad2deg, data))
    return rc

  def getVersion(self):
    """
    Get the firmware version of the Linkbot
    """
    buf = bytearray([barobo.BaroboCtx.CMD_GETVERSION, 3, 0])
    response = self._transactMessage(buf)
    return response[2]
  
  def isMoving(self):
    for i in range(4):
      state = self.getJointState(i)
      if (state != barobo.ROBOT_NEUTRAL) and (state != barobo.ROBOT_HOLD):
        return True 
    return False

  def moveJoint(self, joint, angle):
    """
    Move a joint from it's current position by 'angle' degrees.

    @type joint: number
    @param joint: The joint to move: 1, 2, or 3
    @type angle: number
    @param angle: The number of degrees to move the joint from it's current
      position. For example, "45" will move the joint in the positive direction
      by 45 degrees from it's current location, and "-30" will move the joint
      in the negative direction by 30 degrees.
    """
    curangle = self.getJointAngle(joint)
    self.moveJointTo(joint, curangle + angle)

  def moveJointNB(self, joint, angle):
    """
    Non-blocking version of moveJoint(). See moveJoint() for more details.
    """
    curangle = self.getJointAngle(joint)
    self.moveJointToNB(joint, curangle + angle)

  def moveJointTo(self, joint, angle):
    """
    Move a joint to an angle.

    @type joint: number
    @param joint: The joint to move: 1, 2, or 3
    @type angle: number
    @param angle: The absolute position you want the joint to move to. Values are
      in degrees and can be any value. For example, the value "720" means two full
      rotations in the positive directions past "0".
    """
    self.moveJointToNB(joint, angle)
    self.moveWait()

  def moveJointToNB(self, joint, angle):
    """
    Non-blocking version of moveJointTo. See moveJointTo for more details.
    """
    angle = _util.deg2rad(angle)
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORANGLEABS, 0x08, joint-1])
    buf += bytearray(struct.pack('<f', angle))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def move(self, angle1, angle2, angle3, angle4):
    """
    Move all of the joints on a robot by a number of degrees.

    @type angle1: number
    @param angle1: Number of degrees to move joint 1. Similar for parameters
      'angle2' and 'angle3'.
    """
    self.moveNB(angle1, angle2, angle3, angle4)
    self.moveWait()

  def moveNB(self, angle1, angle2, angle3, angle4):
    """
    Non-blocking version of move(). See move() for more details
    """
    angles = self.getJointAngles()
    self.moveToNB(angle1+angles[0], angle2+angles[1], angle3+angles[2], angle4+angles[3])

  def moveContinuous(self, dir1, dir2, dir3, dir4):
    """
    Make the joints begin moving continuously.

    @type dir1: Barobo Direction Macro
    @param dir1: This parameter may take the following values:
      - ROBOT_NEUTRAL: Makes the joint relax
      - ROBOT_FORWARD: Rotates the joint to move the robot in the "forward"
        direction, if the robot has wheels.
      - ROBOT_BACKWARD: Same as above but backwards
      - ROBOT_HOLD: Hold the joint at its current position
      - ROBOT_POSITIVE: Rotates the joint in the "positive" direction,
        according to the right-hand-rule.
      - ROBOT_NEGATIVE: Same as above but in the negative direction.
    """
    Mobot.setMovementState(self, dir1, dir2, dir3, dir4)

  def moveTo(self, angle1, angle2, angle3, angle4):
    self.moveToNB(angle1, angle2, angle3, angle4)
    self.moveWait()

  def moveToNB(self, angle1, angle2, angle3, angle4):
    """
    Move all joints on the Linkbot. Linkbot-I modules will ignore the 'angle2'
    parameter and Linkbot-L modules will ignore the 'angle3' parmeter.

    This function is the non-blocking version of moveTo(), meaning this
    function will return immediately after the robot has begun moving and will
    not wait until the motion is finished.

    @type angle1: number
    @param angle1: Position to move the joint, in degrees
    @type angle2: number
    @param angle2: Position to move the joint, in degrees
    @type angle3: number
    @param angle3: Position to move the joint, in degrees
    """
    angle1 = _util.deg2rad(angle1)
    angle2 = _util.deg2rad(angle2)
    angle3 = _util.deg2rad(angle3)
    angle4 = _util.deg2rad(angle4)
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORANGLESABS, 0x13])
    buf += bytearray(struct.pack('<4f', angle1, angle2, angle3, 0.0))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def moveWait(self):
    """
    Wait until the current robotic motion is finished.
    """
    while self.isMoving():
      time.sleep(0.1)

  def recordAnglesBegin(self, delay=0.05):
    """
    Begin recording joint angles.

    @type delay: number
    @param delay: The number of seconds to delay between joint angles readings.
    """
    self.recordThread = _MobotRecordThread(self, delay)
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
        self.recordThread.angles[2],
        self.recordThread.angles[3]]

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
        self.recordThread.angles[2],
        self.recordThread.time, 
        self.recordThread.angles[3])
    pylab.show()

  def reset(self):
    """
    Reset the multi-revolution counter on the joints.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_RESETABSCOUNTER, 0x03, 0x00])
    self._transactMessage(buf)

  def resetToZero(self):
    """
    Reset the multi-revolution counter and move all the joints to zero
    positions.
    """
    self.reset()
    self.moveTo(0, 0, 0)

  def resetToZeroNB(self):
    self.reset()
    self.moveToZeroNB()

  def setJointMovementState(self, joint, state):
    """
    Set a joint movement state

    @type state: Barobo Direction Macro
    @param state: This parameter may take the following values:
      - ROBOT_NEUTRAL: Makes the joint relax
      - ROBOT_FORWARD: Rotates the joint to move the robot in the "forward"
        direction, if the robot has wheels.
      - ROBOT_BACKWARD: Same as above but backwards
      - ROBOT_HOLD: Hold the joint at its current position
      - ROBOT_POSITIVE: Rotates the joint in the "positive" direction,
        according to the right-hand-rule.
      - ROBOT_NEGATIVE: Same as above but in the negative direction.
    """
    if (self.form == ROBOTFORM_I) and (joint==3):
      if state == ROBOT_FORWARD:
        state = ROBOT_BACKWARD
      elif state == ROBOT_BACKWARD:
        state = ROBOT_FORWARD
      elif state == ROBOT_POSITIVE:
        state = ROBOT_FORWARD
      elif state == ROBOT_NEGATIVE:
        state = ROBOT_BACKWARD
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORDIR, 0x05, joint-1, state, 0x00])
    self._transactMessage(buf)

  def setJointSpeed(self, joint, speed):
    """
    Set the speed of a joint.

    @type joint: number
    @param joint: The joint to change the speed. Can be 1, 2, or 3
    @type speed: number
    @param speed: The speed to set the joint to, in degrees/second.
    """
    _speed = _util.deg2rad(speed)
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORSPEED, 0x08, joint-1])
    buf += bytearray(struct.pack('<f', _speed))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def setJointSpeeds(self, speed1, speed2, speed3, speed4):
    """
    Set all three motor speed simultaneously. Parameters in degrees/second.
    """
    self.setJointSpeed(1, speed1)
    self.setJointSpeed(2, speed2)
    self.setJointSpeed(3, speed3)
    self.setJointSpeed(4, speed4)
    
  def setJointState(self, joint, state):
    """
    Set a joint's movement state.

    @param joint: The joint id: 1, 2, or 3
    @type state: Barobo Direction Macro
    @param state: This parameter may take the following values:
      - ROBOT_NEUTRAL: Makes the joint relax
      - ROBOT_FORWARD: Rotates the joint to move the robot in the "forward"
        direction, if the robot has wheels.
      - ROBOT_BACKWARD: Same as above but backwards
      - ROBOT_HOLD: Hold the joint at its current position
      - ROBOT_POSITIVE: Rotates the joint in the "positive" direction,
        according to the right-hand-rule.
      - ROBOT_NEGATIVE: Same as above but in the negative direction.
    """
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORDIR, 5, joint-1, state, 0])
    self._transactMessage(buf)

  def setJointStates(self, states, speeds):
    """
    Set the joint states for all 3 joints simultaneously.

    @type states: [state1, state2...]
    @param states: Each state may take the following values:
      - ROBOT_NEUTRAL: Makes the joint relax
      - ROBOT_FORWARD: Rotates the joint to move the robot in the "forward"
        direction, if the robot has wheels.
      - ROBOT_BACKWARD: Same as above but backwards
      - ROBOT_HOLD: Hold the joint at its current position
      - ROBOT_POSITIVE: Rotates the joint in the "positive" direction,
        according to the right-hand-rule.
      - ROBOT_NEGATIVE: Same as above but in the negative direction.
    @type speeds: [speed1, speed2 ...]
    @param speeds: The speeds to set each joint

    """
    if len(states) < 4:
      states += [0]*(4-len(states))
    if len(speeds) < 4:
      speeds += [0.0]*(4-len(speeds))
    speeds = list(map(_util.deg2rad, speeds))
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORSTATES, 23])
    buf += bytearray(states)
    buf += bytearray(struct.pack('<4f', speeds[0], speeds[1], speeds[2], speeds[3]))
    buf += bytearray([0x00])
    self._transactMessage(buf)

  def setMotorPower(self, joint, power):
    """
    Set the power of a motor.

    @type joint: number
    @param joint: The joint to control. Can be 1, 2, or 3
    @type power: integer
    @param power: An integer between -255 and 255.
    """
    joint = joint-1
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORPOWER, 0x0A, 1<<joint])
    buf +=bytearray(struct.pack('>3h', power, power, power))
    buf +=bytearray([0x00])
    self._transactMessage(buf)

  def setMotorPowers(self, power1, power2, power3, power4):
    buf = bytearray([barobo.BaroboCtx.CMD_SETMOTORPOWER, 0x0A, 0x07])
    buf +=bytearray(struct.pack('>4h', power1, power2, power3, power4))
    buf +=bytearray([0x00])
    self._transactMessage(buf)

  def setMovementState(self, state1, state2, state3, state4):
    """
    Set the movement state for all three motors.

    @type state1: movement_state
    @param state1: This parameter may take the following values:
      - ROBOT_NEUTRAL: Makes the joint relax
      - ROBOT_FORWARD: Rotates the joint to move the robot in the "forward"
        direction, if the robot has wheels.
      - ROBOT_BACKWARD: Same as above but backwards
      - ROBOT_HOLD: Hold the joint at its current position
      - ROBOT_POSITIVE: Rotates the joint in the "positive" direction,
        according to the right-hand-rule.
      - ROBOT_NEGATIVE: Same as above but in the negative direction.
    """
    if self.form == barobo.ROBOTFORM_I:
      if state3 == barobo.ROBOT_FORWARD:
        state3 = barobo.ROBOT_BACKWARD
      elif state3 == barobo.ROBOT_BACKWARD:
        state3 = barobo.ROBOT_FORWARD
      elif state3 == barobo.ROBOT_POSITIVE:
        state3 = barobo.ROBOT_FORWARD
      elif state3 == barobo.ROBOT_NEGATIVE:
        state3 = barobo.ROBOT_BACKWARD
    states = [state1, state2, state3, state4]
    buf = bytearray([barobo.BaroboCtx.CMD_TIMEDACTION, 0, 0x07])
    for state in states:
      buf += bytearray([state1, barobo.ROBOT_HOLD])
      buf += bytearray(struct.pack('<i', -1))
    buf += bytearray([0x00])
    buf[1] = len(buf)
    self._transactMessage(buf)

  def stop(self):
    buf = bytearray([barobo.BaroboCtx.CMD_STOP, 0x03, 0x00])
    self._transactMessage(buf)

  def _transactMessage(self, buf, maxTries = 3, timeout = 2.0):
    self.messageLock.acquire()
    numTries = 0
    response = [0xff]
    while numTries < maxTries:
      try:
        # Flush the responseQueue
        while not self.responseQueue.empty():
          self.responseQueue.get(block=False)
        self.baroboCtx.writePacket(_comms.Packet(buf, self.zigbeeAddr))
        response = self.responseQueue.get(block=True, timeout = timeout)
        break
      except:
        if numTries < maxTries:
          numTries+=1
          continue
        else:
          self.messageLock.release()
          raise barobo.BaroboException('Did not receive response from robot.')
    if response[0] != barobo.BaroboCtx.RESP_OK:
      self.messageLock.release()
      raise barobo.BaroboException('Robot returned error status.')
    self.messageLock.release()
    return response

  def __messageThread(self):
    # Scan and act on incoming messages
    while True:
      pkt = self.readQueue.get(block=True, timeout=None)
      if (pkt[0] == barobo.BaroboCtx.RESP_OK) or \
         (pkt[0] == barobo.BaroboCtx.RESP_ERR) or \
         (pkt[0] == barobo.BaroboCtx.RESP_ALREADY_PAIRED):
        self.responseQueue.put(pkt)
      else:
        self.eventQueue.put(pkt)

  def __eventThread(self):
    while True:
      evt = self.eventQueue.get(block=True, timeout=None)
      if (evt[0] == barobo.BaroboCtx.EVENT_BUTTON) and self.callbackEnabled:
        self.callbackfunc(evt[6], evt[7], self.callbackUserData)
      elif (evt[0] == barobo.BaroboCtx.EVENT_JOINT_MOVED) and self.jointCallbackEnabled:
        values = barobo._unpack('<Lfff', evt[2:18]) 
        self.jointcallbackfunc(
            values[0], 
            values[1]*180.0/math.pi,
            values[2]*180.0/math.pi,
            values[3]*180.0/math.pi)
      elif (evt[0] == barobo.BaroboCtx.EVENT_ACCEL_CHANGED) and self.accelCallbackEnabled:
        values =  barobo._unpack('<L', evt[2:6]) + barobo._unpack('>3h', evt[6:12])
        self.accelcallbackfunc(values[0], values[1]/16384.0, values[2]/16384.0, values[3]/16384.0)
      elif evt[0] == barobo.BaroboCtx.EVENT_DEBUG_MSG:
        s = barobo._unpack(s, evt[2:-1])
        print ("Debug msg from {0}: {1}".format(self.serialID, s))

class _MobotRecordThread(threading.Thread):
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
        self.angles[3].append(data[4])
        time.sleep(self.delay)
      except IOError:
        numtries += 1
        if numtries >= 3:
          raise
          break
        continue
    self.isRunning = False
