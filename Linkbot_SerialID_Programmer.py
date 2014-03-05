#!/usr/bin/env python

#from gi.repository import Gtk
import pygtk
import gtk as Gtk
import glib
import time

import os
from os.path import join

import barobo
from barobo import Linkbot
import serial

versioninfo = ''
if os.name != 'nt':
  logfile = os.path.join(os.environ['HOME'], 'linkbot_serialID_logfile.csv')
else:
  logfile = None

try:
  import _winreg as winreg
  import itertools
except:
  pass

def enumerate_serial_ports():
  if os.name == 'nt':
    """ Uses the Win32 registry to return an
        iterator of serial (COM) ports
        existing on this computer.
    """
    path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
    key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    except WindowsError:
        raise IterationError

    for i in itertools.count():
        try:
            val = winreg.EnumValue(key, i)
            yield '\\\\.\\' + str(val[1])
        except EnvironmentError:
            break

def _getSerialPorts():
  if os.name == 'nt':
    available = []
    for i in range(256):
      try:
        s = serial.Serial(i)
        available.append('\\\\.\\COM'+str(i+1))
        s.close()
      except serial.SerialException:
        pass
    return available
  else:
    from serial.tools import list_ports
    return [port[0] for port in list_ports.comports()]

def find_tty_usb(idVendor, idProduct):
    """find_tty_usb('067b', '2302') -> '/dev/ttyACM0'"""
    # Note: if searching for a lot of pairs, it would be much faster to search
    # for the enitre lot at once instead of going over all the usb devices
    # each time.
    for dnbase in os.listdir('/sys/bus/usb/devices'):
        dn = join('/sys/bus/usb/devices', dnbase)
        if not os.path.exists(join(dn, 'idVendor')):
            continue
        idv = open(join(dn, 'idVendor')).read().strip()
        if idv != idVendor:
            continue
        idp = open(join(dn, 'idProduct')).read().strip()
        if idp != idProduct:
            continue
        for subdir in os.listdir(dn):
            if subdir.startswith(dnbase+':'):
                for subsubdir in os.listdir(join(dn, subdir)+"/tty"):
                    if subsubdir.startswith('ttyACM'):
                        return join('/dev', subsubdir)

class Handler:
  def __init__(self, gtkbuilder):
    self.builder = gtkbuilder
    self.liststore = gtkbuilder.get_object("liststore_comports")
    self.combobox = gtkbuilder.get_object("combobox1")
    self.__updateComPorts()
    glib.timeout_add(500, self.__updateComPorts)
    self.ctx = barobo.BaroboCtx()

  def button_apply_clicked_cb(self, *args):
    self.__programID()
    entry = self.builder.get_object("entry1")
    entry.set_text('')

  def button_cancel_clicked_cb(self, *args):
    Gtk.main_quit(*args)
    pass

  def button_ok_clicked_cb(self, *angs):
    self.__programID()
    Gtk.main_quit(*args)
    pass

  def button_clear_clicked_cb(self, *args):
    self.builder.get_object("entry1").set_text('')

  def gtk_main_quit(self, *args):
    Gtk.main_quit(*args)

  def entry1_activate_cb(self, *args):
    self.button_apply_clicked_cb(*args)

  def button_getid_clicked_cb(self, *args):
    print 'Connecting to {}...'.format(self.combobox.get_child().get_text())
    self.ctx.connectDongleSFP(self.combobox.get_child().get_text())
    linkbot = self.ctx.getLinkbot()
    myid = linkbot.getSerialID()
    self.ctx.disconnect()
    d = Gtk.MessageDialog(type = Gtk.MESSAGE_ERROR, flags=Gtk.DIALOG_MODAL, buttons = Gtk.BUTTONS_CLOSE)
    d.set_markup('Linkbot id is {}'.format(myid))
    d.run()
    d.destroy()


  def __programID(self):
    entry = self.builder.get_object("entry1")
    text = entry.get_text()
    if len(text) != 4:
      self.__errorDialog("The Serial ID must be 4 characters long.")
      return
    # Connect to a linkbot
    try:
      """
      ttydev = find_tty_usb('03eb', '204b')
      if ttydev is None:
        self.__errorDialog("No Linkbot detected. Please turn on and connect a Linkbot.")
        return
      """
      print 'Connecting to {}...'.format(self.combobox.get_child().get_text())
      self.ctx.connectDongleSFP(self.combobox.get_child().get_text())
      linkbot = self.ctx.getLinkbot()
      time.sleep(0.2)
      linkbot.setLEDColor(0, 255, 0)
      time.sleep(0.1)
      linkbot._setSerialID(text.upper())
      time.sleep(0.1)
      if linkbot.getSerialID() != text.upper():
        self.ctx.disconnect()
        raise Exception('Error programming serial ID')
      linkbot.setBuzzerFrequency(440)
      time.sleep(0.5)
      linkbot.setBuzzerFrequency(0)
      self.ctx.disconnect()
      if logfile:
        try:
          f = open(logfile, 'a')
        except:
          f = open(logfile, 'w')
        f.write('{0}, {1}, {2}{3:02}{4:02}{5:02}{6:02}\n'.format(
              text.upper(), 
              versioninfo, 
              time.gmtime().tm_year,
              time.gmtime().tm_mon,
              time.gmtime().tm_mday,
              time.gmtime().tm_hour,
              time.gmtime().tm_min))
        f.close()
    except Exception as e:
      self.__errorDialog(str(e))

  def __errorDialog(self, text):
    d = Gtk.MessageDialog(type = Gtk.MESSAGE_ERROR, flags=Gtk.DIALOG_MODAL, buttons = Gtk.BUTTONS_CLOSE)
    d.set_markup(text)
    d.run()
    d.destroy()

  def __updateComPorts(self):
    if os.name == 'nt':
      ports = enumerate_serial_ports()
    else:
      ports = _getSerialPorts()
    self.liststore.clear()
    for p in sorted(ports):
      self.liststore.append([p])
    return True

#Get the current version string 
f = open('versioninfo.txt', 'r')
versioninfo = f.readline().rstrip()
f.close()
builder = Gtk.Builder()
builder.add_from_file("interface.glade")
sighandler = Handler(builder)
builder.connect_signals(sighandler)
#entry = builder.get_object("entry1")
#entry.set_editable(True)
window = builder.get_object("dialog1")
window.show_all()
#print "Main!"
Gtk.main()
