#!/usr/bin/env python

#from gi.repository import Gtk
import pygtk
import gtk as Gtk

import os
from os.path import join

from barobo.linkbot import *

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

  def button_apply_clicked_cb(self, *args):
    self.__programID()

  def button_cancel_clicked_cb(self, *args):
    Gtk.main_quit(*args)
    pass

  def button_ok_clicked_cb(self, *angs):
    self.__programID()
    Gtk.main_quit(*args)
    pass

  def gtk_main_quit(self, *args):
    Gtk.main_quit(*args)

  def __programID(self):
    entry = self.builder.get_object("entry1")
    text = entry.get_text()
    if len(text) != 4:
      self.__errorDialog("The Serial ID must be 4 characters long.")
      return
    # Connect to a linkbot
    try:
      ttydev = find_tty_usb('03eb', '204b')
      if ttydev is None:
        self.__errorDialog("No Linkbot detected. Please turn on and connect a Linkbot.")
        return
      linkbot = Linkbot()
      linkbot.connectWithTTY(ttydev)
      linkbot._setID(text.upper())
      linkbot.setBuzzerFrequency(440)
      time.sleep(0.5)
      linkbot.setBuzzerFrequency(0)
      linkbot.disconnect()
    except Exception as e:
      self.__errorDialog(str(e))

  def __errorDialog(self, text):
    d = Gtk.MessageDialog(type = Gtk.MESSAGE_ERROR, flags=Gtk.DIALOG_MODAL, buttons = Gtk.BUTTONS_CLOSE)
    d.set_markup(text)
    d.run()
    d.destroy()

builder = Gtk.Builder()
builder.add_from_file("interface.glade")
sighandler = Handler(builder)
builder.connect_signals(sighandler)
#entry = builder.get_object("entry1")
#entry.set_editable(True)
window = builder.get_object("dialog1")
window.show_all()
print "Main!"
Gtk.main()
