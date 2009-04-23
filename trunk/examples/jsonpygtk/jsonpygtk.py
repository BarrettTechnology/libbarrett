#!/usr/bin/env python

# PyGTK version 2.0
import pygtk
pygtk.require('2.0')
import gtk

# Woo sockets!
import socket

class JsonClient:

   def __init__(self):
      ### Make the window
      self.window = gtk.Window()
      self.window.connect('delete_event', self.quit_request)
      self.window.connect('destroy', self.destroy)
      self.window.set_border_width(10)
      ### Make an HBox for columns
      self.cols = gtk.HBox(True,5) # homogeneous, 5-spacing
      self.window.add(self.cols)
      
      ## Make the grav column
      self.col_grav = gtk.VBox(True,5)
      self.cols.pack_start(self.col_grav)
      # Label
      self.col_grav_label = gtk.Label("Controllers")
      self.col_grav.pack_start(self.col_grav_label)
      # GComp button ON
      self.col_grav_gcomp_on = gtk.Button('Gravity Comp On')
      self.col_grav_gcomp_on.connect('clicked',self.gcomp_on)
      self.col_grav.pack_start(self.col_grav_gcomp_on)
      # GComp button OFF
      self.col_grav_gcomp_off = gtk.Button('Gravity Comp Off')
      self.col_grav_gcomp_off.connect('clicked',self.gcomp_off)
      self.col_grav.pack_start(self.col_grav_gcomp_off)
      # Idle button
      self.col_grav_idle = gtk.Button('Controller Idle')
      self.col_grav_idle.connect('clicked',self.do_idle)
      self.col_grav.pack_start(self.col_grav_idle)
      # Hold button
      self.col_grav_hold = gtk.Button('Controller Hold')
      self.col_grav_hold.connect('clicked',self.do_hold)
      self.col_grav.pack_start(self.col_grav_hold)
      
      ## Make the teach column
      self.col_teach = gtk.VBox(True,5)
      self.cols.pack_start(self.col_teach)
      # Label
      self.col_teach_label = gtk.Label("Teach & Play")
      self.col_teach.pack_start(self.col_teach_label)
      # Teach Start button
      self.col_teach_start = gtk.Button('Teach Start')
      self.col_teach_start.connect('clicked',self.teach_start)
      self.col_teach.pack_start(self.col_teach_start)
      # Teach End button
      self.col_teach_end = gtk.Button('Teach End')
      self.col_teach_end.connect('clicked',self.teach_end)
      self.col_teach.pack_start(self.col_teach_end)
      # Teach Playback button
      self.col_teach_playback = gtk.Button('Teach Playback')
      self.col_teach_playback.connect('clicked',self.teach_playback)
      self.col_teach.pack_start(self.col_teach_playback)
      
      ## Make the quit column
      self.col_quit = gtk.VBox(True,5)
      self.cols.pack_start(self.col_quit)
      # Label
      self.col_quit_label = gtk.Label("Quit")
      self.col_quit.pack_start(self.col_quit_label)
      # MoveHome button
      self.col_quit_home = gtk.Button('Move Home')
      self.col_quit_home.connect('clicked',self.move_home)
      self.col_quit.pack_start(self.col_quit_home)
      # Quit Button button
      self.col_quit_quit = gtk.Button('Quit')
      self.col_quit_quit.connect('clicked',self.quit_button)
      self.col_quit.pack_start(self.col_quit_quit)
      
      ### Show everything
      self.window.show_all()
      
      # Make a new socket
      self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.s.connect(('localhost',1338))
      
      # Attempt to open wam15
      q = '{"method":"bt_wam_create_opt","params":["wam4",0]}\n'
      print 'sending:', q
      self.s.send(q)
      rec = self.s.recv(100)
      tup = rec.split(':',1)
      tup2 = tup[1].split('}',1)
      self.wam = int(tup2[0])
      print 'received:', rec
      print 'wam number:', self.wam
   
   def go(self, q):
      print 'sending:', q
      self.s.send(q)
      print 'received:', self.s.recv(100)
   
   def quit_request(self, widget, event, data=None):
      return False
   
   def destroy(self, widget, data=None):
      self.go('{"method":"bt_wam_destroy","params":[%d]}\n' % (self.wam,))
      self.s.close()
      gtk.main_quit()
      
   def gcomp_on(self, widget, data=None):
      self.go('{"method":"bt_wam_setgcomp","params":[%d,1]}\n' % (self.wam,))
   
   def gcomp_off(self, widget, data=None):
      self.go('{"method":"bt_wam_setgcomp","params":[%d,0]}\n' % (self.wam,))
   
   def do_idle(self, widget, data=None):
      self.go('{"method":"bt_wam_idle","params":[%d]}\n' % (self.wam,))
   
   def do_hold(self, widget, data=None):
      self.go('{"method":"bt_wam_hold","params":[%d]}\n' % (self.wam,))
   
   def teach_start(self, widget, data=None):
      self.go('{"method":"bt_wam_teach_start","params":[%d]}\n' % (self.wam,))
   
   def teach_end(self, widget, data=None):
      self.go('{"method":"bt_wam_teach_end","params":[%d]}\n' % (self.wam,))
   
   def teach_playback(self, widget, data=None):
      self.go('{"method":"bt_wam_playback","params":[%d]}\n' % (self.wam,))
   
   def move_home(self, widget, data=None):
      self.go('{"method":"bt_wam_movehome","params":[%d]}\n' % (self.wam,))
      
   def quit_button(self, widget, data=None):
      if not self.quit_request(self.window,None):
         self.destroy(self.window)
   
   def main(self):
      gtk.main()
   
if __name__ == '__main__':
   client = JsonClient()
   client.main()
