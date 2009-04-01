#!/usr/bin/env python

# PyGTK version 2.0
import pygtk
pygtk.require('2.0')
import gtk

from libbt import Wam, WamList, DiscoverClient

class WamDemo:

   def __init__(self):
   
      # Make the root window w/ mbvbox
      self.w = gtk.Window()
      #self.w.set_border_width(10)
      self.w.set_title("WAM Demo")
      self.w.resize(200,100)
      self.w.connect('delete_event', self.destroy)
      self.w.connect('destroy', self.destroy)
      self.mbvbox = gtk.VBox()
      self.w.add(self.mbvbox)
      
      # Make the menubar
      self.mb = gtk.MenuBar()
      self.mbvbox.pack_start(self.mb,False,False,0)
      
      # Make the WAM menu
      self.m_wam = gtk.Menu()
      self.m_wamitem = gtk.MenuItem("WAM")
      self.m_wamitem.set_submenu(self.m_wam)
      self.mb.append(self.m_wamitem)
      
      self.m_wam_open = gtk.MenuItem("Open WAM ...")
      self.m_wam_open.connect_object("activate", self.click_open, None)
      self.m_wam.append(self.m_wam_open)
      
      self.m_wam_quit = gtk.MenuItem("Quit")
      self.m_wam_quit.connect_object("activate", self.destroy, None)
      self.m_wam.append(self.m_wam_quit)
      
      # Make the Help menu
      self.m_help = gtk.Menu()
      self.m_helpitem = gtk.MenuItem("Help")
      self.m_helpitem.set_submenu(self.m_help)
      self.mb.append(self.m_helpitem)
      
      self.m_help_about = gtk.MenuItem("About")
      self.m_help.append(self.m_help_about)
      
      # Add a label
      self.mbvbox.pack_start(gtk.Label('The WAM Demo Program'))
      
      # Show the window
      self.w.show_all()
      
      # Do we have an open dialog open?
      self.w_open = None
      
      # List of open WAMs
      self.w_wams = []
   
   def click_open(self, widget, data=None):
      if self.w_open:
         pass
      else:
         self.w_open = WamDemoOpen(self)
   
   def click_close(self):
      if self.w_open:
         self.w_open = None
   
   def wam_register(self,wam):
      if self.w_open:
         self.w_open.destroy()
         self.w_open = None
      self.w_wams.append(wam)
   
   def wam_unregister(self,wam):
      self.w_wams.remove(wam)
      
   def destroy(self, widget, data=None):
      gtk.main_quit()
      
   def main(self):
      gtk.main()

class WamDemoOpen:

   def __init__(self, wamdemo):
      
      # Save a pointer to the master window
      self.wamdemo = wamdemo
      
      # Make a new window
      self.w = gtk.Window()
      self.w.set_title("Open WAM")
      self.w.resize(400,200)
      self.w.connect('delete_event', lambda a,b: False)
      self.w.connect('destroy', self.click_destroy)
      self.wv = gtk.VBox()
      self.w.add(self.wv)
      
      # Pack the top row (Enter WAM location:)
      self.wv_toph = gtk.HBox()
      self.wv.pack_start(self.wv_toph,False,False,0)
      self.wv_toph_label = gtk.Label('WAM Location:')
      self.wv_toph.pack_start(self.wv_toph_label,False,False,0)
      self.wv_toph_wamloc = gtk.Entry()
      self.wv_toph.pack_start(self.wv_toph_wamloc)
      self.wv_toph_button = gtk.Button(label='Open')
      self.wv_toph.pack_end(self.wv_toph_button,False,False,0)
      self.wv_toph_button.connect('clicked',self.open_click,'')
      
      # Pack the bottom row
      self.wv_both = gtk.HBox()
      self.wv.pack_start(self.wv_both)
      self.wv_both_button = gtk.Button(label='Refresh')
      self.wv_both_button.connect("clicked",self.refresh,"")
      self.wv_both.pack_start(self.wv_both_button,False,False,0)
      
      # Make the empty treestore, treeview
      self.ts = gtk.TreeStore(str)
      self.ts_list = []
      
      self.tv = gtk.TreeView(self.ts)
      self.tvcolumn = gtk.TreeViewColumn('WAMs Found')
      self.tv.append_column(self.tvcolumn)
      self.tvcell = gtk.CellRendererText()
      self.tvcolumn.pack_start(self.tvcell, True) # Expand ??
      self.tvcolumn.add_attribute(self.tvcell, 'text', 0)
      self.tv.set_search_column(0)
      self.tvcolumn.set_sort_column_id(0)
      
      # Signals
      self.tv.connect('row-activated', self.tv_select, '')
      
      self.wv_both.pack_start(self.tv)
      
      # Show the window
      self.w.show_all()
   
   def destroy(self):
      self.w.destroy()
      self.wamdemo.click_close()
   
   def click_destroy(self, widget, data=None):
      self.destroy()
      
   def refresh(self, widget, data=None):
   
      # Clear the treestore
      self.ts.clear()
      self.ts_list = []
      
      # Discover any PCs out there
      d = DiscoverClient()
      d.discover()
      for i in range(d.get_num()):
         ipstr = d.get_ip(i)
         parent = self.ts.append(None, ['Host: ' + ipstr])
         parent_list = []
         # Get any WAMs on that PC
         l = WamList('tcp+json://' + ipstr)
         for j in range(l.get_num()):
            name = l.get_name(j)
            self.ts.append(parent, ['WAM: ' + name])
            parent_list.append('tcp+json://' + ipstr + '/' + name)
         del(l)
         self.ts_list.append(parent_list)
      del(d)
   
   def tv_select(self, tv, titer, path, data=None):
      if (len(titer) != 2):
         return
      self.wv_toph_wamloc.set_text(self.ts_list[titer[0]][titer[1]])
   
   def open_click(self, widget, data=None):
      wam = WamInstance(self.wamdemo,self.wv_toph_wamloc.get_text())

class WamInstance:

   def __init__(self,wamdemo,wamloc):
   
      # Save the wamdemo
      self.wamdemo = wamdemo
   
      ### Make the window
      self.window = gtk.Window()
      self.window.connect('delete_event', lambda a,b: False)
      self.window.connect('destroy', self.click_destroy)
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
      self.col_grav_gcomp_on.connect('clicked', lambda wid: self.wam.setgcomp(1))
      self.col_grav.pack_start(self.col_grav_gcomp_on)
      # GComp button OFF
      self.col_grav_gcomp_off = gtk.Button('Gravity Comp Off')
      self.col_grav_gcomp_off.connect('clicked', lambda wid: self.wam.setgcomp(0))
      self.col_grav.pack_start(self.col_grav_gcomp_off)
      # Idle button
      self.col_grav_idle = gtk.Button('Controller Idle')
      self.col_grav_idle.connect('clicked', lambda wid: self.wam.idle())
      self.col_grav.pack_start(self.col_grav_idle)
      # Hold button
      self.col_grav_hold = gtk.Button('Controller Hold')
      self.col_grav_hold.connect('clicked', lambda wid: self.wam.hold())
      self.col_grav.pack_start(self.col_grav_hold)
      
      ## Make the teach column
      self.col_teach = gtk.VBox(True,5)
      self.cols.pack_start(self.col_teach)
      # Label
      self.col_teach_label = gtk.Label("Teach & Play")
      self.col_teach.pack_start(self.col_teach_label)
      # Teach Start button
      self.col_teach_start = gtk.Button('Teach Start')
      self.col_teach_start.connect('clicked', lambda wid: self.wam.teach_start())
      self.col_teach.pack_start(self.col_teach_start)
      # Teach End button
      self.col_teach_end = gtk.Button('Teach End')
      self.col_teach_end.connect('clicked', lambda wid: self.wam.teach_end())
      self.col_teach.pack_start(self.col_teach_end)
      # Teach Playback button
      self.col_teach_playback = gtk.Button('Teach Playback')
      self.col_teach_playback.connect('clicked', lambda wid: self.wam.playback())
      self.col_teach.pack_start(self.col_teach_playback)
      
      ## Make the quit column
      self.col_quit = gtk.VBox(True,5)
      self.cols.pack_start(self.col_quit)
      # Label
      self.col_quit_label = gtk.Label("Quit")
      self.col_quit.pack_start(self.col_quit_label)
      # MoveHome button
      self.col_quit_home = gtk.Button('Move Home')
      self.col_quit_home.connect('clicked', lambda wid: self.wam.movehome())
      self.col_quit.pack_start(self.col_quit_home)
      # Quit Button button
      self.col_quit_quit = gtk.Button('Close')
      self.col_quit_quit.connect('clicked',self.click_destroy)
      self.col_quit.pack_start(self.col_quit_quit)
      
      ### Show everything
      self.window.show_all()
      
      # Attempt to open wam15
      self.wam = Wam(wamloc)
      
      # Register myself
      self.wamdemo.wam_register(self)
   
   def destroy(self):
      if self.wam:
         # Remove myself
         del(self.wam)
         self.wam = None
         self.wamdemo.wam_unregister(self)
         self.window.destroy()
   
   def click_destroy(self, widget, data=None):
      self.destroy()

      
   
if __name__ == '__main__':
   demo = WamDemo()
   demo.main()


