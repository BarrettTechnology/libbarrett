#!/usr/bin/python
## @package proficio.py
#  Development GUI for Proficio Applications
#
#  Main GUI to Launch all Applications that work with the Proficio Arm(s)

import wx

class UserInterFace(wx.Frame):
    
    def __init__(self, *args, **kwargs):
        super(UserInterFace, self).__init__(*args, **kwargs)
        
        self.InitUI()
        
    def InitUI(self):
        """ Initializing All Buttons on Tool Bar"""
        toolbar = self.CreateToolBar()
        ptool = toolbar.AddLabelTool(wx.ID_ANY, 'Patient',  wx.Bitmap('images/patient.png'))
        toolbar.AddSeparator()
        gtool = toolbar.AddLabelTool(wx.ID_ANY, 'Gravity',  wx.Bitmap('images/gravity.png'))
        btool = toolbar.AddLabelTool(wx.ID_ANY, 'Box',      wx.Bitmap('images/box.png'))
        stool = toolbar.AddLabelTool(wx.ID_ANY, 'Simon',    wx.Bitmap('images/simon.png'))
        wtool = toolbar.AddLabelTool(wx.ID_ANY, 'World',    wx.Bitmap('images/world.png'))
        qtool = toolbar.AddLabelTool(wx.ID_ANY, 'Quit',     wx.Bitmap('images/texit.png'))
        toolbar.Realize()
        
        """ Bind Each button to Event Function"""
        """
        self.Bind(wx.EVT_TOOL, self.Gravity,    ptool)
        self.Bind(wx.EVT_TOOL, self.Gravity,    gtool)
        self.Bind(wx.EVT_TOOL, self.BuildBox,   btool)
        self.Bind(wx.EVT_TOOL, self.Simon,      stool)
        self.Bind(wx.EVT_TOOL, self.RehabWorld, wtool)
        """
        self.Bind(wx.EVT_TOOL, self.OnQuit, qtool)
        
        """ Set Up Title and Window Sizing """
        self.SetSize((800,500))
        #self.SetSize(wx.DisplaySize()) 
        self.SetTitle('Proficio Applications')
        self.Centre()
        self.Show(True)
    
    """ Button Function Definitions """    
    def OnQuit(self, e):
        self.Close()
        
    """
    def PatientInfo()
    def GravitySet()
    def BoxBuilder()
    def SimonGame()
    def RehabWorld()
    
    """

def main():
    
    uif = wx.App()
    UserInterFace(None)
    uif.MainLoop()    


if __name__ == '__main__':
    main()