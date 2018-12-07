# -*- coding: utf-8 -*-
'''
Created on Thursday 06.12.2018
Copyright (©) Henricus N. Basien
Author: Henricus N. Basien
Email: Henricus@Basien.de
'''

#****************************************************************************************************
# Imports
#****************************************************************************************************

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# External
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#--- System ---
import os#,sys
from copy import copy
from collections import OrderedDict
#--- Timing ---
from time import time as getTime
from time import sleep
#--- Mathematics ---
import numpy as np

#--- User Input ---
import pygame as pg

#--- Visualization ---
try:
    from visual import frame,arrow,label
    Imported_Visual = True
except:
    print "WARNING: Unable to import Visual!"
    Imported_Visual = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Internal
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from VectorOperations import CoordinateSystem,getCS

lbf_to_N = 4.448222
g = 9.81

def Print(text):
    print text

#****************************************************************************************************
# F16Sim
#****************************************************************************************************

NrStates = 18

class F16Sim(object):
    """docstring for F16Sim"""

    #================================================================================
    # Initialize
    #================================================================================
    
    def __init__(self,pos=np.zeros(3),att=np.zeros(3),vel=np.zeros(3),rps=np.zeros(3),HighFidelity=True,Controlled=True,Visualize=False):
        super(F16Sim, self).__init__()

        self.pos = np.array(pos).astype(float) # [m]
        self.att = np.array(att).astype(float) # [rad]
        self.vel = np.array(vel).astype(float) # [m/s]
        self.rps = np.array(rps).astype(float) # [rad/s]
        self.acc = np.zeros(3)
        self.HighFidelity = HighFidelity

        self.Controlled = Controlled

        self.Init_State()
        self.Init_Time()
        self.Init_Control()

        self.Visualize = Visualize
        if self.Visualize:
            self.Create3DModel()

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # State
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def Init_State(self):

        self.xu   = np.zeros(NrStates)
        self.xdot = np.zeros(NrStates)

        for i in range(3):
            self.xu[0+i] = self.pos[i]
            self.xu[3+i] = self.att[i]
            self.xu[6+i] = self.vel[i]
            self.xu[9+i] = self.rps[i]

        #--- Fix East-West ---
        self.xu[1]*=-1

        #--- Thrust ---
        self.xu[12] = 20000 # [N]\

        #--- Fidelity ---
        self.xu[17] = int(self.HighFidelity)

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Time
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def Init_Time(self):

        self.RunNr = 0
        self.t0 = getTime() # [s]
        self.t  = 0.0 # [s] 
        self.dt = 0.0 # [s]

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Control
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def Init_Control(self):

        pg.init()
        sleep(1.0)
        pg.joystick.init()
        sleep(1.0)

        self.Joysticks = OrderedDict()
        for i in range(pg.joystick.get_count()):
            js = pg.joystick.Joystick(i)
            self.Joysticks[js.get_name().strip()] = js

        for name in self.Joysticks:
            joystick = self.Joysticks[name]
            joystick.init()
            print "'"+name+"'",joystick

        self.Yoke   = self.Joysticks["Saitek Pro Flight Yoke"]
        self.Pedals = self.Joysticks["CH PRO PEDALS USB"]

        for i in range(1):#(100):
            self.PrintAllControls()
            sleep(0.2)

        #----------------------------------------
        # Extremes
        #----------------------------------------

        self.Control_ext = OrderedDict()
        self.Control_ext["Thrust"  ]   = [1000*lbf_to_N,19000*lbf_to_N]
        self.Control_ext["Elevator"] = [-25.0,25.0]
        self.Control_ext["Ailerons"] = [-21.5,21.5]
        self.Control_ext["Rudder"  ]   = [-30.0,30.0]

        #----------------------------------------
        # Trim
        #----------------------------------------
        
        self.Controls_trim = OrderedDict()
        for name in self.Control_ext:
            self.Controls_trim[name] = 0.0

    def PrintAllControls(self):
        pg.event.get() #pg.event.pump() 
        for js in [self.Yoke,self.Pedals]:
            print js.get_name(),js.get_init()
            for i in range(js.get_numaxes()):
                print "Axis#",i,js.get_axis(i)
            for i in range(js.get_numbuttons()):
                print "Button#",i,js.get_button(i)

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # 3D Model
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      
    def Create3DModel(self,ShowInfo=True,opacity=0.9):#False):#True):
        
        self.Frame = frame(pos=self.pos,make_trail=True,trail_type="points")

        self.CS = CoordinateSystem(pos=self.pos,att=self.att,RotationSigns=[1,-1,-1],name="F16 CS")
        for i in range(3):
            if i==0: l = 15.0/2.
            else:    l = 9.96/2.
            axis = np.identity(3)[i]*l # self.CS.matrix[:,i]*l
            arrow(axis=axis,color=np.identity(3)[i],opacity=opacity,frame=self.Frame)#,make_trail=True)

        self.rps_vector = arrow(axis=[0,0,-1*10**(-10)],color=[1,0,0],opacity=opacity,frame=self.Frame)
        self.acc_vector = arrow(axis=[0,0,-1*10**(-10)],color=[1,1,0],opacity=opacity,frame=self.Frame)

        self.ShowInfo = ShowInfo
        if self.ShowInfo:
            self.InfoLabel = label(text="F16",frame=self.Frame,xoffset=int(1920*0.1),yoffset=int(1080*0.1))

    #================================================================================
    # Run
    #================================================================================

    def Run(self):

        while True:
            self.Update()

    #================================================================================
    # Update
    #================================================================================
    
    def Update(self,PrintData=False):#True):

        self.Update_Time()
        self.Update_Controls()
        self.CallF16Sim()
        self.ExtractOutput()
        self.IntegrateState()
        self.ConvertOutput()
        if PrintData:
            self.PrintState()
        if self.Visualize:
            self.Update_3DModel()

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Time
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def Update_Time(self):

        self.RunNr+=1
        self.dt = (getTime()-self.t0)-self.t
        self.t  =  getTime()-self.t0

        InfoText = "Run#"+str(self.RunNr)+": t="+str(round(self.t,2))+" s | dt="+str(round(self.dt*1000,2))+" ms"
        if self.dt!=0:
            InfoText+=" | FPS="+str(round(1./self.dt,2))+" Hz"
        print InfoText

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Controls
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def Update_Controls(self):

        #--- Get Input ---
        pg.event.get()  

        #----------------------------------------
        # Percentages
        #----------------------------------------
        
        self.Controls_per = OrderedDict()
        self.Controls     = OrderedDict()
        self.Controls_per["Thrust"  ] = (-self.Yoke.get_axis(2)+1)/2
        self.Controls_per["Elevator"] = -self.Yoke.get_axis(1)       
        self.Controls_per["Ailerons"] = -self.Yoke.get_axis(0)       
        self.Controls_per["Rudder"  ] = -self.Pedals.get_axis(2)       

        #----------------------------------------
        # Trim
        #----------------------------------------        

        v_trim = 5#10 # [%/s]
        d_trim = v_trim/100.*self.dt

        if self.Yoke.get_button(4):
            self.Controls_trim["Elevator"]+=d_trim
        if self.Yoke.get_button(5):
            self.Controls_trim["Elevator"]-=d_trim

        if self.Yoke.get_button(6):
            self.Controls_trim["Ailerons"]+=d_trim
        if self.Yoke.get_button(7):
            self.Controls_trim["Ailerons"]-=d_trim

        #----------------------------------------
        # Control
        #----------------------------------------
        
        for name in self.Controls_per:
            self.Controls_per[name] += self.Controls_trim[name]

            if   self.Controls_per[name]> 1.0:
                self.Controls_per[name] = 1.0
            elif self.Controls_per[name]<-1.0:
                self.Controls_per[name] =-1.0

            per = self.Controls_per[name]
            if name!="Thrust":
                per = (per+1)/2
            self.Controls[name] = self.Control_ext[name][0] + per * (self.Control_ext[name][1]-self.Control_ext[name][0])
            if   self.Controls[name]>self.Control_ext[name][1]:
                self.Controls[name] = copy(self.Control_ext[name][1])
            elif self.Controls[name]<self.Control_ext[name][0]:
                self.Controls[name] = copy(self.Control_ext[name][0])

        #----------------------------------------
        # Set State
        #----------------------------------------
        
        self.xu[12] = self.Controls["Thrust"  ]   
        self.xu[13] = self.Controls["Elevator"]   
        self.xu[14] = self.Controls["Ailerons"] 
        self.xu[15] = self.Controls["Rudder"  ]   

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Call F16Sim
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def CallF16Sim(self):   

        parameters = ""
        for x in self.xu:
            parameters+=" "+str(x)

        cmd = "./F16Sim"
        if os.name=="nt":
            #cmd = 'bash -c "'+cmd+'"'
            cmd = "F16Sim.exe"

        cmd+=" "+parameters
        os.system(cmd)

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Extract
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def ExtractOutput(self):
        
        with open("F16Sim_output.csv","r") as OutputFile:
            lines = OutputFile.readlines()

            self.xu   = [float(v.strip()) for v in filter(None,lines[0].split(",")) if v.strip()!=""]
            self.xdot = [float(v.strip()) for v in filter(None,lines[1].split(",")) if v.strip()!=""]

    #----------------------------------------
    # Convert
    #----------------------------------------
    
    def ConvertOutput(self):

        for i in range(3):
            self.pos[i] = self.xu[0+i]
            self.att[i] = self.xu[3+i]
            self.vel[i] = self.xu[6+i]
            self.rps[i] = self.xu[9+i]

            self.acc[i] = self.xdot[12+i]*g

        #--- Fix East-West ---
        self.pos[1]*=-1
        #--- Coordinate System ---
        self.CS.setCS(self.att)

        #--- ToDoConvert Velocity! ---
        #self.vel = ...based on AoA and AoS

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Integrate
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def IntegrateState(self):

        for i in range(12):
            self.xu[i]+=self.dt*self.xdot[i]

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Print
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def PrintState(self):

        print self.GetInfoText()

        print "xu:  ",self.xu
        print "xdot:",self.xdot

    def GetInfoText(self,nd=3):

        InfoText = ""
        InfoText+="pos: "+str([str(np.round(           v ,nd)) for v in self.pos])+" [m]"    +"\n"
        #InfoText+="vel: "+str([str(np.round(           v ,nd)) for v in self.vel])+"[m/s]"  +"\n"
        InfoText+="vel: "+str(np.round(self.vel[0],nd))+" [m/s]"+" | AoA/AoS "+str(np.round(np.degrees(self.vel[1])))+"/"+str(np.round(np.degrees(self.vel[2])))+" [deg]"+"\n"
        InfoText+="att: "+str([str(np.round(np.degrees(v),nd)) for v in self.att])+" [deg]"  +"\n"
        InfoText+="rps: "+str([str(np.round(np.degrees(v),nd)) for v in self.rps])+" [deg/s]"+"\n"

        InfoText+="\n"
        InfoText+="Thrust:   "+str(round(self.Controls["Thrust"  ],nd))+" [N]  "+" ("+str(round(self.Controls_per["Thrust"  ]*100,nd))+"%)"+"\n" 
        InfoText+="Elevator: "+str(round(self.Controls["Elevator"],nd))+" [deg]"+" ("+str(round(self.Controls_per["Elevator"]*100,nd))+"%)"+"\n" 
        InfoText+="Ailerons: "+str(round(self.Controls["Ailerons"],nd))+" [deg]"+" ("+str(round(self.Controls_per["Ailerons"]*100,nd))+"%)"+"\n" 
        InfoText+="Rudder:   "+str(round(self.Controls["Rudder"  ],nd))+" [deg]"+" ("+str(round(self.Controls_per["Rudder"  ]*100,nd))+"%)"+"\n" 

        return InfoText

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Update 3D Model
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    def Update_3DModel(self):

        self.Frame.pos = self.pos
        self.Frame.axis = copy(self.CS.matrix[:,0])
        self.Frame.up   = copy(self.CS.matrix[:,2])

        self.rps_vector.axis = self.rps
        self.acc_vector.axis = self.acc
        
        if self.ShowInfo:
            self.InfoLabel.text = "F16:\n"+self.GetInfoText()

#****************************************************************************************************
# Test Code
#****************************************************************************************************

if __name__=="__main__":
    
    #================================================================================
    # Settings
    #================================================================================
    
    #--- Simulation ---
    pos = [0,0,1000]
    vel = [100,np.radians(7),0] # [Vt,AoA,AoS]
    att = [0  ,np.radians(4),0]
    rps = np.zeros(3)
    HighFidelity = True

    Controlled = True

    Visualize = True#False#True
    if not Imported_Visual:
        Visualize = False

    MaxSimTime = 60#20#3#10 # [s]

    #--- Visualization ---
    FPV = False#True#False#True#False
    Resolution = [1920,1080]
    MaxFPS = 500 # [FPS]
    stereo = "crosseyed" # None

    #================================================================================
    # Initialize
    #================================================================================
    
    if Visualize:
        from visual import display,rate,arrow
        window = display(title="F16 Simulation",width=Resolution[0],height=Resolution[1])

        window.up      = np.identity(3)[2]
        window.forward = -np.ones(3)
        window.center  = pos

        if FPV:
            window.range = 15*2

        window.stereo = stereo

        #--- CS ---
        for i in range(3):
            arrow(axis=np.identity(3)[i]*1000,color=np.identity(3)[i],opacity=0.75)

    #================================================================================
    # Run Simulation
    #================================================================================
    
    sim = F16Sim(pos=pos,att=att,vel=vel,rps=rps,HighFidelity=HighFidelity,Controlled=Controlled,Visualize=Visualize)

    while True:
        #--- Update Simulation ---
        sim.Update()
        #--- Update Visualization ---
        if Visualize:
            window.center  = sim.pos
            if FPV:
                window.forward = sim.CS.matrix[:,0]
                window.up      = -sim.CS.matrix[:,1]
            rate(MaxFPS)
        #--- Terminate Simulation ---
        if sim.t>MaxSimTime:
            break