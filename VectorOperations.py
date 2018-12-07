# -*- coding: utf-8 -*-
"""
Created on Saturday 20.09.2014
Development plattform: Spyder

Script:              VectorOperations.py
Description:         Defines vector manipulating functions and linear algebra modulation.

Project title:       Compact Copters - Flight Simulator
Project version:     1.0
Author:              Henricus N. Basien
Author E-Mail:       Henricus@Basien.de
Copyright:           Copyright (c) 2014 CompactCopters
Description:         Global Drone-Swarm Flightsimulator

"""

#****************************************************************************************************
# Imports
#****************************************************************************************************

import sys,os;RootPath = os.path.realpath(__file__)
while True:
    try:    from Importer import ImportScripts;ImportScripts();break        
    except: RootPath = os.path.split(RootPath)[0]; sys.path.insert(0,RootPath); print(RootPath)
    if RootPath.strip()=="" or RootPath.strip()[1:]==":\\":
        print("Error while trying to Import Importer. RootPath: ",RootPath);break

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# External
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from math import isnan #sin,cos,sqrt,radians,pi,isnan

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Internal
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

try:
    from Settings import NorthVector,StandardWorldCS,StandardCopterCS,CopterRotationSigns,RotationOrder
except:
    # Coordinate System Rotation
    RotationOrder  = np.array([2,1,0]) # 0=x,1=y,z=2 || [Z,Y,X]

    # World Coordinate System
    NorthVector          = np.array([0,1,0])#np.array([-1,0,0])
    StandardWorldCS      = np.identity(3)

    # Copter Coordinate System
    StandardCopterCS = np.identity(3)
    StandardCopterCS[:,0] = np.array([0,1,0])
    StandardCopterCS[:,1] = np.array([-1,0,0])
    StandardCopterCS[:,2] = np.array([0,0,1])
    CopterRotationSigns   = np.array([1,1,-1])
    
try:
    from TTY import PrintSpecial,ArrayToString
except:
    def PrintSpecial(txt,*args,**kwargs):
        print txt
    def ArrayToString(a):
        return str(a)

try:
    from Math import signedExp
except:
    def signedExp(nr,exp):
        if nr==0. or nr==1.:
            return nr
            
        sign=nr/abs(nr)
        return sign*abs(nr)**(exp)

try:
    from SysInfo import SysInfo
    re=SysInfo.re
except:
    re = 15#16

PH = " "*3

#****************************************************************************************************
# Vector Computations
#****************************************************************************************************

#================================================================================
# Vector Properties
#================================================================================

def VectorAbs(vector):
    '''
    Verified = True
    Returns the length of the 'vector'.
    '''
    abs=0.
    for val in vector:
        abs=abs+val**2
        
    return abs**0.5#sqrt(abs)
    
def unit_vector(vector):
    '''
    Verified = True
    Returns the unit vector of the 'vector'. 
    '''
    if np.all(vector==0):
        return np.array([0.]*len(vector))
        
    uv = vector / np.linalg.norm(vector)
    return uv
    # return np.round(uv,re)#vector / np.linalg.norm(vector)
    
def AngleBetween(v1, v2):
    '''
    Verified = True
    Returns the angle in radians between the vectors 'v1' and 'v2':
    '''
    # Unify Vectors
    # v1,v2 = unit_vector(v1),unit_vector(v2)
    v1,v2 = unit_vector(np.array(v1)),unit_vector(np.array(v2))
#    v1,v2 = np.round(v1,re-8),np.round(v2,re-8) ###                            !!!!
    
    # Check extreme cases
    if np.all(v1==0) or np.all(v2==0):
        return False
    elif np.all(v1 == v2):
        return 0.0
    elif  np.all(v1 == -v2):
        return np.pi 
        
    # Calculate angle
    v=np.dot(v1, v2)
    if abs(v)>1.:
        v=v/abs(v)
    angle = np.arccos(v)
    
    # Check errorous angle
    if isnan(angle):
        angle=False        
        
    return angle

#==============================================================================
# Vector transformations
#==============================================================================

def SphericalCartesian(sphr=np.ones(3)):
    theta,phi,r=sphr[0],sphr[1],sphr[2]
    x = r*np.sin(theta)*np.cos(phi)
    y = r*np.sin(theta)*np.sin(phi)
    z = r*np.cos(theta)
    
    return np.array([x,y,z])
    
def GPSCartesian(GPS=np.zeros(3),h0=0):
    lat,lon,r=np.radians(GPS[0]),np.radians(GPS[1]),GPS[2]+h0
    x = r*np.cos(lat)*np.cos(lon)
    y = r*np.cos(lat)*np.sin(lon)
    z = r*np.sin(lat)
    
    return np.array([x,y,z])

def CartesianSpherical(cart=np.ones(3)):
    x,y,z=cart[0],cart[1],cart[2]
    r=VectorAbs(cart)
    theta = np.arccos(z/r)
    phi   = np.arctan(y/x)    
    
    return np.array([r,theta,phi])

def OrthProjection(vector,planeV1=np.identity(3)[2],planeV2=False):
    '''
    Verified = True
    Returns the Orthogonal projection of the 'vector' onto the plane defined by the vectors 'planeV1' and 'planeV2'.
    If 'planeV2' is not given 'planeV1' is assumed to be the normal vector.
    '''
    # Unify Vectors
    vector=np.array(vector)
    planeV1=np.array(planeV1)
    
    if np.all(planeV2==False):
        n=planeV1
    else:
        planeV2=np.array(planeV2)
        # Plane normal vector
        n=unit_vector(np.cross(planeV1,planeV2))
        
    return vector-np.dot(vector,n)*n

def rotation_matrix(axis,theta,matrix=False):
    '''
    Returns the rotation matrix (R) for a rotation around a given axis for a given angle (theta) - counterclockwise!
    '''
    theta=-theta # Makes the rotation counterclockwise
    axis = unit_vector(axis)
    a = np.cos(theta/2)
    b,c,d = -axis*np.sin(theta/2)
    b,c,d = float(b),float(c),float(d)

    if matrix==True:
        return np.matrix([[a*a+b*b-c*c-d*d , 2*(b*c-a*d)     , 2*(b*d+a*c)    ],
                          [2*(b*c+a*d)     , a*a+c*c-b*b-d*d , 2*(c*d-a*b)    ],
                          [2*(b*d-a*c)     , 2*(c*d+a*b)     , a*a+d*d-b*b-c*c]])
    else:
        return np.array([[a*a+b*b-c*c-d*d , 2*(b*c-a*d)     , 2*(b*d+a*c)    ],
                         [2*(b*c+a*d)     , a*a+c*c-b*b-d*d , 2*(c*d-a*b)    ],
                         [2*(b*d-a*c)     , 2*(c*d+a*b)     , a*a+d*d-b*b-c*c]])

RotationMatrices = dict()

def GetRotationKey(axis,ang):
    return ArrayToString(axis)+","+str(ang)

def EulerRotation(M,axis=[0,0,1],ang=0):
    '''
    Verified = True
    Rotates a matrix (M) of vectors around a given 'axis' for a given angle (ang) - counterclockwise!
    '''

    M = np.array(M)

    if M.shape==(2,):
        return Rotate2D(M,ang)      

    if 0:
        axis = AxisNameToVector(axis)
        
    key = GetRotationKey(axis,ang)
    if not key in RotationMatrices:
        RotationMatrices[key] = rotation_matrix(axis,ang)
        # if len(RotationMatrices)%100==0:
        #     print len(RotationMatrices)
    R = RotationMatrices[key]
    
    # print M,axis,ang,M.shape,R

    if M.shape==(3,) or M.shape==(3,3):#len(M)!=3:
        return np.dot(R,M)#.round(re)
    else:
        List=[]
        for vector in M:
            List.append(np.dot(R,vector))
        return np.array(List)

AxisNames = ['x','y','z']
def AxisNameToVector(axis):
    if type(axis)==str:
        for i in range(3):
            if axis.lower()==AxisNames[i]:
                axis = np.identity(3)[i]
    return axis

def Rotate2D(M=np.identity(2),ang=0):

    # RM = np.array([\
    # [cos(ang),-sin(ang)],\
    # [sin(ang),cos(ang)]\
    # ])
    RM = np.array([\
    [ np.cos(ang),np.sin(ang)],\
    [-np.sin(ang),np.cos(ang)]\
    ])

    return np.dot(M,RM)

#==============================================================================
# Coordinate Systems
#==============================================================================

class CoordinateSystem():
    
    def __init__(self,refCS=np.identity(3),att=[0.,0.,0.],pos=(0.,0.,0.),RotationSigns=[1,1,1],name="default Coordinate System"):
        if isinstance(refCS, CoordinateSystem):
            refCS=refCS.matrix        
        
        self.name=name        
        self.refCS=refCS
        self.pos=np.array(pos)
        self.RotationSigns=RotationSigns
        self.matrix=getCS(att=att,refCS=self.refCS,RotationSigns=self.RotationSigns)   
        self.getAttitude()
        self.check()
         
    def setCS(self,att):
        self.matrix=getCS(att=att,refCS=self.refCS,RotationSigns=self.RotationSigns)
        self.getAttitude()
        self.check()
        
    def shiftCS(self,datt):
        self.matrix=shiftCS(datt=datt,CS=self.matrix,refCS=self.refCS,RotationSigns=self.RotationSigns)
        self.getAttitude()
        self.check()
        
    def getAttitude(self):
        self.att=getAttitude(self.matrix,self.refCS,self.RotationSigns)
        return self.att
    
    def check(self):
        angxy=AngleBetween(self.matrix[:,0],self.matrix[:,1])
        angyz=AngleBetween(self.matrix[:,1],self.matrix[:,2])
        angxz=AngleBetween(self.matrix[:,0],self.matrix[:,2])
        angxyDif=abs(round(angxy-np.pi/2,re))
        angyzDif=abs(round(angyz-np.pi/2,re))
        angxzDif=abs(round(angxz-np.pi/2,re))
        
        nz=abs(np.cross(self.matrix[:,0],self.matrix[:,1]))
        nx=abs(np.cross(self.matrix[:,1],self.matrix[:,2]))
        ny=abs(np.cross(self.matrix[:,2],self.matrix[:,0]))
        if np.any(nx-abs(self.matrix[:,0])>2*10**(-re)) or np.any(ny-abs(self.matrix[:,1])>2*10**(-re)) or np.any(nz-abs(self.matrix[:,2])>2*10**(-re)):
            PrintSpecial("The Coordinate System '"+self.name+"' is inconsistent! Axis are not Perpendicular!","Red")
            
            print "\n"+"-"*100
            print "CrossProduct of X- and Y-axis: "+str(nz)+" - Z-axis: "+str(self.matrix[:,2])+" - Difference: "+str(nx-abs(self.matrix[:,0])),str(np.any(nx-abs(self.matrix[:,0])>2*10**(-re)))
            print "CrossProduct of Y- and Z-axis: "+str(nx)+" - X-axis: "+str(self.matrix[:,0])+" - Difference: "+str(ny-abs(self.matrix[:,1])),str(np.any(ny-abs(self.matrix[:,1])>2*10**(-re)))
            print "CrossProduct of Z- and X-axis: "+str(ny)+" - Y-axis: "+str(self.matrix[:,1])+" - Difference: "+str(nz-abs(self.matrix[:,2])),str(np.any(nz-abs(self.matrix[:,2])>2*10**(-re)))
            print
            print "Angle between X- and Y-axis: "+str(angxy)+" - Offset: "+str(angxyDif)
            print "Angle between Y- and Z-axis: "+str(angyz)+" - Offset: "+str(angyzDif)
            print "Angle between X- and Z-axis: "+str(angxz)+" - Offset: "+str(angxzDif)
            print "-"*100+"\n"
            #raw_input("...")
        elif angxyDif>10**(-re) or angyzDif>10**(-re) or angxzDif>10**(-re):
            PrintSpecial("The Coordinate System '"+self.name+"' is inconsistent! Angles are not 90°!","Red")
                    
            print "Angle between X- and Y-axis: "+str(angxy)+" - Offset: "+str(angxyDif)
            print "Angle between Y- and Z-axis: "+str(angyz)+" - Offset: "+str(angyzDif)
            print "Angle between X- and Z-axis: "+str(angxz)+" - Offset: "+str(angxzDif)
            #raw_input("...")

def getCS(att=np.zeros(3),refCS=np.identity(3),RotationSigns=[1,1,1]):
    '''
    Returns a Coordinate System based on the reference System 'refCS' corresponding to a given Attitude 'att'.
    In addition the Rotation directions can specified 'RotationSigns'. (1=Counterclockwise,-1=Clockwise)
    ''' 
    CS=refCS   
    for i in RotationOrder:
        CS=EulerRotation(CS,CS[:,i],att[i]*RotationSigns[i])
    return CS
    
def shiftCS(datt=np.zeros(3),CS=np.identity(3),refCS=np.identity(3),RotationSigns=[1,1,1]): 
    '''
    Returns a Coordinate System based on the reference System 'refCS' corresponding to a given change in Attitude 'datt'.
    In addition the Rotation directions can specified 'RotationSigns'. (1=Counterclockwise,-1=Clockwise)
    ''' 
    if np.all(RotationOrder==[2,1,0]):
        # Heading
        i=RotationOrder[0]
        CS=EulerRotation(CS, refCS[:,2],datt[i]*RotationSigns[i])
        # Pitch
        i=RotationOrder[1]
#        psi=AngleBetween(CS[:,1],refCS[:,1])
#        n=unit_vector(np.cross(CS[:,1],refCS[:,1]))
#        if not np.all(n==refCS[:,2]):
#            psi*=-1
#        elif np.all(n==refCS[:,2]):
#            pass
#        else:
#            print "Get Attitude, Heading Error! - ",np.all(n==-refCS[:,2]),n,-refCS[:,2]
#            raw_input("")
            
        refY=np.cross(refCS[:,2],CS[:,0])#EulerRotation(refCS[:,1],refCS[:,2],psi*RotationSigns[2])
        CS=EulerRotation(CS,refY,datt[i]*RotationSigns[i])
        # Roll
        i=RotationOrder[2]
        CS=EulerRotation(CS,CS[:,i],datt[i]*RotationSigns[i])
    else:
        PrintSpecial("This Function has not yet been implemented for the given Order of rotations! (shiftCS)","Red")
        quit()
    return CS

def getAttitude(CS,refCS=StandardWorldCS,RotationSigns=[1,1,1],PrintErrors=False):
    '''
    Returns the three Attitude variables (Heading,Pitch,Roll) (Z,Y,X)
    '''
    # Check Coordinate Systems
    if isinstance(CS, CoordinateSystem):
        CS=CS.matrix
    if isinstance(refCS, CoordinateSystem):
        refCS=refCS.matrix
        
    # Get Attitude
    att=np.array([0.]*3)
    if np.all(RotationOrder==[2,1,0]):
        # Heading
        Xproj=OrthProjection(CS[:,0],refCS[:,0],refCS[:,1])
        angle=AngleBetween(Xproj,refCS[:,0])
        
        n=unit_vector(np.cross(Xproj,refCS[:,0]))
        if np.all(abs(np.round(n+refCS[:,2]))<2*10**(-re+1)):
            angle=2*np.pi-angle
        elif np.all(abs(np.round(n-refCS[:,2]))<2*10**(-re+1)):
            pass
        elif np.all(np.round(n,re)<2*10**(-re)):
            angle=0.
            
        else:
            if PrintErrors:
                PrintSpecial("Get Attitude, Heading Error! - "+str(np.all(n==-refCS[:,2]))+"\n"+\
                PH+ "Normal Vector: "+str(n)+"\n"+\
                PH+"Reference z-axis: "+str(refCS[:,2])\
                ,"Red_H")
                #raw_input("")
        att[2]=angle
        
        # Pitch
        angle=AngleBetween(Xproj,CS[:,0])
        
        difV = unit_vector(CS[:,0]-Xproj)#np.round(unit_vector(np.round(CS[:,0]-Xproj,re)),re)
        refV = refCS[:,2]*RotationSigns[1]#np.round(refCS[:,2]*RotationSigns[1],re)
        
        vPlus  = difV+refV
        vMinus = difV-refV       
        if VectorAbs(vMinus)<2*10**(-re+2):#np.all(abs(vMinus)<2*10**(-re+2)):#np.all(difV==refV):#np.dot(CS[:,0],refCS[2])<0:#
            angle*=-1#pass#angle*=-1
        elif VectorAbs(vPlus)<2*10**(-re+2):#np.all(abs(vPlus)<2*10**(-re+2)):#np.all(difV==-refV):
            pass#angle*=-1#pass
        elif np.all(abs(np.round(CS[:,0]-Xproj,re))<2*10**(-re+2)):
            angle=0.
        else:
            if PrintErrors:
                PrintSpecial("\n"+"-"*40+"\nGet Attitude, Pitch Error! - "+str(np.all(difV==refV))+"\n"+\
                PH+"CS x-axis: "+str(CS[:,0])+"\n"+\
                PH+"CS x-axis (Proj): "+str(Xproj)+"\n"+\
                PH+"CS x-axis - Projection: "+str(difV)+"\n"+\
                PH+"Ref CS z-axis: "+str(refV)+"\n"+\
                PH+"Difference +: "+str(vPlus)+" ("+str(VectorAbs(vPlus))+")\n"+\
                PH+"Difference -: "+str(vMinus)+" ("+str(VectorAbs(vMinus))+")\n"+\
                "-"*40+"\n","Red_H")
                if VectorAbs(vPlus)>VectorAbs(vMinus):
                    angle*=-1
                    print PH*2+"Difference (-) ..."
                else:
                    print PH*2+"Difference (+) ..."     
                    
                #raw_input("")
        att[1]=angle
        
        # Roll
        r0CS=getCS(att,refCS=refCS,RotationSigns=RotationSigns)
        angle=AngleBetween(r0CS[:,1],CS[:,1])
        
#        print "<<<ROLL TEST!!!>>>"
#        [Print(str(round(np.degrees(ang),2))) for ang in att]
#        print "Angle: ",str(round(np.degrees(angle),2))
#        print "r0-y: ",r0CS[:,1],"CS-y: ",CS[:,1]
#        arrow(axis=r0CS[:,0]*2,shaftwidth=0.005,color=Colors["Orange"],opacity=0.1)
#        arrow(axis=r0CS[:,1]*2,shaftwidth=0.005,color=Colors["Orange"],opacity=0.1)
#        arrow(axis=r0CS[:,2]*2,shaftwidth=0.005,color=Colors["Orange"],opacity=0.1)
#        arrow(axis=CS[:,1]*2,shaftwidth=0.01)
#        print "<<<END ROLL TEST!!!>>>"        
        
        Yproj=OrthProjection(CS[:,1],r0CS[:,0],r0CS[:,1])#np.round(OrthProjection(CS[:,1],r0CS[:,0],r0CS[:,1]),re-1)
        vPlus  = unit_vector(CS[:,1]-Yproj)+r0CS[:,2]
        vMinus = unit_vector(CS[:,1]-Yproj)-r0CS[:,2]
        if VectorAbs(vPlus)<2*10**(-re+2):#np.all(abs(np.round(vPlus,re))<2*10**(-re+2)):#np.dot(CS[:,1],refCS[2])>0:#
            angle*=-1
        elif VectorAbs(vMinus)<2*10**(-re+2):#np.all(abs(np.round(vMinus,re))<2*10**(-re+2)):
            pass
        elif np.all(abs(np.round(CS[:,1]-Yproj,re))<2*10**(-re+2)):
            angle=0.
        else:
            if PrintErrors:
                PrintSpecial("\n"+"-"*40+"\nGet Attitude, Roll Error!"+"\n"+\
                PH+"y-axis: "+str(np.round(CS[:,1],re))+"\n"+\
                PH+"y-proj: "+str(Yproj)+"\n"+\
                PH+"y-proj (norm): "+str(unit_vector(Yproj))+"\n"+\
                PH+"y-axis (R0): "+str(r0CS[:,1])+"\n"+\
                PH+"Upward Vector: "+str(unit_vector(CS[:,1]-Yproj))+"\n"+\
                PH+"Zero Roll z-Axis: "+str(r0CS[:,2])+"\n"+\
                PH+"Difference +: "+str(vPlus)+" ("+str(VectorAbs(vPlus))+")\n"+\
                PH+"Difference -: "+str(vMinus)+" ("+str(VectorAbs(vMinus))+")\n"+\
                "-"*40+"\n","Red_H")
                if VectorAbs(vPlus)<VectorAbs(vMinus):
                    angle*=-1
                    print PH*2+"Difference (+) ..."
                else:
                    print PH*2+"Difference (-) ..."
                
        att[0]=angle

#        print "\nHeading: ",n.round(re),-refCS[:,2].round(re),n.round(re)==-refCS[:,2].round(re)
#        print n[2],-refCS[:,2][2],n[2].round(re)==-refCS[:,2][2].round(re),n[2].round(re)+refCS[:,2][2].round(re)
#        print npCS[:,0].cross(Xproj,refCS[:,0]),Xproj,refCS[:,0]
#        print "Pitch: ",unit_vector(CS[:,0]-Xproj).round(re),refCS[:,2].round(re),unit_vector(CS[:,0]-Xproj).round(re)==refCS[:,2].round(re)
#        print "Roll: ",unit_vector(CS[:,1]-Yproj).round(re),r0CS[:,2].round(re),unit_vector(CS[:,1]-Yproj).round(re)==-r0CS[:,2].round(re)
#        print "Attitude: ",att
#        print ("-"*40+"\n")*2
#     or isinstance(M1, CoordinateSystem3D)
#        att=[0.]*3self.
#        # Heading
#        angle,n=AngleBetween(OrthProjection(CS[:,0],[1,0,0],[0,1,0]),NorthVector),np.cross(OrthProjection(CS[:,0],[1,0,0],[0,1,0]),NorthVector)
#        if np.all(unit_vector(n)==[0,0,-1]):
#            angle=2*np.pi-angle
#        att[2]=angle
#        # Pitch
#        angle=AngleBetween(OrthProjection(CS[:,0],[1,0,0],[0,1,0]),CS[:,0])
#        if CS[:,0][2]<0:
#            angle*=-1
#        att[1]=angle
#        # Roll
#        angle=AngleBetween(getCS(att)[:,1],CS[:,1])
#        if CS[:,1][2]>0:
#            angle*=-1
#        att[0]=angle
#        #att[0]=AngleBetween(EulerRotation(NorthVector,[0,0,1],att[2]+radians(90)),CS[:,1])
    else:
        PrintSpecial("This Function has not yet been implemented for the given Order of rotations! (getAttitude)","Red")
        quit()
    return att#np.round(att,re)
        
     
def getHeading(Vector,North=NorthVector,Up=StandardWorldCS[:,2]):
    if np.all(Vector==0.):
        return 0.
    # Unify
    Vector = unit_vector(Vector)
    North  = unit_vector(North)
    # Heading
    if np.all(Vector==North):
        return 0.
    elif np.all(Vector==-North):
        return np.pi
    Xproj=OrthProjection(Vector,Up)
    angle = AngleBetween(Xproj,North)
    
    n=unit_vector(np.cross(Xproj,North))
    if np.all(n==-Up):
        angle=2*np.pi-angle
    return angle

def getPitch(Vector,Up=StandardWorldCS[:,2]):
    proj = OrthProjection(Vector,Up)
    pitch = AngleBetween(Vector,proj)

    if VectorAbs(unit_vector(Vector-proj)+Up)<10**(-re):
        pitch*=-1

    return pitch
     
def TransformationMatrix(M1,M2):
    '''
    Returns the Transformation Matrix to represent Vectors from one coordinate System (M1) in another (M2).
    '''        
    
    # Check Coordinate Systems
    if isinstance(M1, CoordinateSystem):
        M1=M1.matrix
    if isinstance(M2, CoordinateSystem):
        M2=M2.matrix
        
    # Get Transformation Matrix
    x1=M1[:,0]
    y1=M1[:,1]
    z1=M1[:,2]
    x2=M2[:,0]
    y2=M2[:,1]
    z2=M2[:,2]
    
    Q=np.round(np.array([[np.cos(AngleBetween(x2, x1)), np.cos(AngleBetween(x2, y1)), np.cos(AngleBetween(x2, z1))],
                [         np.cos(AngleBetween(y2, x1)), np.cos(AngleBetween(y2, y1)), np.cos(AngleBetween(y2, z1))],
                [         np.cos(AngleBetween(z2, x1)), np.cos(AngleBetween(z2, y1)), np.cos(AngleBetween(z2, z1))]]),re)
        
    return Q
    
def Transform(v,M1=StandardWorldCS,M2=StandardWorldCS):
    # Check Coordinate Systems
    if isinstance(M1, CoordinateSystem):
        M1=M1.matrix
    if isinstance(M2, CoordinateSystem):
        M2=M2.matrix 
        
    v=np.array(v)
    
    return np.dot(TransformationMatrix(M1,M2),v)    
    
    
def getdifatt(att1,att2):
    difatt=att1-att2
    
    difatt[2]=getdifHeading(att1[2],att2[2])
    
    return difatt
    
def getdifHeading(h1,h2):
    
    if h2>np.pi:
        h2-=2*np.pi
#    else:
#        H=h2
        
    if h1>np.pi:
        h1-=2*np.pi
#    else:
#        refH=h1
        
    difh=h1-h2
    
    if difh<-np.pi:
        difh+=2*np.pi
    elif difh>np.pi:
        difh-=2*np.pi
        
    return difh   

def signedExpVector(vec,exp):
    return signedExp(VectorAbs(vec),exp)*unit_vector(vec)

#****************************************************************************************************
# Test Area
#****************************************************************************************************

if __name__=="__main__":

    #================================================================================
    # Unit Performance Tests
    #================================================================================

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Imports
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    from collections import OrderedDict
    from copy        import copy
    from inspect     import getargspec
    from time        import time        as getTime

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Settings
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    #--- Performance ---
    NrTests = 10#50#100#10**4#1000#10
    NrRuns  = 1000 # Should be at least 1000, to be able to derive the time in us
    NrSets  = 4

    #--- Naming Conventions ---
    VectorNames = ["vector","vec","v","v1","v2","axis","up","att1","att2"]
    MatrixNames = ["m","m1","m2","cs"]

    #--- Exceptions ---
    Exceptions = ["GetFunctionsFromDict"]

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Initialization
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    #----------------------------------------
    # Get Sample Set
    #----------------------------------------
    
    Values   = np.random.random([NrTests, NrSets      ])
    Vectors  = np.random.random([NrTests, NrSets, 3   ])
    Matrices = np.random.random([NrTests, NrSets, 3, 3])

    # print Values
    # print Vectors
    
    #----------------------------------------
    # Get Functions
    #----------------------------------------
    
    def GetFunctionsFromDict(dd,Print=False):
        functions = []
        for key in dd:
            obj = dd[key]
            if Print: print " "*3,key,type(obj),key[:2]
            if callable(obj) and key[:2]!="__" and key not in Exceptions:
                try:
                    if Print: print "FUNCTION FOUND",getargspec(obj),"!"*10
                    functions.append(obj)
                except:
                    if Print: print "FUNCTION IS NOT VALID!","X"*10

        return functions
                
    GLO = copy(globals())
    # print "globals ("+str(type(GLO))+"):"
    TestFunctions = GetFunctionsFromDict(GLO)
    TestFunctions = [tf for tf in TestFunctions if (tf.__module__=="__main__" and not "__main__" in str(tf))]

    #----------------------------------------
    # Get Function Parameter Masks
    #----------------------------------------
    
    ParameterMasks = dict()

    for function in TestFunctions:

        #..............................
        # Get Parameters
        #..............................
        
        args_i = getargspec(function)
        args     = args_i[0]
        defaults = args_i[3]

        NrArgs = len(args)

        print function.__name__,NrArgs,args_i

        #..............................
        # Get Input Masks
        #..............................

        Inputs = []
        for i in range(NrArgs):
            i_ = -i-1

            if args[i_]=="RotationSigns":
                Inputs.append([1,1,1]);continue

            #--- Check Defaults ---
            if defaults is not None and len(defaults)>i:
                # print i_,len(defaults)
                if type(defaults[i_])==np.ndarray:
                    # print "shape",defaults[i_].shape
                    if defaults[i_].shape==(3,):
                        Inputs.append("Vec");continue
                    elif defaults[i_].shape==(3,3):
                        Inputs.append("Mat3");continue
                    elif defaults[i_].shape==(2,2):
                        Inputs.append("Mat2");continue
                elif type(defaults[i_])==list:
                    Inputs.append("Vec");continue
                elif type(defaults[i_])==float or type(defaults[i_])==int:
                    Inputs.append("Val");continue
                elif type(defaults[i_])==bool:
                    Inputs.append(defaults[i_]);continue
                else:
                    pass
            #--- Check by Name ---
            if args[i_].lower() in MatrixNames:
                Inputs.append("Mat3")
            elif args[i_].lower() in VectorNames:
                Inputs.append("Vec")
            else:
                Inputs.append("Val")

        Inputs = Inputs[::-1]

        ParameterMasks[function.__name__] = Inputs

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Run Tests
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    SuccesfullyExecuted = 0

    function_total = OrderedDict()
    function_dts   = OrderedDict()

    for function in TestFunctions:
        Mask = ParameterMasks[function.__name__]
        dts = []
        t00 = getTime()
        for n in range(NrTests):

            #----------------------------------------
            # Get Values
            #----------------------------------------
            
            Vals = Values[n]
            Vecs = Vectors[n]
            Mats = Matrices[n]

            # print Vals,Vecs

            #----------------------------------------
            # Get Inputs
            #----------------------------------------
            
            Inputs = []
            for i in range(len(Mask)):
                if   Mask[i]=="Val":  Inputs.append(Vals[i])
                elif Mask[i]=="Vec":  Inputs.append(Vecs[i])
                elif Mask[i]=="Mat3": Inputs.append(Mats[i])
                elif Mask[i]=="Mat2": Inputs.append(Mats[i][:2,:2])

            #----------------------------------------
            # Run Function
            #----------------------------------------
            
            t0 = getTime()
            try:
                for z in range(NrRuns):
                    result = function(*Inputs)
            except:
                print ">"*3,"Failed to execute function",[type(i) for i in Inputs],"!"*30
                continue
            dt = getTime()-t0
            dts.append(dt)
            SuccesfullyExecuted+=1
            if n+1==NrTests or (n+1)%int(NrTests/5)==0:
                print ">"*2,"Function '"+function.__name__+"' Executed Succesfully",n+1,"*",NrRuns #,result
            # for i in dir(function):
            #     print " "*3,i   
                
        dt = getTime()-t00
        function_total[function.__name__] = dt
        function_dts[  function.__name__] = dts

    NrCalls = len(TestFunctions)*NrTests
    print "Succesfully Executed ",SuccesfullyExecuted,"/",NrCalls,round(100.*SuccesfullyExecuted/NrCalls,1),"%"            
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Post Processing
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    function_name_max = np.max([len(n) for n in function_dts])
    
    print "Function:"+" "*(function_name_max-9+3)+"Avg,\t Min,\t Max,\t StDV,\t Sum, Total,Sum/Total [us]"

    functions_sorted = sorted(function_dts, key = lambda key: np.mean(function_dts[key]))[::-1]
    for function in functions_sorted:
        dts   = function_dts[function]
        Total = function_total[function]

        Avg = np.mean(dts)/float(NrRuns)
        Min = np.min( dts)/float(NrRuns)
        Max = np.max( dts)/float(NrRuns)
        Std = np.std( dts)/float(NrRuns)
        Sum = np.sum(dts)

        def Convert(v,pow=6,dec = 0):#1)
            v*=10**pow
            if dec==0: return int(v)
            else:      return round(v,dec)

        print function+":"+" "*(function_name_max-len(function)+1),\
        Convert(Avg),"\t",\
        Convert(Min),"\t",\
        Convert(Max),"\t",\
        Convert(Std),"\t",\
        Convert(Sum),Convert(Total),round(100.*Sum/Total,1),"%"
    quit()
    #================================================================================
    # Imports
    #================================================================================

    from Colors import Colors
    from Models import CoordinateSystem3D
    from Window3D import Window

    #================================================================================
    # Initialization
    #================================================================================
    
    pos=(-1,-1,-1)
    originalCS=CoordinateSystem3D(pos=pos,opacity=0.3,labels=['x','y','z'])

    Window.center = np.zeros(3)
    Window.up=(0,0,1)
    Window.range = 3

    Range=45

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Test CS
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    refCS=CoordinateSystem3D(att=[np.radians((np.random.random()-0.5)*Range),np.radians((np.random.random()-0.5)*Range),np.radians((np.random.random()-0.5)*Range)],opacity=0.3)#originalCS#pos=pos,
    print "RefCS: ",[ np.degrees(a) for a in refCS.att]
    CS1=CoordinateSystem3D(refCS=refCS.matrix,RotationSigns=CopterRotationSigns,colors=[Colors["White"]]+[Colors["Chartreuse"]]*2,opacity=0.5,labels=['x1','y1','z1'])#)att=[np.random.random(),np.random.random(),np.random.random()])
    CS1.getAttitude()
    print "CS1: ",[ np.degrees(a) for a in CS1.att]
    CS2=CoordinateSystem3D(refCS=refCS.matrix,RotationSigns=CopterRotationSigns,colors=[Colors["Black"]]+[Colors["Orange"]]*2,opacity=0.5,labels=['x2','y2','z2'])#,att=[np.radians(np.random.random()*Range),np.radians(np.random.random()*Range),np.radians(np.random.random()*Range)])
    CS2.getAttitude()
    print "CS2: ",[ np.degrees(a) for a in CS2.att]

    #================================================================================
    # Run Basic Tests
    #================================================================================
    
    print "\nStart Test!:\n"

    for i in range(10):
       print "-"*50
       print "Test #"+str(i+1)+"\n"

       CS2.shiftCS([np.radians((np.random.random()-0.5)*Range),np.radians((np.random.random()-0.5)*Range),np.radians((np.random.random()-0.5)*Range)])
       
       Difatt=getdifatt(CS2.getAttitude(),CS1.getAttitude())#(getAttitude(CS2,CS1.refCS,CopterRotationSigns),getAttitude(CS1,CS1.refCS,CopterRotationSigns))
       
       print "CS1: ",[ np.degrees(a) for a in CS1.att]
       print "CS2: ",[ np.degrees(a) for a in CS2.att]
       
       print "Difatt:",[ np.degrees(a) for a in Difatt]
       
       #raw_input("Shift?")
       print "\nShift\n"
       CS1.shiftCS(Difatt)#CS1.setCS(Difatt)#
       CS1.Update()
       
       Difatt=getdifatt(CS2.getAttitude(),CS1.getAttitude())
       

       
       print "CS1: ",[ np.degrees(a) for a in CS1.att]
       print "CS2: ",[ np.degrees(a) for a in CS2.att]
       
       print "Difatt:",[ round(np.degrees(a),re) for a in Difatt]
       if not np.all(abs(Difatt)<10**(-7)):
           PrintSpecial("Difference is not 0!!!","Red_H") 
       print
       
       print "CS1 -ref: \n",CS1.refCS
       print "CS2 -ref: \n",CS2.refCS
       
       print "CS1 -matrix: \n",CS1.matrix
       print "CS2 -matrix: \n",CS2.matrix
       
       print "CS1 -Rot: \n",CS1.RotationSigns
       print "CS2 -Rot: \n",CS2.RotationSigns

    #    raw_input("next?")
    #quit()

    #================================================================================
    # Run Unit Tests
    #================================================================================
