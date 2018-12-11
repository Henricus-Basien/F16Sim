#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Imports
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import subprocess
import shlex

from time import time as getTime

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Run
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def CallF16Sim_cpp(parameters="",MeasureTime=False):#True):
    cmd = "./F16Sim"
    if os.name=="nt":
        #cmd = 'bash -c "'+cmd+'"'
        cmd = "F16Sim.exe"

    cmd+=" "+parameters
    if MeasureTime: t0_ = getTime()

    if 0:
        os.system(cmd)
    elif 1:
        subprocess.call(shlex.split(cmd))
        #subprocess.call(cmd,shell=True)
    else:
        p = subprocess.Popen(shlex.split(cmd))
        p.wait()

    if MeasureTime:
        dt_ = getTime()-t0_
        print "Sim exe time: "+str(round(dt_*1000,2))+" ms | MaxFreq: "+str(round(1./dt_,2))+" Hz"

if __name__=="__main__":
    parameters = ""
    # parameters = " ".join([str(1.23456) for i in range(18)])
    parameters = " 14.3842921301 0.0 999.240528869 0.0 0.0680626236787 0.0 100.08096494 0.121545455798 0.0 0.0 -0.031583522788 0.0 20000.0 0.0 0.0 0.0 0.0 1.0"
    CallF16Sim_cpp(parameters)