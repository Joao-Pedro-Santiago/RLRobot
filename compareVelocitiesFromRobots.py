'''
Open the log of positions and velocities of a robot and compare the velocity from the positions with the velocity logged.
'''
import math
import matplotlib.pyplot as plt
import os
from array import array
import numpy as np

nonhstr = "nh"
s = 3
n = 260
robot_num = 1

def inputfilenames(n,s,robot_num):
  logVbase = "tmp"+str(n)+"/SQF"+nonhstr+"s"+str(s)+"/robotslogs/robot"
  TimeVfilename = "tmp"+str(n)+"/SQF"+nonhstr+"s"+str(s)+"/TimeV"+str(robot_num)+".bin"
  VfromLogfilename = "tmp"+str(n)+"/SQF"+nonhstr+"s"+str(s)+"/VfromLog"+str(robot_num)+".bin"
  TimePosXfilename = "tmp"+str(n)+"/SQF"+nonhstr+"s"+str(s)+"/TimePosX"+str(robot_num)+".bin"
  TimePosYfilename = "tmp"+str(n)+"/SQF"+nonhstr+"s"+str(s)+"/TimePosY"+str(robot_num)+".bin"
  return logVbase,TimeVfilename,VfromLogfilename,TimePosXfilename,TimePosYfilename

def saveFile(outfilename, out):
  output_file = open(outfilename, 'wb')
  float_array = array('d', out)
  float_array.tofile(output_file)
  output_file.close()

def loadFile(filename):
  input_file = open(filename, 'rb')
  float_array = array('d')
  float_array.frombytes(input_file.read())
  return float_array

def VelocByTime(robot_number,logVbase,TimeVfilename,VfromLogfilename,TimePosXfilename,TimePosYfilename):
  if os.path.exists(TimeVfilename) and os.path.exists(VfromLogfilename) and os.path.exists(TimePosXfilename) and os.path.exists(TimePosYfilename):
    TimeV = loadFile(TimeVfilename)
    VfromLog = loadFile(VfromLogfilename)
    TimePosX = loadFile(TimePosXfilename)
    TimePosY = loadFile(TimePosYfilename)
  else:
    velocByTime = {}
    dataFile = open(logVbase+str(robot_number));
    dataFileStr = dataFile.read().splitlines();
    for line in dataFileStr:
      splitlines = line.split()
      if splitlines[4] != "output":
        t  = float(splitlines[0])/1e6
        px = float(splitlines[1])
        py = float(splitlines[2])
        v  = float(splitlines[3])
        if t in velocByTime:
          print("Warning: replacing state in", t)
        velocByTime[t] = (px,py,v)
    TimeV = []
    VfromLog = []
    TimePosX = []
    TimePosY = []
    for k in sorted(velocByTime.keys()):
      TimeV.append(k)
      TimePosX.append(velocByTime[k][0])
      TimePosY.append(velocByTime[k][1])
      VfromLog.append(velocByTime[k][2])
    saveFile(TimeVfilename, TimeV)
    saveFile(VfromLogfilename, VfromLog)
    saveFile(TimePosXfilename, TimePosX)
    saveFile(TimePosYfilename, TimePosY)
  return TimeV,VfromLog,TimePosX,TimePosY

def calculateVelocities(TimeV,VfromLog,TimePosX,TimePosY):
  Vcalc = []
  for i in range(len(TimeV)-1):
    px1 = TimePosX[i]
    py1 = TimePosY[i]
    px2 = TimePosX[i+1]
    py2 = TimePosY[i+1]
    t1 = TimeV[i]
    t2 = TimeV[i+1]
    vv = math.hypot((px2 - px1)/(t2 - t1),(py2 - py1)/(t2 - t1))
    Vcalc.append(vv)
  return TimeV[:-1], VfromLog[:-1], Vcalc

def plotVelocities(ts, vLog, vCalc):
  plt.plot(ts,vLog,label="Log",marker='.')
  plt.plot(ts,vCalc,label="Calc",marker='.')
  plt.legend()
  plt.show()

def plotMeanVelocities(ts, vLog, vCalc):
  plt.plot(0,np.mean(vLog),label="Log",marker='x')
  plt.plot(1,np.mean(vCalc),label="Calc",marker='x')
  plt.legend()
  plt.show()

logVbase,TimeVfilename,VfromLogfilename,TimePosXfilename,TimePosYfilename = inputfilenames(n,s,robot_num)
print(logVbase,TimeVfilename,VfromLogfilename,TimePosXfilename,TimePosYfilename)
TimeV,VfromLog,TimePosX,TimePosY = VelocByTime(robot_num,logVbase,TimeVfilename,VfromLogfilename,TimePosXfilename,TimePosYfilename)
ts, vLog, vCalc = calculateVelocities(TimeV,VfromLog,TimePosX,TimePosY)
plotMeanVelocities(ts, vLog, vCalc)
