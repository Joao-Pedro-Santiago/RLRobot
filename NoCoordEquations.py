import numpy as np
import math
import matplotlib.pyplot as plt
import sys

sys.path.insert(0, '.')
from util import NRMSE

class NoCoord(object):
  # Abstracts the estimated calculations about NoCoord algorithm.   
  
  ms1 = 4
  
  Tstart = staticmethod(lambda vmed,n: NoCoord.ms1/(vmed*(n+1)))
  Tarrive = staticmethod(lambda s,vmed,D: (D-s)/vmed)
  Tfilltarget = staticmethod(lambda s,vmed: s/vmed)
  Tleavecongestion = staticmethod(lambda s,vmed,D: (D-s)/vmed)

  C = lambda s,vmed,n,D: NoCoord.Tstart(vmed,n) + NoCoord.Tarrive(s,vmed,D) + NoCoord.Tfilltarget(s,vmed) + NoCoord.Tleavecongestion(s,vmed,D)

  thresholdNum = staticmethod(lambda s,I: 2*((math.pi*(s+I)**2)/(math.pi*(I)**2)))

  f2 = staticmethod(lambda p,p2,n,I,D,s : p*(n**(1.5)/(1.5*math.sqrt(math.pi))-n) if n > NoCoord.thresholdNum(s,I) else p2*(n))
  
  @staticmethod
  def bestK(f, xs, ys, I, s, vmed, D, i0=0):
    thr = [NoCoord.thresholdNum(s,I[i]) for i in range(i0,len(xs))]
    Aq = sum([f(1,1,xs[i],I[i],D,s)*(ys[i] - NoCoord.C(s,vmed[i],xs[i],D)) for i in range(i0,len(xs)) if xs[i] > thr[i]])
    Bq = sum([f(1,1,xs[i],I[i],D,s)**2 for i in range(i0,len(xs)) if xs[i] > thr[i]])
    Kq = Aq/Bq if Bq != 0 else 0
    Al = sum([f(1,1,xs[i],I[i],D,s)*(ys[i] - NoCoord.C(s,vmed[i],xs[i],D)) for i in range(i0,len(xs)) if xs[i] <= thr[i]])
    Bl = sum([f(1,1,xs[i],I[i],D,s)**2 for i in range(i0,len(xs)) if xs[i] <= thr[i]])
    Kl = Al/Bl if Bl != 0 else 0
    return Kq,Kl
  
  @staticmethod
  def calcTime(f, n, vmed, s, I, bestK1, bestK2, D):
    Tfinal = f(bestK1,bestK2,n,I,D,s)
    Tblock = [NoCoord.Tstart(vmed,n), NoCoord.Tarrive(s,vmed,D), NoCoord.Tfilltarget(s,vmed), Tfinal, NoCoord.Tleavecongestion(s,vmed,D)]
    return sum(Tblock)
  
  @staticmethod
  def checkKs(Y1, Y2, dataMean, a, option):
    # Code for confirming the best Ks and show the best difference between calcTimes.
    diff1 = sum([(Y1[i] - dataMean[i,a,option])**2 for i in range(len(dataMean[:,a,option]))])
    diff2 = sum([(Y2[i] - dataMean[i,a,option])**2 for i in range(len(dataMean[:,a,option]))])
    print("difference for chosen Ks:",diff1,diff2)
  
  @staticmethod
  def doTheFitting(xs,ys,maxIndex=None):
    maxFit = len(xs)//2 if maxIndex==None else maxIndex
    sqrtData = [math.sqrt(v) for v in xs]
    mymodel0 = np.poly1d(np.polyfit(sqrtData[:maxFit],ys[:maxFit],3))
    print(mymodel0)
    mymodel0values = mymodel0(sqrtData)
    return mymodel0values
  
  @staticmethod
  def plotEstimation(varValues, dataMean, a, option, s, D):
    I = 0.5*np.array(dataMean[:,a,13])
    vmed = dataMean[:,a,16]
    K1,K2 = NoCoord.bestK(NoCoord.f2,varValues,dataMean[:,a,option],I,s,vmed,D)
    Y1 = [NoCoord.calcTime(NoCoord.f2,varValues[i],vmed[i],s,I[i],K1,K2,D) for i in range(len(varValues))]
    print("NoCoord K1,K2 =", round(K1,4), round(K2,4),"NRMSE=",round(NRMSE(dataMean[:,a,option],Y1),4))
    plt.plot(varValues,Y1,label="Estimation")
    #Plot estimations by polynomial fitting.
    if False:
      mymodel0values = NoCoord.doTheFitting(varValues,dataMean[:,a,option],len(varValues))
      plt.plot(varValues,mymodel0values,label="reg")
  
  @staticmethod
  def plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed2, vmed2, plotIt=True):
    if 0 in dataMean[:,m,a,i_sf,13] and Imed2 != None:
      Imed = Imed2
    else:
      Imed = 0.5*np.array(dataMean[:,m,a,i_sf,13])
    if 0 in dataMean[:,m,a,i_sf,16] and vmed2 != None:
      vmed = vmed2
    else:
      vmed = dataMean[:,m,a,i_sf,16]
    K1,K2 = NoCoord.bestK(NoCoord.f2,varValues,dataMean[:,m,a,i_sf,option],Imed,s,vmed,D)
    Y2 = [NoCoord.calcTime(NoCoord.f2,varValues[i],vmed[i],s,Imed[i],K1,K2,D) for i in range(len(varValues))]
    if plotIt: plt.plot(varValues,Y2,label="Estimation"%K2)
    return Y2
