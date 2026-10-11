import numpy as np
import math
import matplotlib.pyplot as plt
import sys

sys.path.insert(0, '.')
from util import NRMSE

class SQF(object):
  ms1 = 4
  
  @staticmethod
  def TleavingTarget(vmed,s,D):
    L = math.sqrt(D**2 - s**2) 
    ang1 = math.acos(L/(2*D))
    ang2 = math.asin(s/D)
    return (ang1 + ang2)*L/vmed
  
  TfillCorridor = staticmethod(lambda vmed,s,D: (D - s)/(vmed))
  
  @staticmethod
  def TLongestArrive(vmed,n,s,D): 
    Res = (n*(math.pi - math.asin(s/D))*D)/(vmed*(n+2)) 
    return Res
  
  TEnterAlgArea = staticmethod(lambda vmed,n: (n*SQF.ms1)/(vmed*(n+1)))
  
  C = staticmethod(lambda vmed,s,n,D: SQF.TfillCorridor(vmed,s,D) + SQF.TleavingTarget(vmed,s,D) + SQF.TEnterAlgArea(vmed,n) + SQF.TLongestArrive(vmed,n,s,D))

  
  @staticmethod
  def calcTh(s,D,Im):
    DSemiCircleArea = math.pi*D**2/2
    Height = math.sqrt(D**2 - s**2) - s
    Triangle = 0.5*Height*2*s
    Beta = 2*math.asin(s/D)
    Segment = 0.5*D**2*(Beta - math.sin(Beta))
    A1 = DSemiCircleArea - Triangle - Segment
    A2 = 0.5*(D-s)**2*math.acos((D-s)/(2*D))
    A3 = D**2*math.asin((D-s)/(2*D)) - ((D-s)/4)*math.sqrt(4*D**2 - (D-s)**2)
    Area = A1+2*(A2+A3)
    
    Thr = Area/(4*Im**2)
    return Thr
  
  @staticmethod
  def f1(Kl,Ks,n,s,D,Im):
    Thr = SQF.calcTh(s,D,Im)
    if n > Thr:
      return Ks*((n)**2)
    else:
      return Kl*n
  
  @staticmethod
  def bestK(f, xs, ys, vs, s, D, Ims):
    Cs = [SQF.C(vs[i],s,xs[i],D) for i in range(len(xs))]
    Thr = [SQF.calcTh(s,D,Ims[i]) for i in range(len(Ims))]
    As = sum([f(1,1,xs[i],s,D,Ims[i])*(ys[i] - Cs[i]) for i in range(len(xs)) if xs[i] >= Thr[i]])
    Bs = sum([f(1,1,xs[i],s,D,Ims[i])**2 for i in range(len(xs)) if xs[i] >= Thr[i]])
    Ks = As/Bs if Bs != 0 else 0
    Al = sum([f(1,1,xs[i],s,D,Ims[i])*(ys[i] - Cs[i]) for i in range(len(xs)) if xs[i] < Thr[i]])
    Bl = sum([f(1,1,xs[i],s,D,Ims[i])**2 for i in range(len(xs)) if xs[i] < Thr[i]])
    Kl = Al/Bl if Bl != 0 else 0
    return Kl,Ks

  @staticmethod
  def doTheFitting(xs,ys,maxIndex=None):
    maxFit = len(xs)//2 if maxIndex==None else maxIndex
    mymodel0 = np.poly1d(np.polyfit(xs[:maxFit],ys[:maxFit],2))
    print(mymodel0)
    mymodel0values = mymodel0(xs)
    return mymodel0values

  @staticmethod
  def plotEstimation(varValues, dataMean, a, option, s, D):
    Imed = 0.5*np.array(dataMean[:,a,13])
    vmed = dataMean[:,a,16]
    Kl,Ks = SQF.bestK(SQF.f1,varValues,dataMean[:,a,option],vmed,s,D,Imed)
    Y1 = [SQF.C(vmed[i],s,varValues[i],D) + SQF.f1(Kl,Ks,varValues[i],s,D,Imed[i]) for i in range(len(varValues))]
    print("SQF Kl,Ks =", round(Kl,4),round(Ks,4),"NRMSE=",round(NRMSE(dataMean[:,a,option],Y1),4))
    plt.plot(varValues,Y1,label="Estimation")

  @staticmethod
  def plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed2, vmed2, plotIt=True):
    if 0 in dataMean[:,m,a,i_sf,13]:
      Imed = Imed2
    else:
      Imed = 0.5*np.array(dataMean[:,m,a,i_sf,13])
    if 0 in dataMean[:,m,a,i_sf,16]:
      vmed = vmed2
    else:
      vmed = dataMean[:,m,a,i_sf,16]
    Kl,Ks = SQF.bestK(SQF.f1,varValues,dataMean[:,m,a,i_sf,option],vmed,s,D,Imed)
    Y1 = [SQF.C(vmed[i],s,varValues[i],D) + SQF.f1(Kl,Ks,varValues[i],s,D,Imed[i]) for i in range(len(varValues))]
    if plotIt: plt.plot(varValues,Y1,label="Estimation")
    return Y1
