import sys
import numpy as np 
import matplotlib
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array


def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue


prefixDirectory="./neighbourAngle zero experiments/"
algorithmsLabels = ["SQF","TRVF"];
alternativeAlgorithmsDirs = ["NoCoordAlt","SQFAlt","TRVFAlt"]
algorithmsSymbol = ["."]*len(algorithmsLabels);
varValues = np.arange(0, 91, 7.5)
nSamples = [40]*len(algorithmsLabels);
datalines = 19+1
holoSubStr = ['nonholo','holo']
numRobotsList = [100,200]
# ~ numRobotsList = [300,400]
probList = [10,50,90]
data = np.zeros((len(varValues),len(algorithmsLabels),max(nSamples),len(holoSubStr),len(numRobotsList),len(probList),len(alternativeAlgorithmsDirs),datalines));
dataMean = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),len(alternativeAlgorithmsDirs),datalines));
dataVari = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),len(alternativeAlgorithmsDirs),datalines));
dataUpCi = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),len(alternativeAlgorithmsDirs),datalines));
printValuesForTTest = False # Set true if one wishes to print values for t-test.


for h in range(len(holoSubStr)):
  for nRob in range(len(numRobotsList)):
    for p in range(len(probList)):
      for n in range(len(varValues)):
        for a in range(len(algorithmsLabels)):
          for aa in range(len(alternativeAlgorithmsDirs)):
            if algorithmsLabels[a] == "SQF" and alternativeAlgorithmsDirs[aa] == "SQFAlt" or algorithmsLabels[a] == "TRVF" and alternativeAlgorithmsDirs[aa] == "TRVFAlt": continue
            strValues = str(varValues[n]) if varValues[n] % 1 != 0 else str(int(varValues[n]))
            algorithmLocation =  prefixDirectory+holoSubStr[h]+"/"+alternativeAlgorithmsDirs[aa]+"/"+algorithmsLabels[a]+"/robots_"+str(numRobotsList[nRob])+"/m_"+str(int(numRobotsList[nRob]*probList[p]/100))+"/neighbourhoodAngle_"+strValues
            for s in range(nSamples[a]):
              dataFile = open(algorithmLocation+"/log_"+str(s));
              dataFileStr = dataFile.readlines();
              for option in range(datalines):
                if option == 19:
                  FirstRobotReachingTimeline = 8 
                  LastRobotReachingTimeline = 9
                  data[n, a, s, h, nRob, p, aa, option] = (numRobotsList[nRob]-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
                elif option in [11,12,13,14,16,17]:
                  data[n, a, s, h, nRob, p, aa, option] = float(dataFileStr[option]);
                elif option in [8,9,10]:
                  data[n, a, s, h, nRob, p, aa, option] = float(dataFileStr[option])/1e6;
                else:
                  data[n, a, s, h, nRob, p, aa, option] = int(dataFileStr[option]);
            for option in range(datalines):
              dataMean[n, a, h, nRob, p, aa, option] = np.mean(data[n,a,:nSamples[a],h,nRob,p,aa,option]);
              dataVari[n, a, h, nRob, p, aa, option] = np.var(data[n,a,:nSamples[a],h,nRob,p,aa,option]);
              if all(data[n,a,0,h,nRob,p,aa,option] == rest for rest in data[n,a,:,h,nRob,p,aa,option]):
                dataUpCi[n, a, h, nRob, p, aa, option] = dataMean[n, a, h, nRob, p, aa, option]
              else:
                dataUpCi[n, a, h, nRob, p, aa, option] = calcConfInt(dataMean[n, a, h, nRob, p, aa, option],dataVari[n, a, h, nRob, p, aa, option],nSamples[a]);
  
list_line_ylabel = [ 
  #           Label                                  # index
  #--------------------------------------------------#-------
  "Total number of iterations",                      # 0
  "Total iterations of the last robot",              # 1
  "Number of messages",                              # 2
  "Summation of the iter. for reaching",             # 3
  "Summation of the iter. for exiting",              # 4
  "Last robot's iterations for reaching",            # 5
  "Last robot's iterations for exiting",             # 6
  "Stalls",                                          # 7
  "First robot's reaching time (s)",                 # 8
  "Last robot's reaching time (s)",                  # 9
  "Total time of the simulation (s)",                # 10
  "Minimum distance (m)",                            # 11
  "Maximum velocity (m/s)",                          # 12
  "Mean distance (m)",                               # 13
  "St. dev. of distance (m)",                        # 14
  "Number of samples for distance",                  # 15
  "Mean velocity (m/s)",                             # 16
  "St. dev. of velocity (m/s)",                      # 17
  "Number of samples for velocity",                  # 18
  "Throughput (1/s)"                                 # 19
]

plt.rcParams.update({'font.size': 20})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def plotLogs(a,h,nRob,p,option,nSamples,data,varValues,aa):
  for s in range(nSamples[a]):
    plt.plot(varValues,data[:,a,s,h,nRob,p,aa,option],color='orange',marker='x',linestyle="None");
    for i in range(len(varValues)):
      plt.annotate(s, (varValues[i], data[i,a,s,h,nRob,p,aa,option]))
  plt.show()
  plt.clf()

def testTEqual(means1, variances1, n1, means2, variances2, n2, alpha=0.01):
  '''
    means1, means2: vector
    variances1, variances2: vector
    n1,n2: integer
    alpha: one minus confidence level
    return True if p-value >= alpha, that is, means are equal.
  '''
  l1,l2 = len(means1),len(means2)
  df = n1 + n2 - 2
  ts = [(means1[i] - means2[i])/math.sqrt(variances1[i]/n1 + variances2[i]/n2) for i in range(min(l1,l2))]
  ps = [(1-t.cdf(abs(ts[i]),df))*2 for i in range(min(l1,l2))]
  return [(ps[i] >= alpha, ps[i]) for i in range(min(l1,l2))]

def pairwiseTTest(index,meansSQF,variSQF,nsamples):
  finalresult=True
  i = 0
  RE = testTEqual([meansSQF[i]], [variSQF[i]], nsamples, [meansSQF[index]], [variSQF[index]], nsamples)
  if RE[0][0] == True:
    finalresult=False
  return finalresult

def mainLoop(option):
  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for aa in range(len(alternativeAlgorithmsDirs)):
    for a in range(len(algorithmsLabels)):
      if algorithmsLabels[a] == "SQF" and alternativeAlgorithmsDirs[aa] == "SQFAlt" or algorithmsLabels[a] == "TRVF" and alternativeAlgorithmsDirs[aa] == "TRVFAlt": continue
      for nRob in range(len(numRobotsList)):
        for h in range(len(holoSubStr)):
          for p in range(len(probList)):
            varValuesRad = [v*math.pi/180 for v in varValues]
            plt.errorbar(varValuesRad,dataMean[:,a,h,nRob,p,aa,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,h,nRob,p,aa,option],dataMean[:,a,h,nRob,p,aa,option])], label=str(probList[p])+"%" ,marker=algorithmsSymbol[a],capsize=5,zorder=1);
            minindex = list(dataMean[:,a,h,nRob,p,aa,option]).index(min(dataMean[:,a,h,nRob,p,aa,option]))
            if pairwiseTTest(minindex,dataMean[:,a,h,nRob,p,aa,option],dataVari[:,a,h,nRob,p,aa,option],nSamples[a]):
              plt.plot([varValuesRad[minindex]],[dataMean[minindex,a,h,nRob,p,aa,option]],color="orange",marker='s',zorder=2)
            # ~ plotLogs(a,h,nRob,p,option,nSamples,data,varValues,aa)
            if printValuesForTTest:
              print('#',end='')
              print(algorithmsLabels[a],holoSubStr[h],numRobotsList[nRob],probList[p],sep='-------')
              print('means'+algorithmsLabels[a]+' = ',end='')
              print(*dataMean[:,a,h,nRob,p,aa,option], sep=', ')
              print('vari'+algorithmsLabels[a]+' = ',end='')
              print(*dataVari[:,a,h,nRob,p,aa,option], sep=', ')
              textResult = algorithmsLabels[a]+" "+holoSubStr[h]+" n="+str(numRobotsList[nRob])+" p="+str(probList[p])
              print("printResult(means"+algorithmsLabels[a]+",vari"+algorithmsLabels[a]+",n1,n2,\""+textResult+"\")")
          plt.legend()
          plt.xlabel("Angle (rad)");
          plt.ylabel(list_line_ylabel[option])
          imgLocation = "AnglesRobots"+alternativeAlgorithmsDirs[aa]+algorithmsLabels[a]+str(numRobotsList[nRob])+holoSubStr[h]+str(option)
          print(imgLocation + " generated.")
          plt.savefig(imgLocation+".pdf",bbox_inches="tight",pad_inches=0.00);
          # ~ plt.savefig(imgLocation+".png",bbox_inches="tight",pad_inches=0.00);
          # ~ plt.show()
          plt.clf()


mainLoop(10)
# ~ mainLoop(19)

