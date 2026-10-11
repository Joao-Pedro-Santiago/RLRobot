import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array


sys.path.insert(0, '.')
from NoCoordEquations import NoCoord
from SQFEquations import SQF
from TRVFEquations import TRVF

def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

# ~ h = "holo"
h = "nonholo"
al = "NoCoord"
S = [3,5,7]
# ~ S = [3]
D = 13

algorithmDirectories = {"holo":{}, "nonholo":{}}
prefixNum = {"holo":{}, "nonholo":{}}
algorithmsLabels = {"holo":{}, "nonholo":{}}
for h1 in ["holo","nonholo"]:
  for alg1 in ["NoCoord","SQF","TRVF"]:
    algorithmDirectories[h1][alg1] = []
    prefixNum[h1][alg1] = []
    algorithmsLabels[h1][alg1] = [] 

for s in S:
  algorithmDirectories["holo"]["NoCoord"] += ['neighbourAngle zero experiments/holo/NoCoord/s'+str(s)+'/']
  prefixNum["holo"]["NoCoord"] += ['n_']
  algorithmsLabels["holo"]["NoCoord"] += ['NC'];
  
  algorithmDirectories["nonholo"]["NoCoord"] += ['neighbourAngle zero experiments/nonholo/NoCoord/s'+str(s)+'/']
  prefixNum["nonholo"]["NoCoord"] += ['n_']
  algorithmsLabels["nonholo"]["NoCoord"] += ['NC'];
  
  
  algorithmDirectories["holo"]["TRVF"] += ['original root/holo/TRVF/s'+str(s)+'/']
  prefixNum["holo"]["TRVF"] += ['n_']
  algorithmsLabels["holo"]["TRVF"] += ['TRVF']
  
  algorithmDirectories["nonholo"]["TRVF"] += ['original root/nonholo/TRVF/s'+str(s)+'/']
  prefixNum["nonholo"]["TRVF"] += ['n_']
  algorithmsLabels["nonholo"]["TRVF"] += ['TRVF']
  
  algorithmDirectories["nonholo"]["SQF"] += ['neighbourAngle zero experiments/nonholo/SQF/s'+str(s)+'/']
  prefixNum["nonholo"]["SQF"] += ['n_']
  algorithmsLabels["nonholo"]["SQF"] += ['SQF']
  
  algorithmDirectories["holo"]["SQF"] += ['neighbourAngle zero experiments/holo/SQF/s'+str(s)+'/']
  prefixNum["holo"]["SQF"] += ['n_']
  algorithmsLabels["holo"]["SQF"] += ['SQF']

printValuesForTTest = False # Set true if one wishes to print values for t-test.

def initData(algorithmLabelsList, algorithmDirectoriesList, prefixNumList):
  algorithmsSymbol = ["."]*len(algorithmLabelsList);
  varValues = range(20, 301, 20)
  nSamples = [40]*len(algorithmLabelsList);
  
  datalines = 19+1
  data = np.zeros((len(varValues),len(algorithmDirectoriesList),max(nSamples),datalines));
  dataMean = np.zeros((len(varValues),len(algorithmDirectoriesList),datalines));
  dataVari = np.zeros((len(varValues),len(algorithmDirectoriesList),datalines));
  dataUpCi = np.zeros((len(varValues),len(algorithmDirectoriesList),datalines));
  
  for n in range(len(varValues)):
    for a in range(len(algorithmDirectoriesList)):
      algorithm = algorithmDirectoriesList[a];
      for smp in range(nSamples[a]):
        dataFile = open(algorithm+"/"+prefixNumList[a]+str(varValues[n])+"/log_"+str(smp));
        dataFileStr = dataFile.readlines();
        for option in range(min(datalines,len(dataFileStr))):
          if option == 19:
            FirstRobotReachingTimeline = 8 
            LastRobotReachingTimeline = 9
            data[n, a, smp, option] = (n-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
          elif option in [11,12,13,14,16,17]:
            data[n, a, smp, option] = float(dataFileStr[option]);
          elif option in [8,9,10]:
            data[n, a, smp, option] = float(dataFileStr[option])/1e6;
          else:
            data[n, a, smp, option] = int(dataFileStr[option]);
      for option in range(datalines):
        dataMean[n, a, option] = np.mean(data[n,a,:nSamples[a],option]);
        dataVari[n, a, option] = np.var(data[n,a,:nSamples[a],option]);
        if all(data[n,a,0,option] == rest for rest in data[n,a,:,option]):
          dataUpCi[n, a, option] = dataMean[n, a, option]
        else:
          dataUpCi[n, a, option] = calcConfInt(dataMean[n, a, option],dataVari[n, a, option],nSamples[a]);
  return varValues, algorithmsSymbol, nSamples, dataMean, dataVari, dataUpCi


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
  "Time for reaching target of the first robot (s)", # 8
  "Time for reaching target of the last robot (s)",  # 9
  "Simulation time (s)",                             # 10
  "Minimum distance (m)",                            # 11
  "Maximum velocity (m/s)",                          # 12
  "Mean distance between robots (m)",                # 13
  "St. dev. of distance (m)",                        # 14
  "Number of samples for distance",                  # 15
  "Mean linear speed (m/s)",                         # 16
  "St. dev. of lin. speed (m/s)",                    # 17
  "Number of samples for velocity",                  # 18
  "Throughput (1/s)"                                 # 19
]
  
    
def selectPlotEstimation(varValues, dataMean, a, option, s, D):
  if al == "TRVF":
    TRVF.plotEstimation(varValues, dataMean, a, option, s, D)
  elif al == "NoCoord":
    NoCoord.plotEstimation(varValues, dataMean, a, option, s, D)
  elif al == "SQF":
    SQF.plotEstimation(varValues, dataMean, a, option, s, D)

plt.rcParams.update({'font.size': 20})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def mainLoop(option, showEstimated=True):
  varValues, algorithmsSymbol, nSamples, dataMean, dataVari, dataUpCi = initData(algorithmsLabels[h][al], algorithmDirectories[h][al], prefixNum[h][al])
  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for a in range(len(algorithmDirectories[h][al])):
    # ~ plt.errorbar(varValues,dataMean[:,a,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,option],dataMean[:,a,option])], label="Experiments",marker=algorithmsSymbol[a],capsize=5);
    plt.errorbar(varValues,dataMean[:,a,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,option],dataMean[:,a,option])], label=algorithmsLabels[h][al][a],marker=algorithmsSymbol[a],capsize=5);
    
    holoSubStr = "nonholo" if "nonholo" in algorithmDirectories[h][al][a] else "holo"
    if showEstimated:
      print(h,al,list_line_ylabel[option],algorithmsLabels[h][al][a],S[a])
      selectPlotEstimation(varValues, dataMean, a, option, S[a], D)
    
    if printValuesForTTest:
        print('#',end='')
        print(algorithmsLabels[h][al][a],holoSubStr,sep='-------')
        print('means'+algorithmsLabels[h][al][a]+' = ',end='')
        print(*dataMean[:,a,option], sep=', ')
        print('vari'+algorithmsLabels[h][al][a]+' = ',end='')
        print(*dataVari[:,a,option], sep=', ')
    plt.legend()
    plt.xlabel("Number of robots");
    plt.ylabel(list_line_ylabel[option])
    plt.savefig("FigureEQ"+al+str(option)+holoSubStr+"s"+str(S[a])+".pdf",bbox_inches="tight",pad_inches=0.00);
    # ~ plt.show()
    plt.clf()

def mainLoopAlgorithmsTogether(algs, option):
  for h in ["holo", "nonholo"]:
    AlgsLab = []
    AlgsDir = []
    PrefNum = []
    for al1 in algs:
      AlgsLab += algorithmsLabels[h][al1]
      AlgsDir += algorithmDirectories[h][al1]
      PrefNum += prefixNum[h][al1]

    varValues, algorithmsSymbol, nSamples, dataMean, dataVari, dataUpCi = initData(AlgsLab, AlgsDir, PrefNum)
    if printValuesForTTest:
      print('==== '+list_line_ylabel[option]+' ====')
  
    for a in range(len(AlgsDir)):
      plt.errorbar(varValues,dataMean[:,a,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,option],dataMean[:,a,option])], label=AlgsLab[a],marker=algorithmsSymbol[a],capsize=5);
      
      if printValuesForTTest:
          print('#',end='')
          print(AlgsLab[a],h,sep='-------')
          print('means'+AlgsLab[a]+' = ',end='')
          print(*dataMean[:,a,option], sep=', ')
          print('vari'+AlgsLab[a]+' = ',end='')
          print(*dataVari[:,a,option], sep=', ')
    plt.legend()
    plt.xlabel("Number of robots");
    plt.ylabel(list_line_ylabel[option])
    plt.savefig("Option"+str(option)+h+".pdf",bbox_inches="tight",pad_inches=0.00);
    # ~ plt.show()
    plt.clf()

mainLoop(10)
# ~ mainLoop(10,False)
# ~ mainLoopAlgorithmsTogether(["NoCoord","SQF","TRVF"], 16)
# ~ mainLoopAlgorithmsTogether(["NoCoord","SQF","TRVF"], 13)

