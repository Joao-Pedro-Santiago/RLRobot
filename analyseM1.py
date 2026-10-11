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

def readStatistics(prefixDirectory,algorithmsLabels,varValues,nSamples,datalines,holoSubStr,are_M_dirs):
  data = np.zeros((len(varValues),len(algorithmsLabels),max(nSamples),len(holoSubStr),datalines));
  dataMean = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),datalines));
  dataVari = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),datalines));
  dataUpCi = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),datalines));
  
  for h in range(len(holoSubStr)):
    for n in range(len(varValues)):
      for a in range(len(algorithmsLabels)):
        if are_M_dirs:
          algorithmLocation =  prefixDirectory+holoSubStr[h]+"/NoCoordAlt/"+algorithmsLabels[a]+"/m_1/n_"+str(varValues[n])
        else:
          algorithmLocation =  prefixDirectory+holoSubStr[h]+"/"+algorithmsLabels[a]+"/s3/n_"+str(varValues[n])
        for s in range(nSamples[a]):
          dataFile = open(algorithmLocation+"/log_"+str(s));
          dataFileStr = dataFile.readlines();
          for option in range(datalines):
            if option == 19:
              FirstRobotReachingTimeline = 8 
              LastRobotReachingTimeline = 9
              if are_M_dirs: 
                numRobs = varValues[n]+1
              else:
                numRobs = varValues[n]
              data[n, a, s, h, option] = (numRobs-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
            elif option in [11,12,13,14,16,17]:
              data[n, a, s, h, option] = float(dataFileStr[option]);
            elif option in [8,9,10]:
              data[n, a, s, h, option] = float(dataFileStr[option])/1e6;
            else:
              data[n, a, s, h, option] = int(dataFileStr[option]);
        for option in range(datalines):
          dataMean[n, a, h, option] = np.mean(data[n,a,:nSamples[a],h,option]);
          dataVari[n, a, h, option] = np.var(data[n,a,:nSamples[a],h,option]);
          if all(data[n,a,0,h,option] == rest for rest in data[n,a,:,h,option]):
            dataUpCi[n, a, h, option] = dataMean[n, a, h, option]
          else:
            dataUpCi[n, a, h, option] = calcConfInt(dataMean[n, a, h, option],dataVari[n, a, h, option],nSamples[a]);
  return dataMean, dataUpCi, data, dataVari
  
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

def plotLogs(a,h,option,nSamples,data,varValues):
  for s in range(nSamples[a]):
    plt.plot(varValues,data[:,a,s,h,option],color='orange',marker='x',linestyle="None");
    for i in range(len(varValues)):
      plt.annotate(s, (varValues[i], data[i,a,s,h,option]))


plt.rcParams.update({'font.size': 20})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def mainLoop(option,MTName="MT"):
  printValuesForTTest = False # Set true if one wishes to print values for t-test.
  '''
  Results from t-test
  
  SQF nonholo 20
  [(False, 0.004135464685349977)]
  SQF nonholo 260
  [(False, 0.00033571840620560955)]
  SQF nonholo 300
  [(False, 8.8737765061353e-05)]
  SQF holo 180
  [(False, 0.0008936504346601648)]
  TRVF nonholo 40
  [(False, 0.0022049086363644665)]
  '''
  
  
  prefixDirectory="./neighbourAngle zero experiments/"
  algorithmsLabels = ["SQF","TRVF"];
  algorithmsSymbol = ["."]*len(algorithmsLabels);
  varValues = range(19, 300, 20)
  nSamples = [40]*len(algorithmsLabels);
  datalines = 19+1
  holoSubStr = ['nonholo','holo']

  dataMean, dataUpCi, data, dataVari = readStatistics(prefixDirectory,algorithmsLabels,varValues,nSamples,datalines,holoSubStr,True)
  
  prefixDirectoryAlg="./original root/"
  varValuesAlg = range(20, 301, 20)
  dataMeanAlg, dataUpCiAlg, dataAlg, dataVariAlg = readStatistics(prefixDirectoryAlg,algorithmsLabels,varValuesAlg,nSamples,datalines,holoSubStr,False)


  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for a in range(len(algorithmsLabels)):
    for h in range(len(holoSubStr)):
      varValuesP1 = [x+1 for x in varValues]
      plt.errorbar(varValuesP1,dataMean[:,a,h,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,h,option],dataMean[:,a,h,option])], label=MTName,marker=algorithmsSymbol[a],capsize=5);
      plt.errorbar(varValuesP1,dataMeanAlg[:,a,h,option], yerr=[m - n for m,n in zip(dataUpCiAlg[:,a,h,option],dataMeanAlg[:,a,h,option])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
      
      # ~ plotLogs(a,h,option,nSamples,data,varValuesP1)
      
      if printValuesForTTest:
          print('#',end='')
          print(algorithmsLabels[a],holoSubStr[h],"no ad hoc",sep='-------')
          print('means'+'Alg = ',end='')
          print(*dataMeanAlg[:,a,h,option], sep=', ')
          print('vari'+'Alg = ',end='')
          print(*dataVariAlg[:,a,h,option], sep=', ')
          print('#',end='')
          print(algorithmsLabels[a],holoSubStr[h],"m=1",sep='-------')
          print('means'+'M1 = ',end='')
          print(*dataMean[:,a,h,option], sep=', ')
          print('vari'+'M1 = ',end='')
          print(*dataVari[:,a,h,option], sep=', ')
          textResult = algorithmsLabels[a]+" "+holoSubStr[h]
          print("printResult(meansAlg,variAlg,meansM1,variM1,n1,n2,\""+textResult+"\")")
      plt.legend()
      plt.xlabel("Number of robots");
      plt.ylabel(list_line_ylabel[option])
      imgLocation = "M1"+algorithmsLabels[a]+holoSubStr[h]+str(option)
      print(imgLocation + ".pdf generated.")
      plt.savefig(imgLocation+".pdf",bbox_inches="tight",pad_inches=0.001);
      # ~ plt.show()
      plt.clf()
  
mainLoop(10)
# ~ mainLoop(19)

