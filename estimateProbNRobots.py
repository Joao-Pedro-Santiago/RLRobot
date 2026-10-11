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

from util import NRMSE

def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

'''Recent directories'''
algorithmDirectoriesNonHolo = {
  "TRVF": "original root/nonholo/TRVF/s3/n_",
  "SQF": "nonholo/SQF/s3/n_",
  "NC": "./nonholo/NoCoord/s3/n_",
  "Adhoc": "./nonholo/NoCoordAlt/Adhoc/m_"
}

algorithmDirectoriesHolo = {
  "TRVF": "original root/holo/TRVF/s3/n_",
  "SQF": "holo/SQF/s3/n_",
  "NC": "./holo/NoCoord/s3/n_",
  "Adhoc": "./holo/NoCoordAlt/Adhoc/m_"
}

def dictSlice(d,l):
  return {k:d[k] for k in l}

def dictHoloNonHolo(algorithmsLabelsList):
  return {
    'nonholo': dictSlice(algorithmDirectoriesNonHolo,algorithmsLabelsList),
    'holo': dictSlice(algorithmDirectoriesHolo,algorithmsLabelsList),
  }

'''
Short description of the directories:

if percent to compare is 0, the location is
  algDirPrefixes[suffix_file][algorithm label] + a num of Robots + "/log_"+sample
else
  "./"+suffix_file+"/"+algorithmAlternativeDir+algorithmsLabels[algorithm number]+"/"+prefixNumRobots;
'''

# Test for NoCoord as alternative and SQF or TRVF by the knowing robots. Adhoc is the NoCoord but robots follow the neighbour.
algorithmAlternativeDir="NoCoordAlt/"
algorithmsLabels = ["SQF","TRVF"];
algorithmsLabels2 = ["NC","NC"]

# Test for TRVF as alternative and SQF by the knowing robots.
# ~ algorithmAlternativeDir="TRVFAlt/"
# ~ algorithmsLabels = ["SQF"]
# ~ algorithmsLabels2 = ["TRVF"]

# Test for SQF as alternative and TRVF by the knowing robots.
# ~ algorithmAlternativeDir="SQFAlt/"
# ~ algorithmsLabels = ["TRVF"]
# ~ algorithmsLabels2 = ["SQF"]

suffix_file_list = ['holo','nonholo']
algDirPrefixes =dictHoloNonHolo(algorithmsLabels)
algDirPrefixes2 = dictHoloNonHolo(algorithmsLabels2)
prefixNumRobots = "robots_"
prefixNumFollowingRobots = "m_"

algorithmsSymbol = ["."]*(len(algorithmsLabels)+len(algorithmsLabels2));
numRobotsValues = range(20, 301, 20)
numPercentValues = range(10, 91, 10)
nSamples = 40
datalines = 19+1
printValuesForTTest = False # Set True if one wishes to print values for t-test.
S = 3
D = 13

def readStatistics(algorithmsLabels,algDirPrefixes,numPercentValues=[0],prefix_dir="./"):
  data = np.zeros((len(numRobotsValues),len(numPercentValues),len(algorithmsLabels),nSamples,len(suffix_file_list),datalines));
  dataMean = np.zeros((len(numRobotsValues),len(numPercentValues),len(algorithmsLabels),len(suffix_file_list),datalines));
  dataVari = np.zeros((len(numRobotsValues),len(numPercentValues),len(algorithmsLabels),len(suffix_file_list),datalines));
  dataUpCi = np.zeros((len(numRobotsValues),len(numPercentValues),len(algorithmsLabels),len(suffix_file_list),datalines));
  for i_sf, suffix_file in enumerate(suffix_file_list):
    for a in range(len(algorithmsLabels)):
      for n in range(len(numRobotsValues)):
        for m in range(len(numPercentValues)):
          for s in range(nSamples):
            if numPercentValues[m] == 0:
              logLocation = algDirPrefixes[suffix_file][algorithmsLabels[a]]+str(numRobotsValues[n])+"/log_"+str(s)
            else:
              algorithmDir = prefix_dir+suffix_file+"/"+algorithmAlternativeDir+algorithmsLabels[a]+"/"+prefixNumRobots;
              numFollowing = int(numPercentValues[m]*numRobotsValues[n]/100)
              logLocation = algorithmDir+str(numRobotsValues[n])+"/"+prefixNumFollowingRobots+str(numFollowing)+"/log_"+str(s)
            dataFile = open(logLocation);
            dataFileStr = dataFile.readlines();
            for option in range(min(datalines,len(dataFileStr))):
              if option == 19:
                FirstRobotReachingTimeline = 8 
                LastRobotReachingTimeline = 9
                data[n, m, a, s, i_sf, option] = (numRobotsValues[n]-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
              elif option in [11,12,13,14,16,17]:
                data[n, m, a, s, i_sf, option] = float(dataFileStr[option]);
              elif option in [8,9,10]:
                data[n, m, a, s, i_sf, option] = float(dataFileStr[option])/1e6;
              else:
                data[n, m, a, s, i_sf, option] = int(dataFileStr[option]);
          for option in range(datalines):
            dataMean[n, m, a, i_sf, option] = np.mean(data[n, m, a, :, i_sf, option]);
            dataVari[n, m, a, i_sf, option] = np.var(data[n, m, a, :, i_sf, option]);
            if all(data[n, m, a, 0, i_sf, option] == rest for rest in data[n, m, a, :, i_sf, option]):
              dataUpCi[n, m, a, i_sf, option] = dataMean[n, m, a, i_sf, option]
            else:
              dataUpCi[n, m, a, i_sf, option] = calcConfInt(dataMean[n, m, a, i_sf, option],dataVari[n, m, a, i_sf, option],nSamples);
  return dataMean, dataUpCi

dataMeanProb,dataUpCiProb = readStatistics(algorithmsLabels,algDirPrefixes,numPercentValues,"neighbourAngle zero experiments/") 
dataMean0,dataUpCi0 = readStatistics(algorithmsLabels,algDirPrefixes) 
dataMean1,dataUpCi1 = readStatistics(algorithmsLabels2,algDirPrefixes2) 

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
  "Simulation time (s)",                              # 10
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

def selectPlotEstimation(al,varValues, dataMean, a, option, s, D, m, i_sf, Imed=None, vmed=None, plotIt=False):
  if al == "TRVF":
    return TRVF.plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed, vmed, plotIt)
  elif al == "NC":
    return NoCoord.plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed, vmed, plotIt)
  elif al == "SQF":
    return SQF.plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed, vmed, plotIt)

plt.rcParams.update({'font.size': 20})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def calcminIndex(alternativeEstimate, algEstimate):
  minIndex = 0
  for i in range(len(alternativeEstimate)):
    if alternativeEstimate[i] > algEstimate[i]:
      minIndex = i
      break
  return minIndex

def adHocEstimate(K1,K2,percent,alternativeEstimate, algEstimate):
  minIndex = 0
  returnArray = np.zeros(len(alternativeEstimate))
  returnArray[:minIndex] = algEstimate[:minIndex]
  returnArray[minIndex:] = K1*np.array(algEstimate[minIndex:])+K2*np.array(alternativeEstimate[minIndex:])
  return returnArray

def bestK12List(f0,f1,y,ns,ms):
  min_n=0
  listKs = []
  for j in range(y.shape[1]):
    S1 = sum([y[i][j]*f0[i] for i in range(min_n,len(ns))])
    S2 = sum([y[i][j]*f1[i] for i in range(min_n,len(ns))])
    A = sum([f0[i]**2 for i in range(min_n,len(ns))])
    B = sum([f1[i]*f0[i] for i in range(min_n,len(ns))])
    C = sum([f1[i]**2 for i in range(min_n,len(ns))])
    K1 = ( C*S1 - B*S2)/(A*C-B*B)
    K2 = (-B*S1 + A*S2)/(A*C-B*B)
    listKs.append((K1,K2))
  return listKs

def plotConstants(option,ControlAlgConstName=r'$C_{MTAw}$',NCConstName=r'$C_{MTNC}$'):
  plt.rcParams['text.usetex'] = True
  myLineStyle = ['solid']*2+['dashed']*2
  for a in range(len(algorithmsLabels)):
    count = 0;
    for i_sf in range(len(suffix_file_list)):
      listk1k2 = bestK12List(dataMean0[:,0,a,i_sf,option],dataMean1[:,0,0,i_sf,option],dataMeanProb[:,:,a,i_sf,option],numRobotsValues,numPercentValues)
      tmpk1 = []
      tmpk2 = []
      for ind in range(len(listk1k2)):
        par = listk1k2[ind]
        perc = numPercentValues[ind]/100.
        tmpk1.append(par[0])
        tmpk2.append(par[1])
      print(algorithmsLabels[a],suffix_file_list[i_sf],tmpk1)
      print(algorithmsLabels[a],suffix_file_list[i_sf],tmpk2)
      subhnh = r'$h$' if suffix_file_list[i_sf] == "holo" else r'$nh$'
      plt.plot(numPercentValues,tmpk2,linestyle=myLineStyle[count],label=NCConstName+' '+subhnh) #NC
      plt.plot(numPercentValues,tmpk1,linestyle=myLineStyle[count+1],label=ControlAlgConstName+' '+subhnh) #control alg.
      count+=2
    plt.legend()
    plt.xlabel("Percent");
    plt.ylabel("Value of the constant")
    filebasename = "ConstantsP"+str(option)+algorithmsLabels[a]
    print(filebasename+".pdf generated")
    plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
    # ~ plt.show();
    plt.clf()

def estimationsFormulaLoop(option,MTName="MT"):
  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for i_sf in range(len(suffix_file_list)):
    for a in range(len(algorithmsLabels)):
      listk1k2 = bestK12List(dataMean0[:,0,a,i_sf,option],dataMean1[:,0,0,i_sf,option],dataMeanProb[:,:,a,i_sf,option],numRobotsValues,numPercentValues)
      for m in range(len(numPercentValues)):
        plt.errorbar(numRobotsValues,dataMeanProb[:,m,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCiProb[:,m,a,i_sf,option],dataMeanProb[:,m,a,i_sf,option])], label=MTName+" "+str(numPercentValues[m])+"% Exp.",marker=algorithmsSymbol[a],capsize=5);
        algEstimation = selectPlotEstimation(algorithmsLabels[a], numRobotsValues, dataMean0, a, option, S, D, 0, i_sf)
        alternativeEstimation = selectPlotEstimation(algorithmsLabels2[0], numRobotsValues, dataMean1, 0, option, S, D, 0, i_sf)
        (k1,k2) = listk1k2[m]
        Y1 = adHocEstimate(k1,k2,numPercentValues[m],alternativeEstimation,algEstimation)
        plt.plot(numRobotsValues,Y1,label="Estimation")
        print(algorithmsLabels[a]," ",algorithmsLabels2[a]," ",suffix_file_list[i_sf]," ",numPercentValues[m],"% Ks = ",round(k1,4),round(k2,4),"NRMSE=",round(NRMSE(dataMeanProb[:,m,a,i_sf,option],Y1),4))
        plt.errorbar(numRobotsValues,dataMean0[:,0,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi0[:,0,a,i_sf,option],dataMean0[:,0,a,i_sf,option])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
        plt.errorbar(numRobotsValues,dataMean1[:,0,0,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi1[:,0,0,i_sf,option],dataMean1[:,0,0,i_sf,option])], label=algorithmsLabels2[0],marker=algorithmsSymbol[0+len(algorithmsLabels)],capsize=5);
        if printValuesForTTest:
            print('#',end='')
            print(algorithmsLabels[a],suffix_file_list[i_sf],sep='-------')
            print('means'+algorithmsLabels[a]+' = ',end='')
            print(*dataMeanProb[:,a,option], sep=', ')
            print('vari'+algorithmsLabels[a]+' = ',end='')
            print(*dataVari[:,a,option], sep=', ')
        plt.legend()
        plt.xlabel("Number of robots");
        plt.ylabel(list_line_ylabel[option])
        imgDir = "plots/"+algorithmAlternativeDir+algorithmsLabels[a]+"/"+suffix_file_list[i_sf]
        filebasename = "FigureEstim"+"Perc_"+str(numPercentValues[m])+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
        # ~ print(filebasename+".pdf generated")
        plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
        # ~ plt.show();
        plt.clf()

# ~ estimationsFormulaLoop(10)
# ~ plotConstants(10)

# ~ estimationsFormulaLoop(10,"MT")
# ~ plotConstants(10,r'$C_{MTB}$')
plotConstants(10)


