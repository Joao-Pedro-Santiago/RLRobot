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


'''Directories of new experiments'''
algorithmDirectoriesNonHolo2 = {
  "TRVF": "original root/nonholo/TRVF/s3/n_",
  "SQF": "neighbourAngle zero experiments/nonholo/SQF/s3/n_",
  "NC": "neighbourAngle zero experiments/nonholo/NoCoord/s3/n_",
}

algorithmDirectoriesHolo2 = {
  "TRVF": "original root/holo/TRVF/s3/n_",
  "SQF": "neighbourAngle zero experiments/holo/SQF/s3/n_",
  "NC": "neighbourAngle zero experiments/holo/NoCoord/s3/n_",
}



def dictSlice(d,l):
  return {k:d[k] for k in l}

def dictHoloNonHolo(algorithmsLabelsList,aDh,aDnh):
  return {
    'nonholo': dictSlice(aDnh,algorithmsLabelsList),
    'holo': dictSlice(aDh,algorithmsLabelsList),
  }

def readStatistics(algorithmsLabels,algDirPrefixes,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots,numPercentValues=[0],prefix_dir="./",ignoreNotFound=False):
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
              algorithmDir = prefix_dir+suffix_file+"/"+algorithmAlternativeDir[a]+algorithmsLabels[a]+"/"+prefixNumRobots;
              numFollowing = int(numPercentValues[m]*numRobotsValues[n]/100)
              logLocation = algorithmDir+str(numRobotsValues[n])+"/"+prefixNumFollowingRobots+str(numFollowing)+"/log_"+str(s)
            if ignoreNotFound:
              try:
                dataFile = open(logLocation);
              except:
                continue
            else:
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

def doTheFitting(a,i_sf,option,dataMean0,numRobotsValues,exponentialFit = False):
  maxFit = len(numRobotsValues)//2
  if exponentialFit:
    # two-degree polynomial fit
    mymodel0 = np.poly1d(np.polyfit(numRobotsValues[:maxFit],dataMean0[:maxFit,0,a,i_sf,option],2))
    mymodel0values = mymodel0(numRobotsValues)
  else:
    # exponential fit
    logmodel0 = np.poly1d(np.polyfit(numRobotsValues[:maxFit],np.log(dataMean0[:maxFit,0,a,i_sf,option]),1))
    mymodel0values = np.exp(logmodel0(numRobotsValues))
  return mymodel0values

def plotAndListOutliers(prefixDir,option,suffix_file_list,i_sf,algorithmAlternativeDir,algorithmsLabels,a,prefixNumRobots,numPercentValues,m,numRobotsValues,dataProb2,prefixNumFollowingRobots,nSamples,checkTimeLimit=False):
  algorithmDir = prefixDir+suffix_file_list[i_sf]+"/"+algorithmAlternativeDir[a]+algorithmsLabels[a]+"/"+prefixNumRobots;
  timeLimit,epsTimeLimit = 36000,10
  for n in range(len(numRobotsValues)):
    numFollowing = int(numPercentValues[m]*numRobotsValues[n]/100)
    s = np.argmax(dataProb2[n,m,a,:,i_sf,option])
    logLocation = algorithmDir+str(numRobotsValues[n])+"/"+prefixNumFollowingRobots+str(numFollowing)+"/log_"+str(s)
    print(logLocation)
    if checkTimeLimit:
      if dataProb2[n,m,a,s,i_sf,option] >= timeLimit-epsTimeLimit:
        print(logLocation)
    
  for s in range(nSamples):
    plt.plot(numRobotsValues,dataProb2[:,m,a,s,i_sf,option],color='orange',marker='x',linestyle="None");
    for i in range(len(numRobotsValues)):
      plt.annotate(s, (numRobotsValues[i], dataProb2[i,m,a,s,i_sf,option]))
  
  if checkTimeLimit:
    plt.axhline(y=timeLimit,linestyle = '--')

plt.rcParams.update({'font.size': 20})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


def mainLoop(option,doFitting = False,MTName="MT"):
  '''Plot the results based only on the directories given.'''
  '''
  Short description of the directories:
  
  if percent to compare is 0, the location is
    algDirPrefixes[suffix_file][algorithm label] + a num of Robots + "/log_"+sample
  else
    "./"+suffix_file+"/"+algorithmAlternativeDir[algorithm number]+algorithmsLabels[algorithm number]+"/"+prefixNumRobots;
  '''
  
  # Test for NoCoord as alternative and SQF or TRVF by the knowing robots.
  algorithmAlternativeDir=["NoCoordAlt/","NoCoordAlt/"]
  algorithmsLabels = ["SQF","TRVF"];
  algorithmsLabels2 = ["NC","NC"]
  prefixImgFile = "NoCoordalt"
  
  # ~ # Same as above, but one element per array.
  # ~ algorithmAlternativeDir=["NoCoordAlt/"]
  # ~ algorithmsLabels = ["SQF"];
  # ~ algorithmsLabels = ["TRVF"];
  # ~ algorithmsLabels2 = ["NC"]
  
  # ~ # Test for TRVF as alternative and SQF by the knowing robots.
  # ~ algorithmAlternativeDir=["TRVFAlt/","TRVFAlt/"]
  # ~ algorithmsLabels = ["SQF"];
  # ~ algorithmsLabels2 = ["TRVF","NC"]
  # ~ prefixImgFile = "TRVFaltSQFknowing"
  
  # ~ # Test for SQF as alternative and TRVF by the knowing robots.
  # ~ algorithmAlternativeDir=["SQFAlt/","SQFAlt/"]
  # ~ algorithmsLabels = ["TRVF"];
  # ~ algorithmsLabels2 = ["SQF","NC"]
  # ~ prefixImgFile = "SQFaltTRVFknowing"
  
  suffix_file_list = ['nonholo','holo']
  
  algDirPrefixes1_1 = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  algDirPrefixes2_1 = dictHoloNonHolo(algorithmsLabels2,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  
  # ~ algDirPrefixes = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo,algorithmDirectoriesNonHolo)
  # ~ algDirPrefixes2 = dictHoloNonHolo(algorithmsLabels2,=algorithmDirectoriesHolo,algorithmDirectoriesNonHolo)
  prefixNumRobots = "robots_"
  prefixNumFollowingRobots = "m_"
  
  algorithmsSymbol = ["."]*(len(algorithmsLabels)+len(algorithmsLabels2));
  numPercentValues = range(10, 91, 10)
  numRobotsValues = range(20, 301, 20)
  nSamples = 40

  datalines = 19+1
  printValuesForTTest = False # Set True if one wishes to print values for t-test.
  
  dataMeanProb2,dataUpCiProb2,dataProb2,dataVari2 = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  dataMean0_1,dataUpCi0_1,data0_1,_ = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots) 
  dataMean1_1,dataUpCi1_1,data1_1,_ = readStatistics(algorithmsLabels2,algDirPrefixes2_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots) 
  
  # ~ dataMeanProb,dataUpCiProb,dataProb,_ = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  
  
  
  if printValuesForTTest:
    print('#==== '+list_line_ylabel[option]+' ====')
  for i_sf in range(len(suffix_file_list)):
    for a in range(len(algorithmsLabels)):
      for m in range(len(numPercentValues)):
        # ~ plt.errorbar(numRobotsValues,dataMeanProb[:,m,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCiProb[:,m,a,i_sf,option],dataMeanProb[:,m,a,i_sf,option])], label="Ad hoc "+str(numPercentValues[m])+"% old",marker=algorithmsSymbol[a],capsize=5);
        if doFitting:
          mymodel0values = doTheFitting(a,i_sf,option,dataMean0_1,numRobotsValues)
          plt.plot(numRobotsValues,mymodel0values,label=algorithmsLabels[a]+" reg")
        
        plt.errorbar(numRobotsValues,dataMean0_1[:,0,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi0_1[:,0,a,i_sf,option],dataMean0_1[:,0,a,i_sf,option])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
        plt.errorbar(numRobotsValues,dataMean1_1[:,0,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi1_1[:,0,a,i_sf,option],dataMean1_1[:,0,a,i_sf,option])], label=algorithmsLabels2[a],marker=algorithmsSymbol[a+len(algorithmsLabels)],capsize=5);
        plt.errorbar(numRobotsValues,dataMeanProb2[:,m,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCiProb2[:,m,a,i_sf,option],dataMeanProb2[:,m,a,i_sf,option])], label=MTName+" "+str(numPercentValues[m])+"%",marker=algorithmsSymbol[a],capsize=5);
        
        # ~ plotAndListOutliers("neighbourAngle zero experiments/",option,suffix_file_list,i_sf,algorithmAlternativeDir,algorithmsLabels,a,prefixNumRobots,numPercentValues,m,numRobotsValues,dataProb2,prefixNumFollowingRobots,nSamples)
        # ~ print('----------------')
        
        if doFitting:
          plt.plot(numRobotsValues,((100-numPercentValues[m])/100.)*mymodel0values+(numPercentValues[m]/100.)*mymodel0values,label="possible")
        if printValuesForTTest:
            print('#',end='')
            print(algorithmsLabels[a],suffix_file_list[i_sf],sep='-------')
            print('means'+algorithmsLabels[a]+' = ',end='')
            print(*dataMeanProb2[:,a,option], sep=', ')
            print('vari'+algorithmsLabels[a]+' = ',end='')
            print(*dataVari2[:,a,option], sep=', ')
        plt.legend()
        plt.xlabel("Number of robots");
        plt.ylabel(list_line_ylabel[option])
        filebasename = prefixImgFile+"FigNoEstim"+"Perc_"+str(numPercentValues[m])+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
        print(filebasename+".pdf generated")
        plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
        # ~ plt.show();
        plt.clf()

def sumOverRobots(option,MTName="MT"):
  
  # Test for NoCoord as alternative and SQF or TRVF by the knowing robots.
  algorithmAlternativeDir=["NoCoordAlt/","NoCoordAlt/"]
  algorithmsLabels = ["SQF","TRVF"];
  algorithmsLabels2 = ["NC","NC"]
  prefixImgFile = "NoCoordalt"
  
  
  # ~ # Same as above, but one element per array.
  # ~ algorithmAlternativeDir=["NoCoordAlt/"]
  # ~ algorithmsLabels = ["SQF"];
  # ~ algorithmsLabels = ["TRVF"];
  # ~ algorithmsLabels2 = ["NC"]
  
  # ~ # Test for TRVF as alternative and SQF by the knowing robots.
  # ~ algorithmAlternativeDir=["TRVFAlt/","TRVFAlt/"]
  # ~ algorithmsLabels = ["SQF"];
  # ~ algorithmsLabels2 = ["TRVF","NC"]
  # ~ prefixImgFile = "TRVFaltSQFknowing"
  
  # ~ # Test for SQF as alternative and TRVF by the knowing robots.
  # ~ algorithmAlternativeDir=["SQFAlt/","SQFAlt/"]
  # ~ algorithmsLabels = ["TRVF"];
  # ~ algorithmsLabels2 = ["SQF","NC"]
  # ~ prefixImgFile = "SQFaltTRVFknowing"
  
  suffix_file_list = ['nonholo','holo']
  
  algDirPrefixes1_1 = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  algDirPrefixes2_1 = dictHoloNonHolo(algorithmsLabels2,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  
  # ~ algDirPrefixes = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo,algorithmDirectoriesNonHolo)
  # ~ algDirPrefixes2 = dictHoloNonHolo(algorithmsLabels2,=algorithmDirectoriesHolo,algorithmDirectoriesNonHolo)
  prefixNumRobots = "robots_"
  prefixNumFollowingRobots = "m_"
  
  algorithmsSymbol = ["."]*(len(algorithmsLabels)+len(algorithmsLabels2));
  numPercentValues = range(10, 91, 10)
  numRobotsValues = range(20, 301, 20)
  nSamples = 40

  datalines = 19+1
  
  dataMeanProb2,dataUpCiProb2,dataProb2,_ = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  dataMean0_1,dataUpCi0_1,data0_1,_ = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots) 
  dataMean1_1,dataUpCi1_1,data1_1,_ = readStatistics(algorithmsLabels2,algDirPrefixes2_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots) 
  
  for i_sf in range(len(suffix_file_list)):
    for a in range(len(algorithmsLabels)):
        sumData0_1 = [sum(dataMean0_1[:,0,a,i_sf,option])]*len(numPercentValues)
        sumUpCi0_1 = [sum(dataUpCi0_1[:,0,a,i_sf,option])]*len(numPercentValues)
        sumData1_1 = [sum(dataMean1_1[:,0,a,i_sf,option])]*len(numPercentValues)
        sumUpCi1_1 = [sum(dataUpCi1_1[:,0,a,i_sf,option])]*len(numPercentValues)
        sumMeanProb2 = [sum(dataMeanProb2[:,m,a,i_sf,option]) for m in range(len(numPercentValues))]
        sumUpCiProb2 = [sum(dataUpCiProb2[:,m,a,i_sf,option]) for m in range(len(numPercentValues))]
        
        plt.errorbar(numPercentValues,sumData0_1, yerr=[m1 - n1 for m1,n1 in zip(sumUpCi0_1,sumData0_1)], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
        plt.errorbar(numPercentValues,sumData1_1, yerr=[m1 - n1 for m1,n1 in zip(sumUpCi1_1,sumData1_1)], label=algorithmsLabels2[a],marker=algorithmsSymbol[a+len(algorithmsLabels)],capsize=5);
        plt.errorbar(numPercentValues,sumMeanProb2, yerr=[m1 - n1 for m1,n1 in zip(sumUpCiProb2,sumMeanProb2)], label=MTName,marker=algorithmsSymbol[a],capsize=5);
        
        plt.legend()
        plt.xlabel("Percentage");
        plt.ylabel("Sum of total time (s)")
        filebasename = prefixImgFile+"Sum"+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
        print(filebasename+".pdf generated")
        plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
        # ~ plt.show();
        plt.clf()

def sumOverRobotsForDifferentAltAlg(option):
  # ~ # Test for TRVF as alternative and SQF by the knowing robots.
  # ~ algorithmAlternativeDir=["NoCoordAlt/","TRVFAlt/"]
  # ~ algorithmsLabels = ["SQF","SQF"];
  # ~ prefixImgFile = "SumTRVFaltNoCoordAltSQFknowing"
  
  # Test for SQF as alternative and TRVF by the knowing robots.
  algorithmAlternativeDir=["NoCoordAlt/","SQFAlt/"]
  algorithmsLabels = ["TRVF","TRVF"];
  prefixImgFile = "SumSQFaltNoCoordAltTRVFknowing"
  
  suffix_file_list = ['nonholo','holo']
  
  algDirPrefixes1_1 = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)

  prefixNumRobots = "robots_"
  prefixNumFollowingRobots = "m_"
  
  algorithmsSymbol = ["."]*len(algorithmsLabels);
  numPercentValues = range(10, 91, 10)
  numRobotsValues = range(20, 301, 20)
  nSamples = 40

  datalines = 19+1
  printValuesForTTest = False # Set True if one wishes to print values for t-test.
  
  dataMeanProb2,dataUpCiProb2,dataProb2,_ = readStatistics(algorithmsLabels,algDirPrefixes1_1,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  
  if printValuesForTTest:
    print('#==== '+list_line_ylabel[option]+' ====')
  for i_sf in range(len(suffix_file_list)):
    if printValuesForTTest:
      argsStr = ""
    for a in range(len(algorithmsLabels)):
      sumMeanProb2 = [sum(dataMeanProb2[:,m,a,i_sf,option]) for m in range(len(numPercentValues))]
      sumUpCiProb2 = [sum(dataUpCiProb2[:,m,a,i_sf,option]) for m in range(len(numPercentValues))]
      
      tmpAlg = algorithmAlternativeDir[a].replace("Alt/","")
      if tmpAlg == "NoCoord": tmpAlg = "NC" 
      plt.errorbar(numPercentValues,sumMeanProb2, yerr=[m1 - n1 for m1,n1 in zip(sumUpCiProb2,sumMeanProb2)], label="Alg="+tmpAlg,marker=algorithmsSymbol[a],capsize=5);
      if printValuesForTTest:
        print('#',end='')
        print(algorithmsLabels[a],suffix_file_list[i_sf],sep='-------')
        print('means'+str(a)+' = ',end='')
        print(*sumMeanProb2, sep=', ')
        print('vari'+str(a)+' = ',end='')
        print(*sumUpCiProb2, sep=', ')
        argsStr = argsStr + "means"+str(a)+",vari"+str(a)+","
    if printValuesForTTest:
      textResult =  prefixImgFile+" "+algorithmsLabels[a]+suffix_file_list[i_sf]
      print("printResult("+argsStr+"n1,n2,\""+textResult+"\")")
    plt.legend()
    plt.xlabel("Percentage");
    plt.ylabel("Sum of total time (s)")
    filebasename = prefixImgFile+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
    print(filebasename+".pdf generated")
    plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
    # ~ plt.show();
    plt.clf()

def plotAllAlternatives(option):
  '''Plot a comparison between the alternative algorithms.'''
  '''
  Short description of the directories:
  
  if percent to compare is 0, the location is
    algDirPrefixes[suffix_file][algorithm label] + a num of Robots + "/log_"+sample
  else
    "./"+suffix_file+"/"+algorithmAlternativeDir[algorithm number]+algorithmsLabels[algorithm number]+"/"+prefixNumRobots;
  '''
  
  # ~ algorithmAlternativeDir=["NoCoordAlt/","TRVFAlt/"]
  # ~ algorithmsLabels = ["SQF"];
  # ~ algorithmsLabels2 = ["NC","TRVF"]
  # ~ prefixImgFile = "TRVFaltSQFknowing"

  algorithmAlternativeDir=["NoCoordAlt/","SQFAlt/"]
  algorithmsLabels = ["TRVF"];
  algorithmsLabels2 = ["NC","SQF"]
  prefixImgFile = "SQFaltTRVFknowing"
  
  suffix_file_list = ['nonholo','holo']
  algDirPrefixes = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  algDirPrefixes2 = dictHoloNonHolo(algorithmsLabels2,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  prefixNumRobots = "robots_"
  prefixNumFollowingRobots = "m_"
  
  algorithmsSymbol = ["."]*(len(algorithmsLabels)+len(algorithmsLabels2));
  numPercentValues = range(10, 91, 10)
  numRobotsValues = range(20, 301, 20)
  nSamples = 40
  datalines = 19+1
  printValuesForTTest = False # Set True if one wishes to print values for t-test.

  
  dataMeanProb = [None]*len(algorithmAlternativeDir)
  dataUpCiProb = [None]*len(algorithmAlternativeDir)
  dataVariProb = [None]*len(algorithmAlternativeDir)
  for b in range(len(algorithmAlternativeDir)):
    dataMeanProb[b],dataUpCiProb[b],_,dataVariProb[b] = readStatistics(algorithmsLabels,algDirPrefixes,numRobotsValues,suffix_file_list,datalines,nSamples,[algorithmAlternativeDir[b]]*len(algorithmsLabels),prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  # ~ dataMean0,dataUpCi0,_,_ = readStatistics(algorithmsLabels,algDirPrefixes,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots) 
  # ~ dataMean1,dataUpCi1,_,_ = readStatistics(algorithmsLabels2,algDirPrefixes2,numRobotsValues,suffix_file_list,datalines,nSamples,algorithmAlternativeDir,prefixNumRobots,prefixNumFollowingRobots)

  if printValuesForTTest:
    print('#==== '+list_line_ylabel[option]+' ====')
  for a in range(len(algorithmsLabels)):
    for m in range(len(numPercentValues)):
      for i_sf in range(len(suffix_file_list)):
        for b in range(len(algorithmsLabels2)):
          plt.errorbar(numRobotsValues,dataMeanProb[b][:,m,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCiProb[b][:,m,a,i_sf,option],dataMeanProb[b][:,m,a,i_sf,option])], label="Alg="+algorithmsLabels2[b],marker=algorithmsSymbol[a],capsize=5);
        if printValuesForTTest:
          print('#',end='')
          print(numPercentValues[m],algorithmsLabels2,suffix_file_list[i_sf],sep='-------')
          argsStr = ""
          for b in range(len(algorithmsLabels2)):
            algbname = "Noc" if algorithmsLabels2[b] == "NC" else algorithmsLabels2[b]
            print('means'+algbname+' = ',end='')
            print(*dataMeanProb[b][:,m,a,i_sf,option], sep=', ')
            print('vari'+algbname+' = ',end='')
            print(*dataVariProb[b][:,m,a,i_sf,option], sep=', ')
            argsStr = argsStr + "means"+algbname+",vari"+algbname+","
          textResult = str(numPercentValues[m])+' '+algorithmsLabels[a]+" knowing "+suffix_file_list[i_sf]
          print("printResult("+argsStr+"n1,n2,\""+textResult+"\")")
        # ~ for b in range(len(algorithmsLabels2)):
          # ~ plt.errorbar(numRobotsValues,dataMean1[:,0,b,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi1[:,0,b,i_sf,option],dataMean1[:,0,b,i_sf,option])], label=algorithmsLabels2[b],marker=algorithmsSymbol[b+len(algorithmsLabels)],capsize=5);
        # ~ plt.errorbar(numRobotsValues,dataMean0[:,0,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCi0[:,0,a,i_sf,option],dataMean0[:,0,a,i_sf,option])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5);
        plt.legend()
        plt.xlabel("Number of robots");
        plt.ylabel(list_line_ylabel[option])
        filebasename = prefixImgFile+"FigAllPerc"+str(numPercentValues[m])+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
        print(filebasename+".pdf generated")
        plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
        # ~ plt.savefig(filebasename+".png",bbox_inches="tight",pad_inches=0.00);
        # ~ plt.show();
        plt.clf()

def comparingSameAlternatives(option):
  
  algorithmAlternativeDir=["NoCoordAlt/","TRVFAlt/"]
  algorithmsLabels = ["SQF"];
  algorithmsLabels2 = ["NC","TRVF"]
  prefixImgFile = "TRVFaltSQFknowing"

  # ~ algorithmAlternativeDir=["NoCoordAlt/","SQFAlt/"]
  # ~ algorithmsLabels = ["TRVF"];
  # ~ algorithmsLabels2 = ["NC","SQF"]
  # ~ prefixImgFile = "SQFaltTRVFknowing"
  
  suffix_file_list = ['nonholo','holo']
  algDirPrefixes = dictHoloNonHolo(algorithmsLabels,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  algDirPrefixes2 = dictHoloNonHolo(algorithmsLabels2,algorithmDirectoriesHolo2,algorithmDirectoriesNonHolo2)
  prefixNumRobots = "robots_"
  prefixNumFollowingRobots = "m_"
  
  algorithmsSymbol = ["."]*(len(algorithmsLabels)+len(algorithmsLabels2));
  numPercentValues = range(10, 91, 10)
  numRobotsValues = range(20, 301, 20)
  nSamples = 40
  datalines = 19+1
  
  dataMeanProb = [None]*len(algorithmAlternativeDir)
  dataUpCiProb = [None]*len(algorithmAlternativeDir)
  for b in range(len(algorithmAlternativeDir)):
    dataMeanProb[b],dataUpCiProb[b],_,_ = readStatistics(algorithmsLabels,algDirPrefixes,numRobotsValues,suffix_file_list,datalines,nSamples,[algorithmAlternativeDir[b]]*len(algorithmsLabels),prefixNumRobots,prefixNumFollowingRobots,numPercentValues,"neighbourAngle zero experiments/") 
  

  for a in range(len(algorithmsLabels)):
    for m in range(len(numPercentValues)):
      for i_sf in range(len(suffix_file_list)):
        for b in range(len(algorithmsLabels2)):
          if b == 1: plt.errorbar(numRobotsValues,dataMeanProb[b][:,m,a,i_sf,option], yerr=[m1 - n1 for m1,n1 in zip(dataUpCiProb[b][:,m,a,i_sf,option],dataMeanProb[b][:,m,a,i_sf,option])], label="AHF "+algorithmsLabels2[b]+" "+str(numPercentValues[m])+"%"+" "+suffix_file_list[i_sf],marker=algorithmsSymbol[a],capsize=5);
        plt.legend()
        plt.xlabel("Number of robots");
        plt.ylabel(list_line_ylabel[option])
      filebasename = prefixImgFile+"FigAllPerc"+str(numPercentValues[m])+"_"+str(option)+algorithmsLabels[a]+suffix_file_list[i_sf]
      print(filebasename+".pdf generated")
      # ~ plt.savefig(filebasename+".pdf",bbox_inches="tight",pad_inches=0.00);
      plt.show();
      plt.clf()

# ~ mainLoop(10,False,"MT")
# ~ mainLoop(10)
# ~ sumOverRobots(10)
# ~ sumOverRobotsForDifferentAltAlg(10)
plotAllAlternatives(10)
# ~ comparingSameAlternatives(10)
# ~ mainLoop(19)

