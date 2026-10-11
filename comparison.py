import sys
sys.path.insert(1, '../common/')
from cool_path_throughput import limitF, Kmax
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array

def loadFile(filename):
  input_file = open(filename, 'rb')
  float_array = array('d')
  float_array.fromstring(input_file.read())
  return float_array

def saveFile(outfilename, out):
  output_file = open(outfilename, 'wb')
  float_array = array('d', out)
  float_array.tofile(output_file)
  output_file.close()

def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue

#holo
suffix_file = 'holo'
'''
#nonholo
suffix_file = 'nonholo'
'''

algorithms = ['./'+suffix_file+'/K_3/','./'+suffix_file+'/K_4/','./'+suffix_file+'/K_5/','./'+suffix_file+'/K_6/']
algorithmsLabels = ['K=3','K=4','K=5','K=6']
algorithmsSymbol = ["."]*len(algorithmsLabels);
algorithmsLinestyle = ["-"]*(len(algorithmsLabels)-1) + ["--"];
prefixNum='n_'
nRobots = range(20, 300, 20)
nSamples = [40]*len(algorithmsLabels);
datalines = 19+1
data = np.zeros((len(nRobots),len(algorithms),max(nSamples),datalines));
dataMean = np.zeros((len(nRobots),len(algorithms),datalines));
dataVari = np.zeros((len(nRobots),len(algorithms),datalines));
dataUpCi = np.zeros((len(nRobots),len(algorithms),datalines));


for n in range(len(nRobots)):
  for a in range(len(algorithms)):
    algorithm = algorithms[a];
    for s in range(nSamples[a]):
      dataFile = open(algorithm+"/"+prefixNum+str(nRobots[n])+"/log_"+str(s));
      dataFileStr = dataFile.readlines();
      for fileline in range(datalines):
        if fileline == 19:
          FirstRobotReachingTimeline = 8 
          LastRobotReachingTimeline = 9
          data[n, a, s, fileline] = (nRobots[n]-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
        elif fileline in [11,12,13,14,16,17]:
          data[n, a, s, fileline] = float(dataFileStr[fileline]);
        elif fileline == 10:
          data[n, a, s, fileline] = int(dataFileStr[fileline])/1000000;
        else:
          data[n, a, s, fileline] = int(dataFileStr[fileline]);
    for fileline in range(datalines):
      dataMean[n, a, fileline] = np.mean(data[n,a,:nSamples[a],fileline]);
      dataVari[n, a, fileline] = np.var(data[n,a,:nSamples[a],fileline]);
      if all(data[n,a,0,fileline] == rest for rest in data[n,a,:,fileline]):
        dataUpCi[n, a, fileline] = dataMean[n, a, fileline]
      else:
        dataUpCi[n, a, fileline] = calcConfInt(dataMean[n, a, fileline],dataVari[n, a, fileline],nSamples[a]);

def testTEqual(means1, variances1, n1, means2, variances2, n2):
  '''
    means1, means2: vector
    variances1, variances2: vector
    n1,n2: integer
  '''
  l1,l2 = len(means1),len(means2)
  df = min(n1,n2) - 1
  ts = [(means1[i] - means2[i])/math.sqrt(variances1[i]/n1 + variances2[i]/n2) for i in range(min(l1,l2))]
  ps = [t.cdf(ts[i],df) for i in range(min(l1,l2))]
  return [ps[i] >= 0.05 for i in range(min(l1,l2))]

def mainLoop(fileline):
  # ~ plt.figure(figsize=(7.5,4.5));
  plt.rcParams.update({'font.size': 15})
  matplotlib.rcParams['pdf.fonttype'] = 42
  matplotlib.rcParams['ps.fonttype'] = 42


  for a in range(len(algorithms)):
    plt.errorbar(nRobots,dataMean[:,a,fileline], yerr=[m - n for m,n in zip(dataUpCi[:,a,fileline],dataMean[:,a,fileline])], label=algorithmsLabels[a],marker=algorithmsSymbol[a],capsize=5,linestyle=algorithmsLinestyle[a]);
  plt.legend(loc=0);
  plt.xlabel("Number of robots");
  list_line_ylabel = [ 
    #              Label                               # index
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
  # ~ print(list_line_ylabel[fileline])
  # ~ print(testTEqual(dataMean[:,0,fileline], dataVari[:,0,fileline], nSamples[0], dataMean[:,1,fileline], dataVari[:,1,fileline], nSamples[1]))
  plt.ylabel(list_line_ylabel[fileline])
  plt.savefig("FigureRobots"+str(fileline)+'_'+suffix_file+".png",bbox_inches="tight");
  plt.savefig("FigureRobots"+str(fileline)+'_'+suffix_file+".pdf");
  plt.clf();

# ~ mainLoop(10)
mainLoop(13)
mainLoop(14)
mainLoop(15)
mainLoop(16)
mainLoop(17)
mainLoop(18)
# ~ mainLoop(19)
