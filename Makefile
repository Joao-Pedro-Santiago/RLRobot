# compilation options
CXX = g++ -Wall -fPIC 
CXXFLAGS =  `pkg-config --cflags stage` 
LINKFLAGS = `pkg-config --libs stage` 

all: coordination.so createScenario TRVFdraw.so SQFdraw.so NoCoorddraw.so

ConfigFile.o: ConfigFile.cpp ConfigFile.h
	$(CXX) -c ConfigFile.cpp -o ConfigFile.o

FinalLog.o: FinalLog.cpp FinalLog.h commonConfig.h util.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c FinalLog.cpp -o FinalLog.o -lm

createScenario: createScenario.cpp commonConfig.h ConfigFile.h ConfigFile.o
	$(CXX) ConfigFile.o createScenario.cpp -o createScenario

TargetAreaVisualizer.o: TargetAreaVisualizer.cpp TargetAreaVisualizer.h  commonConfig.h ConfigFile.h util.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c TargetAreaVisualizer.cpp -o TargetAreaVisualizer.o 

TRVFdraw.so: TRVFRegionsVisualizer.cpp  TRVFRegionsVisualizer.h TargetAreaVisualizer.h commonConfig.h  ConfigFile.h util.h TargetAreaVisualizer.o util.o ConfigFile.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o TRVFdraw.so TRVFRegionsVisualizer.cpp TargetAreaVisualizer.o util.o ConfigFile.o

SQFdraw.so: SQFRegionsVisualizer.cpp SQFRegionsVisualizer.h  TargetAreaVisualizer.h commonConfig.h ConfigFile.h util.h TargetAreaVisualizer.o util.o ConfigFile.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o SQFdraw.so SQFRegionsVisualizer.cpp TargetAreaVisualizer.o util.o ConfigFile.o

NoCoorddraw.so: NoCoordRegionsVisualizer.cpp NoCoordRegionsVisualizer.h TargetAreaVisualizer.h commonConfig.h ConfigFile.h util.h TargetAreaVisualizer.o util.o ConfigFile.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o NoCoorddraw.so NoCoordRegionsVisualizer.cpp TargetAreaVisualizer.o util.o ConfigFile.o

forcevisualizer.o: forcevisualizer.cpp forcevisualizer.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c forcevisualizer.cpp -o forcevisualizer.o

Robot.o: Robot.cpp Robot.h FinalLog.h commonConfig.h util.h ConfigFile.h commonDefs.h option.hh forcevisualizer.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c Robot.cpp -o Robot.o 

NoCoord.o: NoCoord.cpp NoCoord.h Robot.h FinalLog.h commonConfig.h util.h ConfigFile.h commonDefs.h option.hh forcevisualizer.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c NoCoord.cpp -o NoCoord.o 

TRVFparameters.o: TRVFparameters.cpp TRVFparameters.h commonConfig.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c TRVFparameters.cpp -o TRVFparameters.o

TRVF.o: TRVF.cpp TRVF.h Robot.h FinalLog.h commonConfig.h util.h  ConfigFile.h commonDefs.h option.hh forcevisualizer.h TRVFparameters.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c TRVF.cpp -o TRVF.o

SQF.o: SQF.cpp SQF.h Robot.h FinalLog.h commonConfig.h util.h ConfigFile.h commonDefs.h option.hh forcevisualizer.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c SQF.cpp -o SQF.o 

FollowNeighbour.o: FollowNeighbour.cpp FollowNeighbour.h Robot.h  FinalLog.h commonConfig.h util.h ConfigFile.h commonDefs.h option.hh  forcevisualizer.h TRVF.h TRVFparameters.h SQF.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c FollowNeighbour.cpp -o FollowNeighbour.o 

RLRobot.o: RLRobot.cpp RLRobot.h Robot.h  FinalLog.h commonConfig.h util.h ConfigFile.h commonDefs.h option.hh  forcevisualizer.h TRVF.h TRVFparameters.h SQF.h FollowNeighbour.h 
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c RLRobot.cpp -o RLRobot.o

util.o: util.cpp util.h
	$(CXX) -c util.cpp -o util.o

init.o: init.cpp init.h commonConfig.h Robot.h FinalLog.h util.h  ConfigFile.h commonDefs.h option.hh forcevisualizer.h TRVF.h TRVFparameters.h NoCoord.h SQF.h FollowNeighbour.h RLRobot.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c init.cpp -o init.o

coordination.so: init.o util.o forcevisualizer.o Robot.o TRVF.o TRVFparameters.o FinalLog.o ConfigFile.o NoCoord.o FollowNeighbour.o SQF.o RLRobot.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o coordination.so init.o forcevisualizer.o FinalLog.o Robot.o ConfigFile.o util.o TRVF.o NoCoord.o FollowNeighbour.o SQF.o TRVFparameters.o RLRobot.o

clean:
	@rm -f *.o *.so  createScenario
	@find -regextype posix-awk  -regex .*configExperiment.*.ini\|.*automatic.*\|.*experimentsLog.* -delete
