import matplotlib.pyplot as plt 


xfile='vi.txt'
yfile='vo.txt'

myfile = open(xfile,'r')
xvalues = list(map(float,myfile.read().splitlines()))

myfile = open(yfile,'r')
yvalues = list(map(float,myfile.read().splitlines()))

plt.plot(xvalues,yvalues,label="I/O")
plt.plot(xvalues,xvalues,label="x=y")
plt.legend()
plt.show()
