import PathAdaptiveCore
import numpy as np
import matplotlib.pyplot as plt

def drawPaths(paths):
	for path in paths:
		if len(path)>0:
			data = np.array(path)
			plt.plot(data[:, 0], data[:, 1])

def feecback(a):
	#print "feedback",a.CurrentPath
	drawPaths([a.CurrentPath])

a2d = PathAdaptiveCore.Adaptive2d()
a2d.toolDiameter = 3

a2d.polyTreeNestingLimit = 1
a2d.SetProgressCallbackFn(feecback)

path0 = [[-10,-10],[110,-10], [110,110], [-10,110]]
path1 = [[0,0],[100,0], [100,100], [0,100]]
path2 = [[30,30],[70,30], [70,70], [30,70]]
path2.reverse()

path3 = [[40,40],[60,40], [60,60], [40,60]]
path4 = [[140,140],[160,140], [160,160], [140,160]]
paths = [path1,path2, path3,path4]
drawPaths([path0])


a2d.Execute(paths)

plt.show()