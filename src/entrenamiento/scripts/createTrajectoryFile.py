#!/usr/bin/env python
import math 
import sys

#paths
worldTemplateFile = sys.argv[1]
newWorldFile = sys.argv[2]

with open(worldTemplateFile, 'r') as file :
  filedata = file.read()

#Tiempo de duracion de la trayectoria
trajectoryTime = 30
#Radio de la trayectoria
radius = 0.55
#Altura del objeto
objectHeight = 0.1



verticalSteps = 8
horizontalSteps = 8
steps = (verticalSteps+1)*(horizontalSteps+1)
thetaStep = 2*math.pi/horizontalSteps
timeStep = trajectoryTime/float(steps)
zStep = radius/steps
pitchStep = 1.57/steps
radiusStep = radius/steps
trayectory = ""

x = 0
y = 0
z = objectHeight
roll = 0.0
pitch = 0.0
yaw = 0.0

for i in range(verticalSteps+1):
	trayectory = trayectory+"<trajectory id=\""+str(i)+"\" type=\"circle\">\n"
	theta = 0
	for j in range(horizontalSteps+1):
		time = timeStep*(j)
		theta = theta+thetaStep
		x = math.cos(theta)*radius
		y = math.sin(theta)*radius
		yaw = math.pi+theta
		z = z+zStep
		pitch = pitch+pitchStep
		trayectory = trayectory+"\t<waypoint>\n"
		trayectory = trayectory+"\t\t<time>"+str(time)+"</time>\n"
		trayectory = trayectory+"\t\t<pose>"+str(x)+" "+str(y)+" "+str(z)+" "+str(roll)+" "+str(pitch)+" "+str(yaw)+"</pose>\n"
		trayectory = trayectory+"\t</waypoint>\n"
		radius = radius-radiusStep
		if(radius < 0):
			break
	trayectory = trayectory+"</trajectory>\n"
filedata = filedata.replace('<my_trayectory>', trayectory)

f = open(newWorldFile, "w")
f.write(filedata)
f.close()