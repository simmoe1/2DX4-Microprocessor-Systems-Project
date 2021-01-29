#Ebrahim Simmons
#simmoe1
#400200042 

import serial
import numpy as np
import open3d as o3d

s = serial.Serial("COM6", 115200)
f = open("points2dx4proj.xyz","w")

print(s.name)

#********************************************************************************************

#while loop
x = s.readline()
line = x.decode()
while line.find("End") == -1: 
    if line.find("r") == -1: 
        f.write(line) 
        print(line) 
    x = s.readline() 
    line = x.decode() 

#while loop
x = s.readline()
line = x.decode()
while line.find("End") == -1:
    if line.find("r") == -1:
        f.write(line)
        print(line)
    x = s.readline()
    line = x.decode()

f.close(); #closing input 
s.close(); #closing input 

pcd = o3d.io.read_point_cloud("points2dx4proj.xyz", format='xyz') #load points from file 
print(pcd)
print(np.asarray(pcd.points))

#********************************************************************************************

#Variables 
point1 = 0
point2 = 1
point3 = 2
point4 = 3
point5 = 4
point6 = 5
point7 = 6
point8 = 7
point9 = 8
point10 = 9
point11 = 10
point12 = 11
point13 = 12
point14 = 13
point15 = 14
point16 = 15

#plane offset 
pOffset = 0

lines = []

#********************************************************************************************

#loop connects offset points on plane 
for x in range(0,9):
    lines.append([point1+pOffset,point2+pOffset])
    lines.append([point2+pOffset,point3+pOffset])
    lines.append([point3+pOffset,point4+pOffset])
    lines.append([point4+pOffset,point5+pOffset])
    lines.append([point5+pOffset,point6+pOffset])
    lines.append([point6+pOffset,point7+pOffset])
    lines.append([point7+pOffset,point8+pOffset])
    lines.append([point8+pOffset,point1+pOffset])

#loop connects points within plane 
for x in range(0,9):
    lines.append([point9+pOffset,point10+pOffset])
    lines.append([point10+pOffset,point11+pOffset])
    lines.append([point11+pOffset,point12+pOffset])
    lines.append([point12+pOffset,point13+pOffset])
    lines.append([point13+pOffset,point14+pOffset])
    lines.append([point14+pOffset,point15+pOffset])
    lines.append([point15+pOffset,point16+pOffset])
    lines.append([point16+pOffset,point9+pOffset])
    
#********************************************************************************************

#setting vertices values 
point1 = 0
point2 = 1
point3 = 2
point4 = 3
point5 = 4
point6 = 5
point7 = 6
point8 = 7
point9 = 8
point10 = 9
point11 = 10
point12 = 11
point13 = 12
point14 = 13
point15 = 14
point16 = 15
pOffset = 0 
pl = 0 

#for loop connects points in plane
for x in range(0,9):
    lines.append([point1+pOffset,point9+pOffset])
    lines.append([point2+pOffset,point10+pOffset])
    lines.append([point3+pOffset,point11+pOffset])
    lines.append([point4+pOffset,point12+pOffset])
    lines.append([point5+pOffset,point13+pOffset])
    lines.append([point6+pOffset,point14+pOffset])
    lines.append([point7+pOffset,point15+pOffset])
    lines.append([point8+pOffset,point16+pOffset])

lines_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([lines_set])

s = serial.Serial("COM6", 115200) #opening serial input 
f = open("points2dx4proj.xyz","w")

print(s.name)

#********************************************************************************************
