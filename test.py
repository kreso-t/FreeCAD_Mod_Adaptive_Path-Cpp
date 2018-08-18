import PathAdaptiveCore
import pygame
import sys
import time
import json
pygame.init()
# x_off = 1024/2 - 200
# y_off = 768/2 + 200
# scale = 4.0
# center_x = 17.58
# center_y = 79.91

# center_x = 39.58
# center_y = 72.55

center_x = 50
center_y = 50


screen_x=1024
screen_y=768
scale = 5

screen=pygame.display.set_mode((screen_x,screen_y))
screen.fill((255,255,255))
toolDia = 10
paths=[]

def transCoord(pos):
    return [screen_x/2 + int(scale*(pos[0]-center_x)) ,screen_y/2 - int(scale*(pos[1]-center_y))]

def drawPaths(paths, closed = False):
    for path in paths:
        if len(path)>0:
            pts = []
            for p in path:
                pts.append(transCoord(p))
            pygame.draw.lines(screen,(255,0,0),closed,pts,1)

def drawToolPos(pos, direction, color):
    tp1 = transCoord(pos)
    pygame.draw.circle(screen, color,tp1 , int(scale*toolDia/2), 2)
    tp2 = transCoord([pos[0] + direction[0] * 500,pos[1] + direction[1] * 500])
    pygame.draw.lines(screen,color,False,[tp1,tp2],1)

count = 0

def clear():
    screen.fill((255,255,255))
    drawPaths(paths)

def doEvents():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

def feecback(paths):
    global count
    for path in paths:
        drawPath(path[1],3)
        #print "feedback",dir(path)
    pygame.display.update()
    doEvents()
    count = count +1
    # if count > 10:
    #     clear()
    #     count =0

    #drawPaths([a.CurrentPath])
   # drawToolPos(a.EngagePos,a.EngageDir,(255,0,0))
   # drawToolPos(a.ToolPos,a.ToolDir,(255,0,255))
    #pygame.display.update()
    #doEvents()
    ##time.sleep(0.001)

def getColor(color):
    color = color % 6
    if color==0: return (0,0,0)
    if color==1: return (255,0,0)
    if color==2: return (0,255,0)
    if color==3: return (0,0,255)
    if color==4: return (255,0,255)
    if color==5: return (255,255,0)
    return (0,0,0)

def drawCircle(x,y, radius, color):
    pygame.draw.circle(screen, getColor(color),transCoord([x,y]) ,int(radius*scale)+1, 1)
    pygame.display.update()
    if(color>=20):time.sleep(1)
    doEvents()

def drawPath(path, color):
    pts = []
    width = 1
    if len(path) < 2: return
    if color>10: width = 3
    for p in path:
        pts.append(transCoord(p))
    pygame.draw.lines(screen,getColor(color),False,pts,width)

def drawPathFn(path, color):
    drawPath(path, color)
    pygame.display.update()
    if color>20: time.sleep(2)

a2d = PathAdaptiveCore.Adaptive2d()
a2d.stepOverFactor=0.25
a2d.tolerance = 0.1
a2d.helixRampDiameter = 0.1

# a2d.DrawCircleFn = drawCircle
# a2d.ClearScreenFn = clear
# a2d.DrawPathFn = drawPathFn
toolDia = 5
a2d.toolDiameter = toolDia


#path0 = [[-10,-10],[110,-10], [110,110], [-10,110]]
path1 = [[0,0],[90,0], [90,100], [0,100]]
path2 = [[15,70],[25,70], [25,88], [15,88]]
path3 = [[75,70],[85,70], [85,80], [75,80]]
path4 = [[15,15],[25,15], [25,25], [15,25]]
path5 = [[55,15],[75,15], [75,35], [55,35]]
path2.reverse()
path3.reverse()
path4.reverse()
path5.reverse()


#path3 = [[40,40],[60,40], [60,60], [40,60]]
#path4 = [[140,140],[160,140], [160,160], [140,160]]
paths.append(path1)
paths.append(path2)
paths.append(path3)
paths.append(path4)
paths.append(path5)
clear()
pygame.display.update()

a2d.polyTreeNestingLimit = 0
a2d.opType =  PathAdaptiveCore.OperationType.Clearing;
result=a2d.Execute(paths,feecback)
clear()
for output in result:
    for pth in output.AdaptivePaths:
       if pth[0] == PathAdaptiveCore.MotionType.Cutting:
            drawPath(pth[1],3)
       elif pth[0] == PathAdaptiveCore.MotionType.LinkClear:
            drawPath(pth[1],2)
       elif pth[0] == PathAdaptiveCore.MotionType.LinkNotClear:
            drawPath(pth[1],1)
       elif pth[0] == PathAdaptiveCore.MotionType.LinkClearAtPrevPass:
            drawPath(pth[1],4)

# for output in result:
#     i=0
#     print result
#     for pth in output.AdaptivePaths:
#         print pth,output.AdaptivePaths[i]#, len(pth.Points), pth.MType, len(output.AdaptivePaths[i].Points),output.AdaptivePaths[i].MType
#         i=i+1
#         if i>5: break
# for output in result:
#     i=0
#     print result
#     for pth in output.AdaptivePaths:
#         print pth,output.AdaptivePaths[i]#, len(pth.Points), pth.MType, len(output.AdaptivePaths[i].Points),output.AdaptivePaths[i].MType
#         i=i+1
#         if i>5: break

pygame.display.update()
#print result[0].HelixCenterPoint

while True:
    doEvents()


   # if pth.MType == PathAdaptiveCore.MotionType.Cutting:
        #     drawPath(pth.Points,3)
        # elif pth.MType == PathAdaptiveCore.MotionType.LinkClear:
        #     drawPath(pth.Points,2)
        # elif pth.MType == PathAdaptiveCore.MotionType.LinkNotClear:
        #     drawPath(pth.Points,1)
        # elif pth.MType == PathAdaptiveCore.MotionType.LinkClearAtPrevPass:
        #     drawPath(pth.Points,4)