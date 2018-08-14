import pygame
import sys
import time
import math
import pyclipper

pygame.init()
x_off = 1024/2 - 400
y_off = 768/2 + 200
scale = 40.0
screen=pygame.display.set_mode((1024,768))
screen.fill((255,255,255))
toolDia = 5.0
paths=[]

def transCoord(pos):
    return [x_off + int(scale*pos[0]) ,y_off - int(scale*pos[1])]

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

def circle2circleItersection(c1,c2,r):
    midpoint = [(c1[0]+c2[0])/2,(c1[1]+c2[1])/2]
    dx = 1.0*(c2[0] - c1[0])
    dy = 1.0*(c2[1] - c1[1])
    d = math.sqrt(dx*dx + dy*dy)
    a_2 = math.sqrt(4.0*r*r-d*d)/2
    pd = [-dy/d,dx/d]
    return [[midpoint[0] + pd[0]*a_2, midpoint[1] + pd[1]*a_2] ,[midpoint[0] - pd[0]*a_2,midpoint[1] - pd[1]*a_2] ]

def pointSideOfLine(p1,p2,p):
    d=(p[0] - p1[0])*(p2[1]-p1[1]) - (p[1] - p2[1])*(p2[0]-p1[0])
    return d

def LineSegToCircleIntersection(c,r,p1,p2,clamp=True):
    dx=1.0*(p2[0]-p1[0])
    dy=1.0*(p2[1]-p1[1])
    lcx = 1.0*(p1[0] - c[0])
    lcy = 1.0*(p1[1] - c[1])
    a=dx*dx+dy*dy
    b=2*dx*lcx+2*dy*lcy
    c=lcx*lcx+lcy*lcy-r*r
    sq = b*b-4*a*c
    if sq<0: return [] #no solution
    sq=math.sqrt(sq)
    t1=(-b-sq)/(2*a)
    t2=(-b+sq)/(2*a)
    res=[]
    if clamp:
        if t1>=0 and t1<=1:
            res.append([p1[0] + t1*dx,p1[1] + t1*dy])
        if t2>=0 and t2<=1:
            res.append([p1[0] + t2*dx,p1[1] + t2*dy])
    else:
        res.append([p1[0] + t1*dx,p1[1] + t1*dy])
        res.append([p1[0] + t2*dx,p1[1] + t2*dy])
    return res

def getIntersectionLL(l1,l2):
    ''' finst intesection of two line segments '''
    d = (l1[1][1]-l1[0][1])*(l2[1][0]-l2[0][0]) - \
        (l2[1][1]-l2[0][1])*(l1[1][0]-l1[0][0])
    if d == 0:  # lines are parallel
        return None
    p1d = (l2[1][1]-l2[0][1])*(l1[0][0]-l2[0][0]) - \
        (l2[1][0]-l2[0][0])*(l1[0][1]-l2[0][1])
    p2d = (l1[1][0]-l1[0][0])*(l2[0][1]-l1[0][1]) - \
        (l1[1][1]-l1[0][1])*(l2[0][0]-l1[0][0])
    #clamp
    if d < 0:
        if (p1d < d or p1d > 0):
            return None
        if (p2d < d or p2d > 0):
            return None
    else:
        if (p1d < 0 or p1d > d):
            return None
        if (p2d < 0 or p2d > d):
            return None
    return [l1[0][0] + (l1[1][0]-l1[0][0])*p1d/d, l1[0][1] + (l1[1][1]-l1[0][1])*p1d/d]
    #nothing found, return None

def area(pts):
    sum=0.0
    for i in range(0, len(pts)):
        sum += pts[i-1][0] * pts[i][1] - pts[i-1][1] * pts[i][0]
    return math.fabs(sum / 2)


def getAngle3p(p1,p2,p3):
    ''' angle between line p1,p2 and p2,p3 '''
    t1= math.atan2(p1[1]-p2[1],p1[0]-p2[0])
    t2=math.atan2(p3[1]-p2[1],p3[0]-p2[0])
    diff = math.fabs( t2 - t1 )
    return diff
    #return min(diff,math.pi-diff)

toolArea = toolDia/2*toolDia/2*math.pi

def calcArea(c1,c2,r,lp1,lp2):
    # find point p1
    rsqrd_2=r*r/2
    result =0
    p1 = None
    p2 = None
    p3 = None
    p4 = None
    p5 = None
    p6 = None
    px = None

    res=LineSegToCircleIntersection(c2,r,lp1,lp2)

    if len(res)<2: # nothing cut
        return 0

    p1 = res[1]
    p2 = res[0]

    res=LineSegToCircleIntersection(c1,r,lp1,lp2)
    if len(res) > 0:
        p3=res[1]
        p4=res[0]

    res=circle2circleItersection(c1,c2,r)
    p5 = res[1]
    p6 = res[0]


    if p3 == None: #case 1: intersecting only the new tool
        fi1=getAngle3p(p1,c2,p2)
        A = fi1*rsqrd_2
        B = math.fabs(area([c2,p1,p2]))
        result = A-B
    elif  pointSideOfLine(lp1,lp2,p6)<0: #case2 intersecting both tools, p6 on the left
        fi1=getAngle3p(p1,c2,p2)
        A = fi1*rsqrd_2 - math.fabs(area([p1,c2,p2]))

        fi2=getAngle3p(p3,c1,p4)
        B = fi2*rsqrd_2 - math.fabs(area([p3,c1,p4]))
        result =A-B
    else:   #case 3: p6 on the right
        fi1=getAngle3p(p1,c2,p2)
        A = fi1*rsqrd_2
        fi2 =getAngle3p(p3,c1,p6)
        B = fi2*rsqrd_2
        px = getIntersectionLL([c2,p6],[c1,p2])
        if px==None:
            px = getIntersectionLL([p1,c2],[c1,p2])

        C= math.fabs(area([c1,px,p6]))
        D= math.fabs(area([p1,c2,px,p2]))
        result = A-(B-C)-D
        print "A", A, " fi1", fi1
        print "B", B, " fi2", fi2
        print "C", C
        print "D",D

    pygame.draw.circle(screen, (0,0,0),transCoord(p1) , 5, 1)
    pygame.draw.circle(screen, (0,0,0),transCoord(p2) , 5, 1)
    if p3!=None:
        pygame.draw.circle(screen, (0,0,0),transCoord(p3) , 5, 1)
    if p4!=None:
        pygame.draw.circle(screen, (0,0,0),transCoord(p4) , 5, 1)

    if p5!=None:
        pygame.draw.circle(screen, (0,0,0),transCoord(p5) , 5, 1)

    if p6!=None:
        pygame.draw.circle(screen, (0,0,0),transCoord(p6) , 5, 1)

    if px!=None:
        pygame.draw.circle(screen, (255,0,0),transCoord(px) , 5, 1)

    return result

print "totalArea:", toolDia*toolDia/4*math.pi
d=-3.0
while d<3:
    clear()
    c1=[10.0,2.0]
    c2=[10.4,2.4]
    pygame.draw.circle(screen, (0,0,0),transCoord(c1) , 5, 1)
    pygame.draw.circle(screen, (0,0,0),transCoord(c2) , 5, 1)

    pygame.draw.circle(screen, (255,0,0),transCoord(c1) , int(scale*toolDia/2), 1)
    pygame.draw.circle(screen, (255,0,0),transCoord(c2) , int(scale*toolDia/2), 1)
    #inter = circle2circleItersection(c1,c2,toolDia/2)

    p1=[12+d,-1]
    p2=[3+d,30]
    drawPaths([[p1,p2]])
    #linter=LineSegToCircleIntersection(c2,toolDia/2,p1,p2)
    #pygame.draw.circle(screen, (0,0,0),transCoord(linter[1]) , 5, 1)

    # print linter
    # for p in linter:
    #     pygame.draw.circle(screen, (0,0,0),transCoord(p) , 5, 1)
    print d,calcArea(c1,c2,toolDia/2,p1,p2)
    d=d+0.01
    pygame.display.update()
    time.sleep(0.1)

# start = time.time()
# for i in range(0,1000000):
#     a=math.atan2(0,i)
# print time.time()-start

while True:
   for event in pygame.event.get():
        if event.type == pygame.QUIT:
             pygame.quit()
             sys.exit()
