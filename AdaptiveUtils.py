import math
import sys
def discretize(edge, flipDirection=False):
    pts=edge.discretize(Deflection=0.01)
    if flipDirection: pts.reverse()
    return pts

def IsEqualInXYPlane(e1, e2):
    return math.sqrt((e2.x-e1.x)*(e2.x-e1.x) +
              (e2.y - e1.y) * (e2.y - e1.y))<0.01

def connectEdges(edges):
    ''' Makes the list of connected discretized paths '''
    # find edge
    lastPoint=None
    remaining = []
    pathArray = []
    combined = []
    for edge in edges:
        p1 = edge.valueAt(edge.FirstParameter)
        p2 = edge.valueAt(edge.LastParameter)
        m1 =  edge.valueAt((edge.LastParameter+edge.LastParameter)/2)
        duplicate = False
        for ex in remaining:
            exp1 = ex.valueAt(ex.FirstParameter)
            exp2 = ex.valueAt(ex.LastParameter)
            exm1 = ex.valueAt((ex.FirstParameter + ex.LastParameter)/2)
            if IsEqualInXYPlane(exp1, p1) and IsEqualInXYPlane(exp2, p2) and IsEqualInXYPlane(exm1, m1):
                duplicate = True
            if IsEqualInXYPlane(exp1, p2) and IsEqualInXYPlane(exp2, p1) and IsEqualInXYPlane(exm1, m1):
                duplicate = True
        if not duplicate:
            remaining.append(edge)

    newPath=True
    while len(remaining)>0:
        if newPath:
            edge=remaining[0]
            p1 = edge.valueAt(edge.FirstParameter)
            p2 = edge.valueAt(edge.LastParameter)
            if len(combined)>0: pathArray.append(combined)
            combined = []
            combined.append(discretize(edge))
            remaining.remove(edge)
            lastPoint=p2
            newPath=False

        anyMatch=False
        for e in remaining:
            p1 = e.valueAt(e.FirstParameter)
            p2 = e.valueAt(e.LastParameter)
            if IsEqualInXYPlane(lastPoint,p1):
                combined.append(discretize(e))
                remaining.remove(e)
                lastPoint=p2
                anyMatch=True
                break
            elif IsEqualInXYPlane(lastPoint,p2):
                combined.append(discretize(e,True))
                remaining.remove(e)
                lastPoint=p1
                anyMatch=True
                break
        if not anyMatch:
            newPath=True


    #make sure last path  is appended
    if len(combined)>0: pathArray.append(combined)
    combined = []
    return pathArray

def convertTo2d(pathArray):
    output = []
    for path in pathArray:
        pth2 = []
        for edge in path:
            for pt in edge:
                pth2.append([pt[0],pt[1]])
        output.append(pth2)
    return output

