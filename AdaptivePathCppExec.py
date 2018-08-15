import AdaptiveUtils

import PathScripts.PathOp as PathOp
import Path
import FreeCAD
import FreeCADGui
import time
from FreeCAD import Console
import json
import math
import PathAdaptiveCore
from pivy import coin

sceneGraph = None
scenePathNodes = [] #for scene cleanup aftewards
topZ = 10

def sceneDrawPath(path, color=(0, 0, 1)):
    global sceneGraph
    global scenePathNodes
    coPoint = coin.SoCoordinate3()
    pts = []
    for pt in path:
        pts.append([pt[0], pt[1], topZ])

    coPoint.point.setValues(0, len(pts), pts)
    ma = coin.SoBaseColor()
    ma.rgb = color
    li = coin.SoLineSet()
    li.numVertices.setValue(len(pts))
    pathNode = coin.SoSeparator()
    pathNode.addChild(coPoint)
    pathNode.addChild(ma)
    pathNode.addChild(li)
    sceneGraph.addChild(pathNode)
    scenePathNodes.append(pathNode) #for scene cleanup afterwards

def sceneClean():
    for n in scenePathNodes:
        sceneGraph.removeChild(n)




def GenerateGCode(op,obj,adaptiveResults, helixDiameter):
    if len(adaptiveResults)==0 or len(adaptiveResults[0].AdaptivePaths)==0:
      return

    minLiftDistance = op.tool.Diameter
    helixRadius = helixDiameter/2.0
    stepDown = obj.StepDown.Value
    passStartDepth=obj.StartDepth.Value
    if stepDown<0.1 : stepDown=0.1
    length = 2*math.pi * helixRadius
    if obj.HelixAngle<1: obj.HelixAngle=1
    helixAngleRad = math.pi * obj.HelixAngle/180.0
    depthPerOneCircle=length * math.tan(helixAngleRad)
    stepUp =  obj.LiftDistance.Value
    if stepUp<0:
        stepUp=0


    lx=adaptiveResults[0].HelixCenterPoint[0]
    ly=adaptiveResults[0].HelixCenterPoint[1]

    #print "Helix step down per full circle:" , depthPerOneCircle, " (len:" , length , ")"

    # for region in adaptiveResults:
    #     for pth in region.AdaptivePaths:
    #         print pth.Points
    step=0
    while passStartDepth>obj.FinalDepth.Value and step<1000:
        step=step+1
        passEndDepth=passStartDepth-stepDown
        if passEndDepth<obj.FinalDepth.Value: passEndDepth=obj.FinalDepth.Value

        for region in adaptiveResults:
            startAngle = math.atan2(region.StartPoint[1] - region.HelixCenterPoint[1], region.StartPoint[0] - region.HelixCenterPoint[0])

            lx=region.HelixCenterPoint[0]
            ly=region.HelixCenterPoint[1]

            r = helixRadius - 0.01
            #spiral ramp
            passDepth = (passStartDepth - passEndDepth)
            maxfi =  passDepth / depthPerOneCircle *  2 * math.pi   #- math.pi/8
            fi = 0
            offsetFi =-maxfi + startAngle-math.pi/16

            helixStart = [region.HelixCenterPoint[0] + r * math.cos(offsetFi), region.HelixCenterPoint[1] + r * math.sin(offsetFi)]

            op.commandlist.append(Path.Command("(helix to depth: %f)"%passEndDepth))
            #if step == 1:
            #rapid move to start point
            op.commandlist.append(Path.Command(
                "G0", {"X": helixStart[0], "Y": helixStart[1], "Z": obj.ClearanceHeight.Value}))
            #rapid move to safe height
            op.commandlist.append(Path.Command(
                "G0", {"X": helixStart[0], "Y": helixStart[1], "Z": obj.SafeHeight.Value}))

            op.commandlist.append(Path.Command("G1", {
                                  "X": helixStart[0], "Y": helixStart[1], "Z": passStartDepth, "F": op.vertFeed}))

            while fi<maxfi:
                x = region.HelixCenterPoint[0] + r * math.cos(fi+offsetFi)
                y = region.HelixCenterPoint[1] + r * math.sin(fi+offsetFi)
                z = passStartDepth - fi / maxfi * (passStartDepth - passEndDepth)
                op.commandlist.append(Path.Command("G1", { "X": x, "Y":y, "Z":z, "F": op.vertFeed}))
                lx=x
                ly=y
                fi=fi+math.pi/16
            op.commandlist.append(Path.Command("(adaptive - depth: %f)"%passEndDepth))
            #add adaptive paths
            for pth in region.AdaptivePaths:
                #print pth.Points
                motionType = pth[0]  #[0] contains motion type
                for pt in pth[1]: #[1] contains list of points
                    x=pt[0]
                    y =pt[1]
                    dist=math.sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly))
                    if motionType == PathAdaptiveCore.MotionType.Cutting:
                        op.commandlist.append(Path.Command("G1", { "X": x, "Y":y, "Z":passEndDepth, "F": op.horizFeed}))
                    elif motionType == PathAdaptiveCore.MotionType.LinkClear:
                         if dist > minLiftDistance:
                            if lx!=x or ly!=y:
                                op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":passEndDepth+stepUp}))
                            op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":passEndDepth+stepUp}))
                    elif motionType == PathAdaptiveCore.MotionType.LinkNotClear:
                        if lx!=x or ly!=y:
                            op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.ClearanceHeight.Value}))
                        op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":obj.ClearanceHeight.Value}))
                    elif motionType == PathAdaptiveCore.MotionType.LinkClearAtPrevPass:
                        if lx!=x or ly!=y:
                            op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":passStartDepth+stepUp}))
                        op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":passStartDepth+stepUp}))
                    lx=x
                    ly=y

        passStartDepth=passEndDepth
        #return to safe height in this Z pass
        op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.ClearanceHeight.Value}))

    op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.ClearanceHeight.Value}))

def Execute(op,obj):
    global sceneGraph
    global topZ
    reload(AdaptiveUtils)
    sceneGraph = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()

    Console.PrintMessage("*** Adaptive toolpath processing started...\n")

    #hide old toolpaths during recalculation
    obj.Path = Path.Path("(calculating...)")
    #store old visibility state
    job = op.getJob(obj)
    oldObjVisibility = obj.ViewObject.Visibility
    oldJobVisibility = job.ViewObject.Visibility

    obj.ViewObject.Visibility = False
    job.ViewObject.Visibility = False

    FreeCADGui.updateGui()
    try:
        Console.PrintMessage("Tool diam: %f \n"%op.tool.Diameter)
        helixDiameter = min(op.tool.Diameter,1000.0 if obj.HelixDiameterLimit.Value==0.0 else obj.HelixDiameterLimit.Value )
        nestingLimit=0
        topZ=op.stock.Shape.BoundBox.ZMax

        opType = PathAdaptiveCore.OperationType.Clearing
        obj.Stopped = False
        obj.StopProcessing = False
        if obj.Tolerance<0.001: obj.Tolerance=0.001

        edges=[]
        for base, subs in obj.Base:
            #print (base,subs)
            for sub in subs:
                shape=base.Shape.getElement(sub)
                for edge in shape.Edges:
                    edges.append(edge)

        pathArray=AdaptiveUtils.connectEdges(edges)

        if obj.OperationType == "Clearing":
            #print "pathArray:",pathArray
            if obj.Side == "Outside":
                # for p in pathArray:
                #     p.reverse()
                stockBB = op.stock.Shape.BoundBox
                v=[]
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
                v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMin,0))
                v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMax,0))
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMax,0))
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
                pathArray.append([v])
            if not obj.ProcessHoles: nestingLimit = 1
            opType = PathAdaptiveCore.OperationType.Clearing
        else: # profiling
            if obj.Side == "Outside":
                opType = PathAdaptiveCore.OperationType.ProfilingOutside
            else:
                opType = PathAdaptiveCore.OperationType.ProfilingInside
            if not obj.ProcessHoles: nestingLimit = 1

        path2d = AdaptiveUtils.convertTo2d(pathArray)
        # put here all properties that influence calculation of adaptive base paths,
        inputStateObject = {
            "tool": op.tool.Diameter,
            "tolerance": obj.Tolerance,
            "geometry" : path2d,
            "stepover" :obj.StepOver,
            "effectiveHelixDiameter": helixDiameter,
            "operationType": obj.OperationType,
            "side": obj.Side,
            "processHoles": obj.ProcessHoles

        }

        inputStateStr= json.dumps(inputStateObject)

        inputStateChanged=False
        adaptiveResults=None

        if obj.AdaptiveOutputState !=None and obj.AdaptiveOutputState != "":
             adaptiveResults = obj.AdaptiveOutputState

        if obj.AdaptiveInputState != inputStateStr:
             inputStateChanged=True
             adaptiveResults=None

        # progress callback fn, if return true it will stop processing
        def progressFn(tpaths):
            for path in tpaths: #path[0] contains the MotionType,#path[1] contains list of points
                sceneDrawPath(path[1])
            FreeCADGui.updateGui()
            return  obj.StopProcessing

        start=time.time()

        if inputStateChanged or adaptiveResults==None:
            a2d = PathAdaptiveCore.Adaptive2d()
            a2d.stepOverFactor = 0.01*obj.StepOver
            a2d.toolDiameter = op.tool.Diameter
            a2d.helixRampDiameter =  helixDiameter
            a2d.tolerance = obj.Tolerance
            a2d.opType = opType
            a2d.polyTreeNestingLimit = nestingLimit
            adaptiveResults = a2d.Execute(path2d,progressFn)


        GenerateGCode(op,obj,adaptiveResults,helixDiameter)

        # for i in range(0 , len(features)):
        #     feat = features[i]
        #     if inputStateChanged or not hasOutputState:
        #         if PROFILE:
        #             profiler = cProfile.Profile()
        #             baseToolPaths, startPoint= profiler.runcall(Adaptive.ProcessFeature.Execute,
        #                              op, obj, i, feat, SCALE_FACTOR)
        #             profiler.print_stats(sort='cumtime')
        #         else:
        #             baseToolPaths,startPoint=Adaptive.ProcessFeature.Execute(op,obj,i,feat, SCALE_FACTOR)
        #         outputState = {
        #             "baseToolPaths": baseToolPaths,
        #             "startPoint" : startPoint
        #         }
        #         outputStateObj.append(outputState)
        #     else:
        #         Console.PrintMessage("State not changed, using cached base paths\n")
        #     #Adaptive.GenerateGCode.Execute(op, obj, outputStateObj[i]["baseToolPaths"], outputStateObj[i]["startPoint"], SCALE_FACTOR)


        if not obj.StopProcessing:
            Console.PrintMessage("*** Done. Elapsed: %f sec\n\n" %(time.time()-start))
            obj.AdaptiveOutputState = adaptiveResults
            obj.AdaptiveInputState=inputStateStr
        else:
            Console.PrintMessage("*** Processing cancelled (after: %f sec).\n\n" %(time.time()-start))
    finally:
        obj.ViewObject.Visibility = oldObjVisibility
        job.ViewObject.Visibility = oldJobVisibility
        sceneClean()
