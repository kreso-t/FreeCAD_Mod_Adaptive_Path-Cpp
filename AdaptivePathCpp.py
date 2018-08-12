import PathScripts.PathOp as PathOp
import Path
import FreeCAD
import FreeCADGui
import time
from FreeCAD import Console
import json
import math
import AdaptiveUtils



class AdaptivePathOpCpp(PathOp.ObjectOp):
    def opFeatures(self, obj):
        '''opFeatures(obj) ... returns the OR'ed list of features used and supported by the operation.
        The default implementation returns "FeatureTool | FeatureDeptsh | FeatureHeights | FeatureStartPoint"
        Should be overwritten by subclasses.'''
        return PathOp.FeatureTool | PathOp.FeatureBaseEdges | PathOp.FeatureDepths | PathOp.FeatureStepDown | PathOp.FeatureHeights | PathOp.FeatureBaseGeometry

    def initOperation(self, obj):
        '''initOperation(obj) ... implement to create additional properties.
        Should be overwritten by subclasses.'''
        obj.addProperty("App::PropertyEnumeration", "Side", "Adaptive", "Side of selected faces that tool should cut")
        obj.Side = ['Outside', 'Inside']  # side of profile that cutter is on in relation to direction of profile
        obj.addProperty("App::PropertyFloat", "Tolerance", "Adaptive",  "Clearing tolerance")
        obj.addProperty("App::PropertyPercent", "StepOver", "Adaptive", "Percent of cutter diameter to step over on each pass")
        obj.addProperty("App::PropertyDistance", "LiftDistance", "Adaptive", "Lift distance for rapid moves")
        obj.addProperty("App::PropertyBool", "ProcessHoles", "Adaptive","Process holes as well as the face outline")
        obj.addProperty("App::PropertyBool", "Stopped",
                        "Adaptive", "Stop processing")
        obj.setEditorMode('Stopped', 2) #hide this property

        obj.addProperty("App::PropertyBool", "StopProcessing",
                                  "Adaptive", "Stop processing")
        obj.setEditorMode('StopProcessing', 2)  # hide this property

        obj.addProperty("App::PropertyString", "AdaptiveInputState",
                        "Adaptive", "Internal input state")
        obj.addProperty("App::PropertyString", "AdaptiveOutputState",
                        "Adaptive", "Internal output state")
        obj.setEditorMode('AdaptiveInputState', 2) #hide this property
        obj.setEditorMode('AdaptiveOutputState', 2) #hide this property
        obj.addProperty("App::PropertyAngle", "HelixAngle", "Adaptive",  "Helix ramp entry angle (degrees)")
        obj.addProperty("App::PropertyLength", "HelixDiameterLimit", "Adaptive", "Limit helix entry diameter, if limit larger than tool diameter or 0, tool diameter is used")


    def opSetDefaultValues(self, obj):
        obj.Side="Inside"
        obj.Tolerance = 0.1
        obj.StepOver = 20
        obj.LiftDistance=1.0
        obj.ProcessHoles = True
        obj.Stopped = False
        obj.StopProcessing = False
        obj.HelixAngle = 5
        obj.HelixDiameterLimit = 0.0
        obj.AdaptiveInputState =""
        obj.AdaptiveOutputState = ""

    def opExecute(self, obj):
        '''opExecute(obj) ... called whenever the receiver needs to be recalculated.
        See documentation of execute() for a list of base functionality provided.
        Should be overwritten by subclasses.'''

        import PathAdaptiveCore
        reload(PathAdaptiveCore)
        reload(AdaptiveUtils)

        Console.PrintMessage("*** Adaptive toolpath processing started...\n")

        #hide old toolpaths during recalculation
        obj.Path = Path.Path("(calculating...)")
        #store old visibility state
        job = self.getJob(obj)
        oldObjVisibility = obj.ViewObject.Visibility
        oldJobVisibility = job.ViewObject.Visibility

        obj.ViewObject.Visibility = False
        job.ViewObject.Visibility = False

        FreeCADGui.updateGui()
        try:
            Console.PrintMessage("Tool diam: %f \n"%self.tool.Diameter)

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
            #print "pathArray:",pathArray
            if obj.Side == "Outside":
                stockBB = self.stock.Shape.BoundBox
                v=[]
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
                v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMin,0))
                v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMax,0))
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMax,0))
                v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
                pathArray.append([v])

            # # put here all properties that influence calculation of adaptive base paths,
            # inputStateObject = {
            #     "tool": self.tool.Diameter,
            #     "tolerance": obj.Tolerance,
            #     "geometry" : pathArray,
            #     "stepover" :obj.StepOver,
            #     "effectiveHelixDiameter": min(self.tool.Diameter,1000.0 if obj.HelixDiameterLimit.Value==0.0 else obj.HelixDiameterLimit.Value )
            # }

            # inputStateStr= json.dumps(inputStateObject)

            # inputStateChanged=False


            # if obj.AdaptiveOutputState !=None and obj.AdaptiveOutputState != "":
            #     outputStateObj = json.loads(obj.AdaptiveOutputState)
            #     hasOutputState=True
            # else:
            #     hasOutputState=False
            #     outputStateObj=[]

            # if obj.AdaptiveInputState != inputStateStr:
            #     inputStateChanged=True
            #     hasOutputState=False
            #     outputStateObj=[]

            start=time.time()
            a2d = PathAdaptiveCore.Adaptive2d()
            path2d = AdaptiveUtils.convertTo2d(pathArray)
            print path2d
            a2d.Execute(path2d)

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
##                obj.AdaptiveOutputState = json.dumps(outputStateObj)
##                obj.AdaptiveInputState=inputStateStr
            else:
                Console.PrintMessage("*** Processing cancelled (after: %f sec).\n\n" %(time.time()-start))
        finally:
            obj.ViewObject.Visibility = oldObjVisibility
            job.ViewObject.Visibility = oldJobVisibility


def Create(name):
    '''Create(name) ... Creates and returns a Pocket operation.'''
    obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "Adaptive_Path_OperationCpp")
    proxy = AdaptivePathOpCpp(obj)
    return obj
