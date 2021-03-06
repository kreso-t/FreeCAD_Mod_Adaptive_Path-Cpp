import PathScripts.PathOp as PathOp
import Path
import FreeCAD
import FreeCADGui
from FreeCAD import Console
import AdaptivePathCppExec

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

        obj.addProperty("App::PropertyEnumeration", "OperationType", "Adaptive", "Type of adaptive operation")
        obj.OperationType = ['Clearing', 'Profiling']  # side of profile that cutter is on in relation to direction of profile

        obj.addProperty("App::PropertyFloat", "Tolerance", "Adaptive",  "Influences accuracy and performance")
        obj.addProperty("App::PropertyPercent", "StepOver", "Adaptive", "Percent of cutter diameter to step over on each pass")
        obj.addProperty("App::PropertyDistance", "LiftDistance", "Adaptive", "Lift distance for rapid moves")
        obj.addProperty("App::PropertyBool", "ProcessHoles", "Adaptive","Process holes as well as the face outline")
        obj.addProperty("App::PropertyBool", "Stopped",
                        "Adaptive", "Stop processing")
        obj.setEditorMode('Stopped', 2) #hide this property

        obj.addProperty("App::PropertyBool", "StopProcessing",
                                  "Adaptive", "Stop processing")
        obj.setEditorMode('StopProcessing', 2)  # hide this property

        obj.addProperty("App::PropertyPythonObject", "AdaptiveInputState",
                        "Adaptive", "Internal input state")
        obj.addProperty("App::PropertyPythonObject", "AdaptiveOutputState",
                        "Adaptive", "Internal output state")
        obj.setEditorMode('AdaptiveInputState', 2) #hide this property
        obj.setEditorMode('AdaptiveOutputState', 2) #hide this property
        obj.addProperty("App::PropertyAngle", "HelixAngle", "Adaptive",  "Helix ramp entry angle (degrees)")
        obj.addProperty("App::PropertyLength", "HelixDiameterLimit", "Adaptive", "Limit helix entry diameter, if limit larger than tool diameter or 0, tool diameter is used")


    def opSetDefaultValues(self, obj, job):
        obj.Side="Inside"
        obj.OperationType = "Clearing"
        obj.Tolerance = 0.08
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
        #reload(AdaptivePathCppExec)
        AdaptivePathCppExec.Execute(self,obj)



def Create(name):
    '''Create(name) ... Creates and returns a Pocket operation.'''
    obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "Adaptive_Operation")
    proxy = AdaptivePathOpCpp(obj, None)
    return obj
