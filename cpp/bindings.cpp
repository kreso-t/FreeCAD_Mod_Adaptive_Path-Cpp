#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "adaptive.hpp"

namespace py = pybind11;


using namespace AdaptivePath;
// main function

PYBIND11_MODULE(PathAdaptiveCore, m) {
    m.doc() = "Module for generating 2d adaptive toolpaths";


	py::class_<AdaptiveOutput> AdaptiveOutput(m, "AdaptiveOutput");
		AdaptiveOutput.def_readonly("HelixCenterPoint",&AdaptiveOutput::HelixCenterPoint);
		AdaptiveOutput.def_readonly("AdaptivePaths",&AdaptiveOutput::AdaptivePaths);
		AdaptiveOutput.def_readonly("ReturnMoveType",&AdaptiveOutput::ReturnMoveType);

	py::class_<TPath> TPath(m, "TPath");
	TPath.def_readonly("MType",&TPath::MType);
	TPath.def_readonly("Points",&TPath::Points);
	
	// 	ProgressInfo.def_readonly("CurrentPath",&ProgressInfo::CurrentPath);
	// 	ProgressInfo.def_readonly("EngageDir",&ProgressInfo::EngageDir);
	// 	ProgressInfo.def_readonly("EngagePos",&ProgressInfo::EngagePos);
	// 	ProgressInfo.def_readonly("PassCompleted",&ProgressInfo::PassCompleted);
	// 	ProgressInfo.def_readonly("PassNo",&ProgressInfo::PassNo);
	// 	ProgressInfo.def_readonly("ToolDir",&ProgressInfo::ToolDir);
	// 	ProgressInfo.def_readonly("ToolPos",&ProgressInfo::ToolPos);

	// py::class_<ProgressInfo> ProgressInfo(m, "ProgressInfo");
	// 	ProgressInfo.def_readonly("CurrentPath",&ProgressInfo::CurrentPath);
	// 	ProgressInfo.def_readonly("EngageDir",&ProgressInfo::EngageDir);
	// 	ProgressInfo.def_readonly("EngagePos",&ProgressInfo::EngagePos);
	// 	ProgressInfo.def_readonly("PassCompleted",&ProgressInfo::PassCompleted);
	// 	ProgressInfo.def_readonly("PassNo",&ProgressInfo::PassNo);
	// 	ProgressInfo.def_readonly("ToolDir",&ProgressInfo::ToolDir);
	// 	ProgressInfo.def_readonly("ToolPos",&ProgressInfo::ToolPos);


	py::class_<Adaptive2d> Adaptive2d(m, "Adaptive2d");
		Adaptive2d.def(py::init<>());
		Adaptive2d.def("Execute",&Adaptive2d::Execute);

	 	Adaptive2d.def_readwrite("stepOverFactor", &Adaptive2d::stepOverFactor);
	 	Adaptive2d.def_readwrite("toolDiameter", &Adaptive2d::toolDiameter);
		Adaptive2d.def_readwrite("helixRampDiameter", &Adaptive2d::helixRampDiameter);
		Adaptive2d.def_readwrite("polyTreeNestingLimit", &Adaptive2d::polyTreeNestingLimit);
		Adaptive2d.def_readwrite("tolerance", &Adaptive2d::tolerance);
		// debugging
		Adaptive2d.def_readwrite("DrawCircleFn", &Adaptive2d::DrawCircleFn);
		Adaptive2d.def_readwrite("ClearScreenFn", &Adaptive2d::ClearScreenFn);
		Adaptive2d.def_readwrite("DrawPathFn", &Adaptive2d::DrawPathFn);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
