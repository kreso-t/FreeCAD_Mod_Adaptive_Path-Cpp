#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include "adaptive.hpp"

namespace py = pybind11;


using namespace AdaptivePath;
// main function

PYBIND11_MODULE(PathAdaptiveCore, m) {
    m.doc() = "Adaptive toolpaths module";


	py::enum_<MotionType> mt(m, "MotionType");
	 mt.value("Cutting", MotionType::mtCutting);
     mt.value("LinkClear", MotionType::mtLinkClear);
	 mt.value("LinkNotClear", MotionType::mtLinkNotClear);
	 mt.value("LinkClearAtPrevPass", MotionType::mtLinkClearAtPrevPass);

	py::enum_<OperationType> ot(m, "OperationType");
	 ot.value("Clearing", OperationType::otClearing);
     ot.value("ProfilingInside", OperationType::otProfilingInside);
	 ot.value("ProfilingOutside", OperationType::otProfilingOutside);


	py::class_<AdaptiveOutput> AdaptiveOutput(m, "AdaptiveOutput");
		AdaptiveOutput.def_readonly("HelixCenterPoint",&AdaptiveOutput::HelixCenterPoint);
		AdaptiveOutput.def_readonly("StartPoint",&AdaptiveOutput::StartPoint);
		AdaptiveOutput.def_readonly("AdaptivePaths",&AdaptiveOutput::AdaptivePaths);
		AdaptiveOutput.def_readonly("ReturnMotionType",&AdaptiveOutput::ReturnMotionType);

	// this does not seem to work correctly, changed to std::pair:
	// py::class_<TPath>(m, "TPath")
	// 	.def_readonly("MType",&TPath::MType)
	// 	.def_readonly("Points",&TPath::Points);


	py::class_<Adaptive2d> Adaptive2d(m, "Adaptive2d");
		Adaptive2d.def(py::init<>());
		Adaptive2d.def("Execute",&Adaptive2d::Execute) ;

	 	Adaptive2d.def_readwrite("stepOverFactor", &Adaptive2d::stepOverFactor);
	 	Adaptive2d.def_readwrite("toolDiameter", &Adaptive2d::toolDiameter);
		Adaptive2d.def_readwrite("helixRampDiameter", &Adaptive2d::helixRampDiameter);
		Adaptive2d.def_readwrite("polyTreeNestingLimit", &Adaptive2d::polyTreeNestingLimit);
		Adaptive2d.def_readwrite("tolerance", &Adaptive2d::tolerance);
		Adaptive2d.def_readwrite("opType", &Adaptive2d::opType);

		#ifdef DEV_MODE
		// debugging
		Adaptive2d.def_readwrite("DrawCircleFn", &Adaptive2d::DrawCircleFn);
		Adaptive2d.def_readwrite("ClearScreenFn", &Adaptive2d::ClearScreenFn);
		Adaptive2d.def_readwrite("DrawPathFn", &Adaptive2d::DrawPathFn);
		#endif

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
