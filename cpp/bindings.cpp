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


	py::enum_<MotionType>(m, "MotionType")
	 .value("Cutting", MotionType::mtCutting)
     .value("LinkClear", MotionType::mtLinkClear)
	 .value("LinkNotClear", MotionType::mtLinkNotClear)
	 .value("LinkClearAtPrevPass", MotionType::mtLinkClearAtPrevPass);

	py::enum_<OperationType>(m, "OperationType")
	 .value("Clearing", OperationType::otClearing)
     .value("ProfilingInside", OperationType::otProfilingInside)
	 .value("ProfilingOutside", OperationType::otProfilingOutside);


	py::class_<AdaptiveOutput>(m, "AdaptiveOutput")
		.def(py::init<>())
		.def_readwrite("HelixCenterPoint",&AdaptiveOutput::HelixCenterPoint)
		.def_readwrite("StartPoint",&AdaptiveOutput::StartPoint)
		.def_readwrite("AdaptivePaths",&AdaptiveOutput::AdaptivePaths)
		.def_readwrite("ReturnMotionType",&AdaptiveOutput::ReturnMotionType);

	// this does not seem to work correctly, changed to std::pair:
	// py::class_<TPath>(m, "TPath")
	// 	.def_readonly("MType",&TPath::MType)
	// 	.def_readonly("Points",&TPath::Points);


	py::class_<Adaptive2d>(m, "Adaptive2d")
		.def(py::init<>())
		.def("Execute",&Adaptive2d::Execute)
	 	.def_readwrite("stepOverFactor", &Adaptive2d::stepOverFactor)
	 	.def_readwrite("toolDiameter", &Adaptive2d::toolDiameter)
		.def_readwrite("helixRampDiameter", &Adaptive2d::helixRampDiameter)
		.def_readwrite("polyTreeNestingLimit", &Adaptive2d::polyTreeNestingLimit)
		.def_readwrite("tolerance", &Adaptive2d::tolerance)
		.def_readwrite("opType", &Adaptive2d::opType)
		#ifdef DEV_MODE
		// debugging
		.def_readwrite("DrawCircleFn", &Adaptive2d::DrawCircleFn)
		.def_readwrite("ClearScreenFn", &Adaptive2d::ClearScreenFn)
		.def_readwrite("DrawPathFn", &Adaptive2d::DrawPathFn);
		#else
			;
		#endif

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
