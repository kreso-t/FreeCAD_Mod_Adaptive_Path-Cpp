#include "clipper.hpp"
#include <vector>
#include <list>

#define NTOL 1.0e-7  // numeric tolerance

namespace AdaptivePath {
	using namespace ClipperLib;

	enum MoveType { mtCutting = 0, mtLinkClear = 1, mtLinkNotClear = 2, mtLinkClearAtPrevPass = 3  };

	typedef std::pair<double,double> DPoint;
	typedef std::vector<DPoint> DPath;
	typedef std::vector<DPath> DPaths;
	struct TPath {
		DPath Points;
		MoveType MType;
	};
	typedef std::vector<TPath> TPaths;

	class AdaptiveOutput {
		public:
			DPoint HelixCenterPoint;
			TPaths AdaptivePaths;
			MoveType ReturnMoveType;
	};

	// used to isolate state -> enable potential adding of multi-threaded processing of separate regions

	class Adaptive2d {
		public:
			Adaptive2d();
			double toolDiameter=5;
			double helixRampDiameter=0;
			double stepOverFactor = 0.2;
			int polyTreeNestingLimit=0;
			double tolerance=0.1;
			std::list<AdaptiveOutput> Execute(const DPaths &paths, std::function<bool(TPaths &)> &progressCallbackFn);
			
			/*for debugging*/
			std::function<void(double cx,double cy, double radius, int color)> DrawCircleFn;
			std::function<void(const DPath &, int color)> DrawPathFn;
			std::function<void()> ClearScreenFn;

		private:
			std::list<AdaptiveOutput> results;
			Paths inputPaths;

			double scaleFactor=100;
			long toolRadiusScaled=10;
			long finishPassOffsetScaled=0;
			long helixRampRadiusScaled=0;
			long bbox_size=0;
			double referenceCutArea=0;
			double optimalCutAreaPD=0;
			double minCutAreaPD=0;

			time_t lastProgressTime = 0;
			
			std::function<bool(TPaths &)> * progressCallback=NULL;
			Path toolGeometry; // tool geometry at coord 0,0, should not be modified								
			Path boundBoxGeometry; // bound box geometry at 0,0, should not be modified

			void ProcessPolyNode(const Paths & boundPaths, const Paths & toolBoundPaths);
			bool FindEntryPoint(const Paths & toolBoundPaths, IntPoint &entryPoint /*output*/);
			double CalcCutArea(Clipper & clip,const IntPoint &toolPos, const IntPoint &newToolPos, const Paths &cleared_paths);
			void AppendToolPath(AdaptiveOutput & output,const Path & passToolPath,const Paths & cleared, bool close=false);
			bool  CheckCollision(const IntPoint &lastPoint,const IntPoint &nextPoint,const Paths & cleared);
			friend class EngagePoint; // for CalcCutArea

			void CheckReportProgress(TPaths &progressPaths);

			//debugging
			void DrawCircle(const IntPoint &  cp, double radiusScaled, int color ) {
				DrawCircleFn(1.0*cp.X/ scaleFactor, 1.0 *cp.Y/scaleFactor, radiusScaled/scaleFactor,color);
			}
			void DrawPath(const Path & path, int color ) {
				DPath dpath;
				if(path.size()==0) return;
				for(const IntPoint &pt : path) {
					std::pair<double,double> dpt = std::pair<double,double>((double)pt.X/scaleFactor, (double)pt.Y/scaleFactor);
					dpath.push_back(dpt);
				}
				DrawPathFn(dpath,color);
			}

			void DrawPaths(const Paths & paths, int color ) {
				for(const Path &p : paths) DrawPath(p,color);
			}

		private: // constants for fine tuning
			//const double RESOLUTION_FACTOR = 8.0;
			const double RESOLUTION_FACTOR = 10.0;
			const int MAX_ITERATIONS = 16;
			const double AREA_ERROR_FACTOR = 40; /* how precise to match the cut area to optimal */
			const int ANGLE_HISTORY_POINTS=10;
			const double ENGAGE_AREA_THR_FACTOR=0.1; // influences minimal engage area (factor relation to optimal)
			const double ENGAGE_SCAN_DISTANCE_FACTOR=0.5; // influences the engage scan/stepping distance
			const int DIRECTION_SMOOTHING_BUFLEN=5;

			const long PASSES_LIMIT = 1000000; // limit used for debugging
			const long POINTS_PER_PASS_LIMIT = 10000000; // limit used for debugging
			const time_t PROGRESS_TICKS = CLOCKS_PER_SEC/4; // progress report interval 
	};
}
