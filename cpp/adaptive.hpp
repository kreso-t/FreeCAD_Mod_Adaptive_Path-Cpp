#include "clipper.hpp"
#include <vector>
#include <list>

#define NTOL 1.0e-7  // numeric tolerance

namespace AdaptivePath {
	using namespace ClipperLib;

	enum MoveType { mtCutting = 0, mtLinkClear = 1, mtLinkClearAtPrevPass = 2, mtLinkClearAtSafeHeight = 3  };

	typedef std::vector<std::pair<double,double>> DPath;
	typedef std::vector<DPath> DPaths;

	// represeting output toolpath in xy plane
	// std::tuple<double,double,int> - first two values are x and y coord and thirs is the move type
	typedef std::vector<std::tuple<double,double,MoveType>> TPath;
	typedef std::vector<TPath> TPaths;

	class AdaptiveOutput {
		public:
			std::pair<double,double> HelixCenterPoint;
			TPaths AdaptivePaths;
	};

	class ProgressInfo {
		public:
			int PassNo;
			bool PassCompleted;
			DPath CurrentPath;
			std::pair<double,double> EngagePos;
			std::pair<double,double> EngageDir;
			std::pair<double,double> ToolPos;
			std::pair<double,double> ToolDir;

			void SetClipperPath(const Path pth, double scaleFactor, bool closed=false) {
				CurrentPath.resize(0);
				CurrentPath.reserve(pth.size());
				if(pth.size()==0) return;
				for(int i=0;i<pth.size();i++) {
					std::pair<double,double> p = std::pair<double,double>((double)pth[i].X/scaleFactor, (double)pth[i].Y/scaleFactor);
					CurrentPath.push_back(p);
				}
				if(closed) {
					std::pair<double,double> p = std::pair<double,double>((double)pth[0].X/scaleFactor, (double)pth[0].Y/scaleFactor);
					CurrentPath.push_back(p);
				}
			}

			void SetEngagePoint(const IntPoint &pos, const DoublePoint &dir, double scaleFactor) {
				EngagePos.first = pos.X/scaleFactor;
				EngagePos.second = pos.Y/scaleFactor;
				EngageDir.first = dir.X/scaleFactor;
				EngageDir.second = dir.Y/scaleFactor;
			}

			void SetToolPos(const IntPoint &pos, const DoublePoint &dir, double scaleFactor) {
				ToolPos.first = pos.X/scaleFactor;
				ToolPos.second = pos.Y/scaleFactor;
				ToolDir.first = dir.X/scaleFactor;
				ToolDir.second = dir.Y/scaleFactor;
			}

	};

	// used to isolate state -> enable potential adding of multi-threaded processing of separate regions

	class Adaptive2d {
		public:
			Adaptive2d();
			double toolDiameter=5;
			double helixRampDiameter=0;
			double stepOverFactor = 0.5;
			int polyTreeNestingLimit=0;
			double tolerance=0.1;
			std::list<AdaptiveOutput> Execute(const DPaths &paths);

			void SetProgressCallbackFn(std::function<bool(ProgressInfo &)> &fn) {
				progressCallbackFn=fn;
			}
			/*for debugging*/
			std::function<void(double cx,double cy, double radius, int color)> DrawCircleFn;
			std::function<void(const DPath &, int color)> DrawPathFn;
			std::function<void()> ClearScreenFn;

		private:
			std::function<bool(ProgressInfo &)> progressCallbackFn;
			ProgressInfo progressInfo;
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

			Path toolGeometry; // tool geometry at coord 0,0, should not be modified			
			Path toolGeometry2; // tool geometry at coord 0,0, should not be modified			
			Path boundBoxGeometry; // bound box geometry at 0,0, should not be modified

			void ProcessPolyNode(const Paths & boundPaths, const Paths & toolBoundPaths );
			bool FindEntryPoint(const Paths & toolBoundPaths, IntPoint &entryPoint /*output*/);
			double CalcCutArea(Clipper & clip,const IntPoint &toolPos, const IntPoint &newToolPos, const Paths &cleared_paths);
			friend class EngagePoint;
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

		private: // constants
			//const double RESOLUTION_FACTOR = 8.0;
			const double RESOLUTION_FACTOR = 10.0;
			const int MAX_ITERATIONS = 16;
			const double AREA_ERROR_FACTOR = 20; /* how precise to match the cut area to optimal */
			const long PASSES_LIMIT = 1000000; // used for debugging
			const long POINTS_PER_PASS_LIMIT = 10000000; // used for debugging
			const int ANGLE_HISTORY_POINTS=10;

	};
}
