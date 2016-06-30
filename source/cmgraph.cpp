#include "cmgraph.hpp"

#pragma warning( push )
# pragma warning (disable:4819)
# pragma warning (disable:4251)
# pragma warning (disable:4355)
#ifdef LIBGLOG_IS_STATIC //see https://github.com/google/glog/issues/103
#define GOOGLE_GLOG_DLL_DECL
#endif
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#pragma warning( pop )

namespace cmg {

#if 0
	struct MarkerReprojectionError
	{
		const CMGraph& G;
		const EID eid;

		MarkerReprojectionError(const CMGraph& G_, const EID& eid_) : G(G_), eid(eid_) {}

		template<typename T>
		static inline void transposeInPlace3x3(T R[9])
		{
			std::swap(R[1], R[3]);
			std::swap(R[2], R[6]);
			std::swap(R[5], R[7]);
		}

		template<typename T>
		static inline void multAxAddb(T A[9], T x[3], T b[3], T out[3]) //A is col-major
		{
			out[0] = A[0]*x[0] + A[3]*x[1] + A[6]*x[2] + b[0];
			out[1] = A[1]*x[0] + A[4]*x[1] + A[7]*x[2] + b[1];
			out[2] = A[2]*x[0] + A[5]*x[1] + A[8]*x[2] + b[2];
		}

		template<typename T>
		static inline void negRt(T R[9], T t[3], T out[3]) //R is col-major
		{
			out[0] = -( R[0]*t[0] + R[3]*t[1] + R[6]*t[2] );
			out[1] = -( R[1]*t[0] + R[4]*t[1] + R[7]*t[2] );
			out[2] = -( R[2]*t[0] + R[5]*t[1] + R[8]*t[2] );
		}

		template <typename T>
		bool operator()(
			const T* const view_pose,
			const T* const marker_pose,
			T* residuals) const
		{
			const Mat3x4T mX = CMGraph::marker_X(G.marker_half_size);
			const Mat2x4T& obs = G.edges[eid].u;

			T R_view_in_world[9];
			T *t_view_in_world = view_pose+3;
			ceres::AngleAxisToRotationMatrix(view_pose, R_view_in_world); //column-major
			transposeInPlace3x3(R_view_in_world);
			
			T *R_world_in_view = &R_view_in_world[0];
			T t_world_in_view[3];
			negRt(R_world_in_view, t_view_in_world, t_world_in_view);

			for(int i=0; i<4; ++i) {
				const T pm[3] = { T(mX(0, i)), T(mX(1, i)), T(mX(2, i)) };

				T pw[3];
				ceres::AngleAxisRotatePoint(marker_pose, pm, pw);
				pw[0] += marker_pose[3];
				pw[1] += marker_pose[4];
				pw[2] += marker_pose[5];

				T pc[3];
				multAxAddb(R_world_in_view, pw, t_world_in_view, pc);

				T U[2];
				G.calib.project(pc, U);

				residuals[i*2+0] = U[0] - T(obs(0, i));
				residuals[i*2+1] = U[1] - T(obs(1, i));
			}
			
			return true;
		}
	};
#endif

	struct MarkerReprojectionError2 {
		const CMGraph& G;
		const EID eid;

		MarkerReprojectionError2(const CMGraph& G_, const EID& eid_) : G(G_), eid(eid_) {}

		template <typename T>
		bool operator()(
			const T* const view_pose_inverse,
			const T* const marker_pose,
			T* residuals) const
		{
			const Mat3x4T mX = CMGraph::marker_X(G.marker_half_size);
			const Mat2x4T& obs = G.edges[eid].u;

			for(int i=0; i<4; ++i) {
				const T pm[3] = { T(mX(0, i)), T(mX(1, i)), T(mX(2, i)) };

				T pw[3];
				ceres::AngleAxisRotatePoint(marker_pose, pm, pw);
				pw[0] += marker_pose[3];
				pw[1] += marker_pose[4];
				pw[2] += marker_pose[5];

				T pc[3];
				ceres::AngleAxisRotatePoint(view_pose_inverse, pw, pc);
				pc[0] += view_pose_inverse[3];
				pc[1] += view_pose_inverse[4];
				pc[2] += view_pose_inverse[5];

				T U[2];
				G.calib.project(pc, U);

				residuals[i*2+0] = U[0] - T(obs(0, i));
				residuals[i*2+1] = U[1] - T(obs(1, i));
			}
			
			return true;
		}
	};

	struct FixedMarkerError {
		const Pose& fixed_pose;

		FixedMarkerError(const Pose& p) : fixed_pose(p) {}

		template<typename T>
		bool operator()(
			const T* const pose,
			T* residuals) const
		{
			//TODO: use fixed_pose.Cp.inverse() * (current_pose.p - fixed_pose.p);
			T w_ang(1./fixed_pose.Cp(0,0));
			residuals[0] = w_ang*(pose[0] - T(fixed_pose.p(0)));
			residuals[1] = w_ang*(pose[1] - T(fixed_pose.p(1)));
			residuals[2] = w_ang*(pose[2] - T(fixed_pose.p(2)));
			T w_pos(1./fixed_pose.Cp(3,3));
			residuals[3] = w_pos*(pose[3] - T(fixed_pose.p(3)));
			residuals[4] = w_pos*(pose[4] - T(fixed_pose.p(4)));
			residuals[5] = w_pos*(pose[5] - T(fixed_pose.p(5)));
			return true;
		}
	};

	NID CMGraph::addObsFromNewView( ObsArray& oa, const std::string &view_name )
	{
		//sort by perimeter descending order
		std::sort(oa.begin(), oa.end());

		//find first marker that has been added to the graph before
		NID vid = INVALID_NID;
		Veci mids(oa.size(), INVALID_NID); //mids[i] stores the i-th observation's marker ID
		NID first_mid = INVALID_NID;
		int first_mid_pos = -1;
		for(size_t i=0; i<oa.size(); ++i) {
			const Observation& obs = oa[i];
			Str2Int::const_iterator itr = name2mid.find(obs.name);
			if(itr!=name2mid.end()) {
				mids[i] = itr->second;
				if(first_mid==INVALID_NID) {
					first_mid = itr->second;
					first_mid_pos = static_cast<int>(i);
				}
			}
		}

		if(first_mid==INVALID_NID) return vid;
		Mat4T Tmw = markers[first_mid].toT();
		Mat4T Tcm = oa[first_mid_pos].init_view_pose.toT();
		Mat4T Tcw = Tmw * Tcm;
		//TODO: use all existing marker observation to init the new view pose
		vid = newView(Pose::T2p(Tcw), Mat6T::Identity(), view_name);

		for(size_t i=0; i<oa.size(); ++i) {
			const Observation& obs = oa[i];
			int midi = mids[i];
			if(midi==INVALID_NID) { //new markers
				Mat4T Tmc = obs.init_view_pose.invT();
				Tmw = Tcw * Tmc;
				midi = newMarker(Pose::T2p(Tmw), Mat6T::Identity(), obs.name);
			}
			newEdge(vid, midi, obs.u);
		}
		return vid;
	}

	bool CMGraph::optimizePose(
		const Precision sigma_u,
		const int max_iter,
		const Precision huber_loss_bandwidth/*=10*/,
		const Precision error_rel_tol/*=1e-2*/)
	{
		//1. push residuals into ceres
		ceres::Problem problem;
		ceres::LossFunction* lossFunc = new ceres::HuberLoss(huber_loss_bandwidth);

		//1.1 marker observation residuals
		for(size_t eid=0; eid<edges.size(); ++eid) {
			const Edge& edge = edges[eid];
			Node &view = views[edge.vid];
			Node &marker = markers[edge.mid];

			view.p = Pose::T2p(view.invT()); //temporarily inverse the pose
			Precision *view_pose_inverse = view.p.data();
			Precision *marker_pose = marker.p.data();

			ceres::CostFunction* costFunc =
				new ceres::AutoDiffCostFunction<MarkerReprojectionError2, 8, 6, 6>
				(new MarkerReprojectionError2(*this, static_cast<int>(eid)));

			problem.AddResidualBlock(costFunc, lossFunc, view_pose_inverse, marker_pose);
		}

		//1.2 constraint residuals
		Precision *marker_pose = fixed_marker_pose.p.data();
		ceres::CostFunction* costFunc =
			new ceres::AutoDiffCostFunction<FixedMarkerError, 6, 6>
			(new FixedMarkerError(fixed_marker_pose));
		problem.AddResidualBlock(costFunc, 0, marker_pose);

		//2. solve
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //TODO: SPARSE_SCHUR?
		options.minimizer_progress_to_stdout = verbose;
		options.max_num_iterations = max_iter;
		options.function_tolerance = error_rel_tol;

		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		if(verbose) {
			std::cout << summary.BriefReport() << std::endl;
		}

		//3. update
		for(size_t eid=0; eid<edges.size(); ++eid) {
			const Edge& edge = edges[eid];
			Node &view = views[edge.vid];

			view.p = Pose::T2p(view.invT()); //update the view pose
		}

		if(summary.termination_type == ceres::FAILURE)
			return false;

		if(verbose) {
			std::cout<<"Before ceres, average Re-projection Error=initial_cost/num_residuals="
				<<summary.initial_cost/summary.num_residuals<<std::endl;
			Precision final_average_reproj_err_ = summary.final_cost/summary.num_residuals;
			std::cout<<"After ceres, average Re-projection Error=final_cost/num_residuals="
				<<final_average_reproj_err_<<std::endl;
		}

		return true;
	}

	//fix input observations and camera intrinsic parameters,
	//optimize and output marker and view poses
	void CMGraph::BatchProcess(
		const std::vector<ObsArray> &frames, /*each frame's Observations' order would be sorted */
		const Calibration &calib,
		CMGraph& G,
		const std::string &fixed_marker_name/*=""*/,
		const Precision p_ang/*=1e-4*/,
		const Precision p_pos/*=1e-2*/,
		const Precision sigma_u/*=0.2*/,
		const int max_iter_per_opt/*=20*/ )
	{
		G.calib = calib;

		//1. setup fixed marker
		Vec6T fixed_cov;
		fixed_cov << p_ang, p_ang, p_ang, p_pos, p_pos, p_pos;
		NID fixed_id = G.newMarker(Vec6T::Zero(),
			fixed_cov.asDiagonal(), fixed_marker_name);
		G.fixed_marker_pose = G.markers[fixed_id];

		//2. process each frame
		Veci frame_order;
		Veci frame_todo;
		range(0, static_cast<int>(frames.size()), 1, frame_order);
		for(size_t i=0; i<frames.size(); ++i) {
			const int ith = frame_order[i];
			ObsArray& oa = const_cast<ObsArray&>(frames[ith]);
			std::stringstream ss;
			ss << "v" << ith;
			NID vid = G.addObsFromNewView(oa, ss.str());
			if(vid==INVALID_NID) {
				frame_todo.push_back(ith);
				continue;
			}
			G.optimizePose(sigma_u, max_iter_per_opt);
		}

		//TODO: handle frame_todo
	}

}