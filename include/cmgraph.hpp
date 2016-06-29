#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <sstream>

#include <Eigen/Dense>

namespace cmg {
	typedef double Precision;
	typedef Eigen::Matrix<Precision, Eigen::Dynamic, Eigen::Dynamic> MatT;
	typedef Eigen::Matrix<Precision, Eigen::Dynamic, 1> VecT;
	typedef Eigen::Matrix<Precision, 2, 2> Mat2T;
	typedef Eigen::Matrix<Precision, 2, 1> Vec2T;
	typedef Eigen::Matrix<Precision, 3, 3> Mat3T;
	typedef Eigen::Matrix<Precision, 3, 1> Vec3T;
	typedef Eigen::Matrix<Precision, 4, 4> Mat4T;
	typedef Eigen::Matrix<Precision, 4, 1> Vec4T;
	typedef Eigen::Matrix<Precision, 6, 6> Mat6T;
	typedef Eigen::Matrix<Precision, 6, 1> Vec6T;
	typedef Eigen::Matrix<Precision, 2, 4> Mat2x4T;
	typedef Eigen::Matrix<Precision, 4, 2> Mat4x2T;
	typedef Eigen::Matrix<Precision, 3, 4> Mat3x4T;
	typedef Eigen::Matrix<Precision, 4, 3> Mat4x3T;

	typedef std::vector<int> Veci;
	inline static void range(const int start, const int end, const int step, Veci& ret)
	{
		ret.clear();
		ret.reserve((end-start+1)/step);
		for(int v=start; v<=end; v+=step) ret.push_back(v);
		ret.resize(ret.size());
	}

	inline static Precision perimeter(const Mat2x4T& u)
	{
		Precision ret=0;
		for(int i=0; i<4; ++i) {
			ret += (u.col(i) - u.col((i+1)%4)).norm();
		}
		return ret;
	}

	struct Calibration {
		Vec4T k;		//[fx, fy, cx, cy]
		Vec2T d;		//[k1, k2]
		Mat4T Ck;		//Cov[k]
		Mat2T Cd;		//Cov[d]
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		inline Mat3T K() const
		{
			Mat3T ret;
			ret.setIdentity();
			ret(0,0)=k(0);
			ret(1,1)=k(1);
			ret(0,2)=k(2);
			ret(1,2)=k(3);
			return ret;
		}

		template<typename T>
		void project(const T X[3], T U[2]) const
		{
			T fx(k(0)), fy(k(1)), cx(k(2)), cy(k(3));
			T k1(d(0)), k2(d(1));

			T xn( X[0]/X[2] ), yn( X[1]/X[2] );
			T r2 = xn*xn + yn*yn;
			T factor=T(1)+(k2*r2+k1)*r2;
			T xnp=xn*factor;
			T ynp=yn*factor;
			U[0] = fx*xnp+cx;
			U[1] = fy*ynp+cy;
		}
	};

	//6D pose
	struct Pose {
		Vec6T p;		//[ra*rx,ra*ry,ra*rz,tx,ty,tz]
		Mat6T Cp;		//Cov[p]
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		static int nParams() { return 6; }

		//return rotation matrix R
		inline Mat3T R() const
		{
			Eigen::AngleAxis<Precision> aa(p.head<3>().norm(), p.head<3>().normalized());
			return aa.toRotationMatrix();
		}

		//return translation vector t
		inline Vec3T t() const
		{
			return p.tail<3>();
		}

		//return T=[R,t;0,1]
		inline Mat4T T() const
		{
			Mat4T ret;
			ret.setIdentity();
			ret.topLeftCorner<3,3>(0,0) = R();
			ret.topLeftCorner<3,1>(0,3) = t();
			return ret;
		}

		inline Mat4T invT() const
		{
			Mat3T rot = R().transpose();
			Mat4T T;
			T.setIdentity();
			T.topLeftCorner<3,3>(0,0) = rot;
			T.topLeftCorner<3,1>(0,3) = -rot * t();
			return T;
		}

		inline void fromR(const Mat3T& R)
		{
			Eigen::AngleAxis<Precision> aa;
			aa.fromRotationMatrix(R);
			p.head<3>() = aa.axis() * aa.angle();
		}

		inline void fromt(const Vec3T& t)
		{
			p.tail<3>() = t;
		}

		inline void fromT(const Mat4T& T)
		{
			fromR(T.topLeftCorner<3,3>(0,0));
			fromt(T.topLeftCorner<3,1>(0,3));
		}

		inline static Vec6T T2p(const Mat4T& T)
		{
			Vec6T p;
			Eigen::AngleAxis<Precision> aa;
			aa.fromRotationMatrix(T.topLeftCorner<3,3>(0,0));
			p.head<3>() = aa.axis() * aa.angle();
			p.tail<3>() = T.topLeftCorner<3,1>(0,3);
			return p;
		}

		inline static Mat4T p2T(const Vec6T& p)
		{
			Mat4T ret;
			ret.setIdentity();
			Eigen::AngleAxis<Precision> aa(p.head<3>().norm(), p.head<3>().normalized());
			ret.topLeftCorner<3,3>(0,0) = aa.toRotationMatrix();
			ret.topLeftCorner<3,1>(0,3) = p.tail<3>();
			return ret;
		}

		Pose() { p.setZero(); Cp.setZero(); }
		Pose(const Mat4T& T, const Mat6T& Covp) : Cp(Covp)
		{
			fromT(T);
		}
	};

	struct Node : public Pose {
		std::string name;	//name of the marker
		Veci vmeids;		//vmEdges' ids linked to this marker (not used now)
	};
	typedef std::vector<Node> NodeArray;
	typedef size_t NID;
	const NID INVALID_NID = -1;

	struct Edge {
		int vid;	//id of view
		int mid;	//id of marker
		Mat2x4T u;	//observations, 2x4, TODO: allow different observations
	};
	typedef std::vector<Edge> EdgeArray;
	typedef size_t EID;
	const EID INVALID_EID = -1;

	struct Observation {
		std::string name;
		Pose init_view_pose; //view pose in marker, i.e., Tcm
		Mat2x4T u;

		bool operator<(const Observation& other) const {
			return perimeter(u) > perimeter(other.u);
		}
	};
	typedef std::vector<Observation> ObsArray;

	typedef std::map<std::string, size_t> Str2Int;

	class CMGraph {
	public: //types
		struct Constraint {
			typedef Constraint *Ptr;
			NID fid;	//from node fid
			NID tid;	//to node tid
			int nResiduals;

			Constraint(NID f, NID t, int n) : fid(f), tid(t), nResiduals(n) {}
			virtual void operator()(const CMGraph &G, MatT& rout) const = 0;
			virtual ~Constraint() {}
		};
		typedef std::vector<Constraint::Ptr> CstPtrArray;
		struct Constraint_FixedMarker : public Constraint {
			Pose fixed_pose;

			Constraint_FixedMarker(NID mid, const Pose& pose)
				: Constraint(mid, mid, Pose::nParams()), fixed_pose(pose)
			{}

			void operator()(const CMGraph &G, MatT& rout) const {
				const Node& current_pose = G.markers[fid];
				rout = fixed_pose.Cp.inverse() * (current_pose.p - fixed_pose.p);
			}
		};

	public: //member variables
		NodeArray markers;
		NodeArray views;
		EdgeArray edges;
		CstPtrArray csts;

		Calibration calib;
		Str2Int name2mid; //name -> makrer id

		Precision marker_half_size;
		bool verbose;

	public: //member functions
		CMGraph() : marker_half_size(1), verbose(true) {}

		inline size_t nParams() const
		{
			return Pose::nParams()*(markers.size()+views.size());
		}

		inline size_t nResiduals() const
		{
			return nObsResiduals() + nCstResiduals();
		}

		inline size_t nObsResiduals() const
		{
			return 8*edges.size(); //TODO: allow each marker to have more than 4 point observations
		}

		inline int nCstResiduals() const
		{
			int ret=0;
			for(int i=0; i<csts.size(); ++i) {
				ret+=csts[i]->nResiduals;
			}
			return ret;
		}

		NID addObsFromNewView(ObsArray& oa, const std::string &view_name);

		bool CMGraph::optimizePose(
			const Precision sigma_u,
			const int max_iter,
			const Precision huber_loss_bandwidth=10, // +/- 10 pixels
			const Precision error_rel_tol=1e-2);

	public: //static functions
		//return 2D coordinates of a marker's 4 corners
		inline static Mat2x4T marker_x(const Precision half_size=1)
		{
			Precision ret_[]={
				-1,-1,
				 1,-1,
				 1, 1,
				-1, 1
			};

			Mat2x4T ret = Mat4x2T(ret_).transpose() * half_size;
			return ret;
		}

		//return 3D coordinates of a marker's 4 corners
		inline static Mat3x4T marker_X(const Precision half_size=1)
		{
			Precision ret_[]={
				-1,-1, 0,
				 1,-1, 0,
				 1, 1, 0,
				-1, 1, 0
			};

			Mat3x4T ret = Mat4x3T(ret_).transpose() * half_size;
			return ret;
		}

		static void BatchProcess(
			const std::vector<ObsArray> &frames, //each frame's Observations' order would be sorted
			const Calibration &calib,
			CMGraph& G,
			const std::string &fixed_marker_name="",
			const Precision p_ang=1e-4,
			const Precision p_pos=1e-2,
			const Precision sigma_u=0.2,
			const int max_iter_per_opt=20);

	protected:
		inline NID newMarker(const Vec6T& p, const Mat6T& Cp, const std::string& name)
		{
			Node node;
			node.p = p;
			node.Cp = Cp;
			node.name = name;
			markers.push_back(node);

			NID mid = markers.size()-1;
			name2mid[name]=mid;
			return mid;
		}

		inline NID newView(const Vec6T& p, const Mat6T& Cp, const std::string& name)
		{
			Node node;
			node.p = p;
			node.Cp = Cp;
			node.name = name;
			views.push_back(node);

			NID vid = views.size()-1;
			return vid;
		}

		inline EID newEdge(const int vid, const int mid, const MatT& u)
		{
			Edge edge;
			edge.vid=vid;
			edge.mid=mid;
			edge.u = u;

			EID eid = edges.size();
			edges.push_back(edge);

			views[vid].vmeids.push_back(int(eid));
			markers[mid].vmeids.push_back(int(eid));
			return eid;
		}

		inline size_t newConstraint(Constraint::Ptr pCst)
		{
			csts.push_back(pCst);
			return csts.size()-1;
		}
	};
}