#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <sstream>

#pragma warning( push )
# pragma warning (disable:4800)
#include <Eigen/Dense>
#include <Eigen/StdVector> //NodeArray, EdgeArray
#pragma warning( pop )

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

	inline static Precision eps() {
		return 10*std::numeric_limits<Precision>::epsilon();
	}

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

		inline void print() const
		{
			std::cout<<"k="<<k.transpose()<<std::endl;
			std::cout<<"d="<<d.transpose()<<std::endl;
			std::cout<<"Ck=\n"<<Ck<<std::endl;
			std::cout<<"Cd=\n"<<Cd<<std::endl;
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
			Precision ang = p.head<3>().norm();
			Vec3T axis(1,0,0);
			if(ang>eps())
				axis = p.head<3>().normalized();
			else
				ang = 0;
			Eigen::AngleAxis<Precision> aa(ang, axis);
			return aa.toRotationMatrix();
		}

		//return translation vector t
		inline Vec3T t() const
		{
			return p.tail<3>();
		}

		//return T=[R,t;0,1]
		inline Mat4T toT() const
		{
			Mat4T ret;
			ret.setIdentity();
			ret.topLeftCorner<3,3>() = R();
			ret.topRightCorner<3,1>() = t();
			return ret;
		}

		inline Mat4T invT() const
		{
			Mat3T rot = R().transpose();
			Mat4T T;
			T.setIdentity();
			T.topLeftCorner<3,3>() = rot;
			T.topRightCorner<3,1>() = -rot * t();
			return T;
		}

		inline Vec6T invp() const
		{
			return T2p(invT());
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
			fromR(T.topLeftCorner<3,3>());
			fromt(T.topRightCorner<3,1>());
		}

		inline static Vec6T T2p(const Mat4T& T)
		{
			return Pose(T).p;
		}

		inline static Mat4T p2T(const Vec6T& p)
		{
			return Pose(p).toT();
		}

		Pose() { p.setZero(); Cp.setZero(); }
		Pose(const Vec6T& p_) : p(p_) {
			Cp.setZero();
		}
		Pose(const Mat4T& T) {
			fromT(T);
			Cp.setZero();
		}
		Pose(const Mat4T& T, const Mat6T& Covp) : Cp(Covp)
		{
			fromT(T);
		}
	};
	typedef std::vector< Vec6T, Eigen::aligned_allocator<Vec6T> > Vec6TArray;

	struct Node : public Pose {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::string name;	//name of the marker
		Veci vmeids;		//vmEdges' ids linked to this marker (not used now)
	};
	typedef std::vector< Node, Eigen::aligned_allocator<Node> > NodeArray;
	typedef int NID;
	const NID INVALID_NID = -1;

	struct Edge {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		int vid;	//id of view
		int mid;	//id of marker
		Mat2x4T u;	//observations, 2x4, TODO: allow different observations
	};
	typedef std::vector< Edge, Eigen::aligned_allocator<Edge> > EdgeArray;
	typedef int EID;
	const EID INVALID_EID = -1;

	struct Observation {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::string name;		//marker's name
		Pose init_marker_pose;	//marker pose in view, i.e., Tmc
		Mat2x4T u;				//observed marker corners in view

		bool operator<(const Observation& other) const {
			return perimeter(u) > perimeter(other.u);
		}
	};
	typedef std::vector< Observation,  Eigen::aligned_allocator<Observation> > ObsArray;
	typedef std::vector<ObsArray> VecObsArray;

	typedef std::map<std::string, int> Str2Int;

	class CMGraph {
	public: //member variables
		NodeArray markers;
		NodeArray views;
		EdgeArray edges;
		
		Pose fixed_marker_pose;
		NID fixed_marker_id;

		Calibration calib;
		Str2Int name2mid; //name -> makrer id

		Precision marker_half_size;
		bool verbose;

	public: //member functions
		CMGraph() : fixed_marker_id(INVALID_NID), marker_half_size(1), verbose(true) {}

		//print input information
		inline void print() const
		{
			std::cout<<"-------------------"<<std::endl;
			std::cout<<"calib:"<<std::endl;
			calib.print();
			std::cout<<std::endl;

			std::cout<<"marker_half_size="<<marker_half_size<<std::endl;
			std::cout<<"verbose="<<verbose<<std::endl;
			std::cout<<"-------------------"<<std::endl;
		}

		//print CMGraph state
		void report(std::ofstream& out) const;

		inline int nParams() const
		{
			return Pose::nParams()*static_cast<int>(markers.size()+views.size());
		}

		inline int nResiduals() const
		{
			return nObsResiduals() + nCstResiduals();
		}

		inline int nObsResiduals() const
		{
			return 8*static_cast<int>(edges.size()); //TODO: allow each marker to have more than 4 point observations
		}

		inline int nCstResiduals() const
		{
			return Pose::nParams();
		}

		NID setFixedMarker(const std::string &fixed_marker_name,
			const Precision p_ang=1e-4,
			const Precision p_pos=1e-2);

		//add all observed markers in this view to the graph, and also add view and edges
		//oa will be sorted by descending order of perimeters of the observed markers in the image
		NID addObsFromNewView(ObsArray& oa, const std::string &view_name);

		//bundle adjustment to optimize all markers' and views' poses
		bool CMGraph::optimizePose(
			const Precision sigma_u,
			const int max_iter,
			const Precision huber_loss_bandwidth=10, // +/- 10 pixels
			const Precision error_rel_tol=1e-2,
			const bool computeCovariance=false);

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

			Mat2x4T ret(ret_);
			return ret * half_size;
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

			Mat3x4T ret(ret_);
			return ret * half_size;
		}

		static void BatchProcess(
			const VecObsArray &frames, //each frame's Observations' order would be sorted
			const Calibration &calib,
			CMGraph& G,
			const std::string &fixed_marker_name="",
			const Precision p_ang=1e-4,
			const Precision p_pos=1e-2,
			const Precision sigma_u=0.2,
			const int max_iter_per_opt=20);

	protected:
		NID newMarker(const Vec6T& p, const Mat6T& Cp, const std::string& name);

		NID newView(const Vec6T& p, const Mat6T& Cp, const std::string& name);

		EID newEdge(const int vid, const int mid, const MatT& u);
	};
}