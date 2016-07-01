#include "lch.hpp"

#include "cmgraph.hpp"

using std::cout;
using std::endl;

void load_calibration(cmg::CMGraph& G)
{
	using namespace cmg;

	G.calib.k(0) = helper::iniGet<Precision>("fx",500);
	G.calib.k(1) = helper::iniGet<Precision>("fy",500);
	G.calib.k(2) = helper::iniGet<Precision>("cx",320);
	G.calib.k(3) = helper::iniGet<Precision>("cy",240);

	G.calib.d(0) = helper::iniGet<Precision>("k1",0);
	G.calib.d(1) = helper::iniGet<Precision>("k2",0);

	Precision sk[4];
	Precision sd[2];
	sk[0] = helper::iniGet<Precision>("sigma_fx",0.1);
	sk[1] = helper::iniGet<Precision>("sigma_fy",0.1);
	sk[2] = helper::iniGet<Precision>("sigma_cx",0.1);
	sk[3] = helper::iniGet<Precision>("sigma_cy",0.1);

	sd[0] = helper::iniGet<Precision>("sigma_k1",0.1);
	sd[1] = helper::iniGet<Precision>("sigma_k2",0.1);

	G.calib.Ck = Vec4T(sk).cwiseAbs2().asDiagonal();
	G.calib.Cd = Vec2T(sd).cwiseAbs2().asDiagonal();
}

void load_all_frames(cmg::VecObsArray& frames)
{
	using namespace cmg;

	std::string fname = helper::iniGet<std::string>("input","");
	std::ifstream fin(fname.c_str());
	if(!fin.is_open()) {
		tagle("cannot open input file: "<<fname);
		exit(-1);
	}

	int nframes;
	fin >> nframes;
	frames.resize(nframes);

	for(int fi=0; fi<nframes; ++fi) {
		int ndets;
		fin >> ndets;
		ObsArray& deti = frames[fi];
		deti.resize(ndets);
		for(int di=0; di<ndets; ++di) {
			Observation& obsi = deti[di];
			fin >> obsi.name;
			for(int pi=0; pi<6; ++pi)
				fin >> obsi.init_view_pose.p(pi);
			for(int ui=0; ui<4; ++ui) {
				fin >> obsi.u(0, ui);
				fin >> obsi.u(1, ui);
			}
		}
	}
}

void save_graph_states(cmg::CMGraph& G)
{
	using namespace cmg;

	std::string fname = helper::iniGet<std::string>("output","");
	std::ofstream fout(fname.c_str());
	if(!fout.is_open()) {
		tagle("cannot open output file: "<<fname);
		exit(-1);
	}

	G.report(fout);
}

int process()
{
	using namespace cmg;
	CMGraph G;

	load_calibration(G);
	G.marker_half_size = helper::iniGet<Precision>("marker_half_size", 0.1);
	G.verbose = helper::iniGet<bool>("verbose", true);
	G.print();

	static std::string fixed_marker_name = helper::iniGet<std::string>("fixed_marker_name", "Tag36h11.0");
	static Precision p_ang = helper::iniGet<Precision>("p_ang", 1e-4);
	static Precision p_pos = helper::iniGet<Precision>("p_pos", 1e-2);
	static Precision sigma_u = helper::iniGet<Precision>("sigma_u", 0.2);
	static int max_iter_per_opt = helper::iniGet<int>("max_iter_per_opt", 20);

	VecObsArray frames;
	load_all_frames(frames);

	CMGraph::BatchProcess(frames, G.calib, G, fixed_marker_name, p_ang, p_pos, sigma_u, max_iter_per_opt);

	save_graph_states(G);
	
	return 0;
}

int main(const int argc, const char **argv, const char** envp )
{
	LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO);

	if(argc>1) {
		helper::iniLoad(argv[1]);
	} else {
		helper::iniLoad("masfm.ini");
	}
	helper::iniLoad(envp, "masfm_", true);
	logli("");

	return process();
}