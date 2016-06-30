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

	G.calib.Ck = Vec4T(sk).asDiagonal();
	G.calib.Cd = Vec2T(sd).asDiagonal();
}

void load_all_frames(cmg::VecObsArray& frames)
{
}

int process()
{
	using namespace cmg;
	CMGraph G;

	load_calibration(G);
	G.marker_half_size = helper::iniGet<Precision>("marker_half_size", 0.1);
	G.print();

	static std::string fixed_marker_name = helper::iniGet<std::string>("fixed_marker_name", "Tag36h11.0");
	static Precision p_ang = helper::iniGet<Precision>("p_ang", 1e-4);
	static Precision p_pos = helper::iniGet<Precision>("p_pos", 1e-2);
	static Precision sigma_u = helper::iniGet<Precision>("sigma_u", 0.2);
	static int max_iter_per_opt = helper::iniGet<int>("max_iter_per_opt", 20);

	VecObsArray frames;
	load_all_frames(frames);

	CMGraph::BatchProcess(frames, G.calib, G, fixed_marker_name, p_ang, p_pos, sigma_u, max_iter_per_opt);
	
	return 0;
}

int main(const int argc, const char **argv, const char** envp )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	if(argc>1) {
		helper::iniLoad(argv[1]);
	} else {
		helper::iniLoad("masfm.ini");
	}
	helper::iniLoad(envp, "masfm_", true);

	return process();
}