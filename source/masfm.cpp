#include "AllHelpers.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

#include "cmgraph.hpp"

using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;

enum PROCESS_MODE {
	MODE_BATCH=0,
	MODE_LIVE=1
};

PROCESS_MODE process_mode()
{
	if(helper::iniGet<std::string>("input","camera://0").find("://")==std::string::npos)
		return MODE_BATCH;
	return MODE_LIVE;
}

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
				fin >> obsi.init_marker_pose.p(pi);
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

int process_batch()
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

	{
		helper::ScopedTimer timer(helper::Timer::UNIT_MS, "[CMGraph.BatchProcess]");
		CMGraph::BatchProcess(frames, G.calib, G, fixed_marker_name, p_ang, p_pos, sigma_u, max_iter_per_opt);
	}

	save_graph_states(G);
	
	return 0;
}

struct AprilTagProcessor : public ImageHelper::ImageSource::Processor {

	std::vector< cv::Ptr<TagFamily> > gTagFamilies;
	cv::Ptr<TagDetector> gDetector;

	cmg::CMGraph G;

	double tagTextScale;
	int tagTextThickness;
	bool doLog,doRecord;
	bool isPhoto; //whether image source is photo/list or others
	bool useEachValidPhoto; //whether do log for each frame
	bool logVisFrame;
	std::string outputDir;
	int hammingThresh;

	AprilTagProcessor() : doLog(false), doRecord(false), isPhoto(false)
	{
		tagTextScale = helper::iniGet<double>("tagTextScale",0.4f);
		tagTextThickness = helper::iniGet<int>("tagTextThickness",1);
		useEachValidPhoto = helper::iniGet<bool>("useEachValidPhoto",true);
		hammingThresh = helper::iniGet<int>("hammingThresh",0);
		logVisFrame = helper::iniGet<bool>("logVisFrame",false);

		//// create tagFamily
		std::string tagid = helper::iniGet<std::string>("tagfamiliesID","0"); //defaul Tag16h5
		TagFamilyFactory::create(tagid, gTagFamilies);
		if(gTagFamilies.size()<=0) {
			tagle("create TagFamily failed all! exit...");
			exit(-1);
		}

		gDetector = new TagDetector(gTagFamilies);
		if(gDetector.empty()) {
			tagle("create TagDetector fail!");
			exit(-1);
		}
		gDetector->segDecimate = helper::iniGet<bool>("segDecimate",false);

		load_calibration(G);
		G.marker_half_size = helper::iniGet<cmg::Precision>("marker_half_size", 0.1);
		G.verbose = helper::iniGet<bool>("verbose", true);
		G.print();
	}

	virtual ~AprilTagProcessor()
	{
		save_graph_states(G);
	}

	bool addDetections2CMGraph(std::vector<TagDetection>& detections)
	{
		if(detections.size()<2) return false; //no need to add view with only 1 or 0 marker

		using namespace cmg;
		static Precision p_ang = helper::iniGet<Precision>("p_ang", 1e-4);
		static Precision p_pos = helper::iniGet<Precision>("p_pos", 1e-2);
		static Precision sigma_u = helper::iniGet<Precision>("sigma_u", 0.2);
		static int max_iter_per_opt = helper::iniGet<int>("max_iter_per_opt", 8);

		Mat3T Kt = G.calib.K().transpose(); //note Kt is in column major

		ObsArray oa;
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;

			if(G.markers.empty()) { //init the fixed marker
				G.setFixedMarker(detections[0].name(), p_ang, p_pos);
			}
			
			Observation obs;
			obs.name = dd.name();
			for(int k=0; k<4; ++k) {
				obs.u(0,k) = dd.p[k][0];
				obs.u(1,k) = dd.p[k][1];
			}
			
			double R[9], t[3];
			helper::RTfromKH(Kt.data(), &dd.homography[0][0], R, t, true);
			obs.init_marker_pose.fromR(Mat3T(R).transpose());
			obs.init_marker_pose.fromt(Vec3T(t) * G.marker_half_size);

			oa.push_back(obs);
		}

		NID ith = G.views.size();
		std::stringstream ss;
		ss << "v" << ith;
		NID vid = G.addObsFromNewView(oa, ss.str());
		if(vid==INVALID_NID) {
			if(G.verbose) {
				clogi("--------------------no connection to the graph, ignored\n");
			}
			return false;
		}
		G.optimizePose(sigma_u, max_iter_per_opt);

		if(G.verbose) {
			clogi(
				"#markers=%3d, #views=%3d, #edges=%4d | #parameters=%5d, #residuals=%6d"
				" --------------------added view %d\n",
				G.markers.size(), G.views.size(), G.edges.size(),
				G.nParams(), G.nResiduals(),
				ith
			);
			std::cout.flush();
		}

		return true;
	}

	/////// Override
	void operator()(cv::Mat& frame)
	{
		static helper::PerformanceMeasurer PM;
		std::vector<TagDetection> detections;
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		PM.tic();
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		//visualization
		int nValidDetections=0;
		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;
			++nValidDetections;

			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(static_cast<int>(dd.cxy[0]),static_cast<int>(dd.cxy[1])),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		//logging results
		if(nValidDetections>1 && (doLog || (isPhoto && useEachValidPhoto) || (!isPhoto && doRecord))) {
			doLog=false;
			addDetections2CMGraph(detections);
			//TODO: save logged frame to outputDir
		}

		cv::putText( frame,
			cv::format("#markers=%-4d #views=%-4d #edges=%-5d",
			G.markers.size(), G.views.size(), G.edges.size()),
			cv::Point(5,15), CV_FONT_NORMAL, 0.4, helper::CV_BLUE );
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[AprilTagProcessor] gDetector.segDecimate="<<gDetector->segDecimate); break;
		case 'l':
			doLog=true; break;
		case 'v':
			G.verbose=!G.verbose; break;
		case '1':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_DEBUG); break;
		case '2':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO); break;
		case 'h':
			std::cout<<
				"d: segDecimate\n"
				"l: do log\n"
				"v: toggle verbose\n"
				"1: debug output\n"
				"2: info output\n"
				<<std::endl; break;
		}
	}
};

int process_live()
{
	cv::Ptr<ImageSource> is = helper::createImageSource(helper::iniGet<std::string>("input","camera://0"));
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	AprilTagProcessor processor;

	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	processor.outputDir = helper::iniGet<std::string>("outputDir", is->getSourceDir());
	tagli("detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
		helper::iniGet<bool>("ImageSource:pause", is->getPause()),
		helper::iniGet<bool>("ImageSource:loop", is->getLoop()) );

	tagli("DONE...exit!");
	return 0;
}

int process()
{
	PROCESS_MODE mode = process_mode();
	switch(mode) {
	case MODE_BATCH:
		return process_batch();
	case MODE_LIVE:
		return process_live();
	default:
		tagle("unknown mode="<<mode<<"!");
	}
	return -1;
}

int main(const int argc, const char **argv, const char** envp )
{
	LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO);
	tagli(helper::getCurrentTimeString());

	if(argc>1) {
		helper::iniLoad(argv[1]);
	} else {
		helper::iniLoad("masfm.ini");
	}
	helper::iniLoad(envp, "masfm_", true);
	logli("");

	return process();
}