#include "lch.hpp"

#include "cmgraph.hpp"

using std::cout;
using std::endl;

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" [cfg=masfm.ini] [output=masfm.txt]"<<endl;
}

int main(const int argc, const char **argv, const char** envp )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	if (argc > 1) {
		std::string arg1(argv[1]);
		if (arg1 == "-h" || arg1 == "/?" || arg1 == "--help") {
			usage(argc, argv);
			return -1;
		}
	}

	if ((argc > 1 && !helper::iniLoad(argv[1]))
		|| (!helper::iniLoad("masfm.ini"))) {
		helper::iniLoad(envp, "masfm_", true);
	}

	return 0;
}