/**
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/LoadProfileReader.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){
	CommandLineArgs args(argc, argv);

	#ifdef _WIN32
		String loadProfilePath("..\\..\\..\\..\\dpsim\\Examples\\CSV\\CIGRE_MV_NoTap\\");
	#elif defined(__linux__) || defined(__APPLE__)
		String loadProfilePath("Examples/CSV/CIGRE_MV_NoTap/");
	#endif

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_06J16h_DI.xml",
			"Rootnet_FULL_NE_06J16h_EQ.xml",
			"Rootnet_FULL_NE_06J16h_SV.xml",
			"Rootnet_FULL_NE_06J16h_TP.xml"
		}, "Examples/CIM/CIGRE_MV_NoTap", "CIMPATH");
	}
	else {
		filenames = args.positionalPaths();
	}

	String simName = "Shmem_CIGRE-MV-NoTap";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::off);
    SystemTopology sys = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);
	LoadProfileReader lpreader(simName, loadProfilePath, Logger::Level::info);
	lpreader.assign(sys, 1, 1, 60, LoadProfileReader::Mode::AUTO);

	RealTimeSimulation sim(simName, sys, args.timeStep, args.duration, args.solver.domain, args.solver.type, args.logLevel);
	Interface intf("/dpsim1-villas", "/villas-dpsim1");

	ofstream villas_conf;
    villas_conf.open ("villas_sent_data.conf");

    // Register exportable node voltages
	string list_varnames[sys.mNodes.size()*2];
	UInt o = 0;
	for (auto n : sys.mNodes) {
		UInt i;
		if (sscanf(n->name().c_str(), "N%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		auto n_stat = std::dynamic_pointer_cast<CPS::SP::Node>(n);
		auto v = n_stat->attributeMatrixComp("v")->coeff(0, 0);

        std::cout << "Signal " << (i*2)+0 << ": Mag  " << n->name() << std::endl;
		std::cout << "Signal " << (i*2)+1 << ": Phas " << n->name() << std::endl;

		intf.exportReal(v->mag(),   (i*2)+0); o++;
		intf.exportReal(v->phase(), (i*2)+1); o++;

		list_varnames[(i*2)+0] = n->name() + ".V.mag";
		list_varnames[(i*2)+1] = n->name() + ".V.phase";
	}

    for (auto varname : list_varnames) {
        villas_conf << varname << std::endl;
	}
    villas_conf.close();

	sim.addInterface(&intf, false);

	sim.run(std::chrono::seconds(5));

	return 0;
}
