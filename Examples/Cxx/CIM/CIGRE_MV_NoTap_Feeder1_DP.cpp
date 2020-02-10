/** (WORKING)
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include "cps/CIM/Reader.h"
#include <DPsim.h>


using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;


/*
 * This example runs the MNA solver for the CIGRE MV benchmark system with dynamic phasor representation
 *  (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){
	Real timeStep = 0.001;
	Real finalTime = 0.6;
	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
			}, "Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "CIGRE-MV-NoTap-DP-Feeder1";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	// doesn't work if not creating second reader object?
	CIM::Reader reader2(simName, Logger::Level::info, Logger::Level::debug);
    SystemTopology systemDP = reader2.loadCIM(system_freq, filenames, CPS::Domain::DP);

	auto loggerPF = DPsim::DataLogger::make(simName+".pf");
	for (auto node : systemSP.mNodes)
	{
		loggerPF->addAttribute(node->name() + ".V", node->attribute("v"));
	}
	Simulation simPF(simName + ".pf", systemSP, 1, 2, Domain::SP, Solver::Type::NRP, Logger::Level::info, true);
	simPF.addLogger(loggerPF);
	simPF.run();

	// extract initial voltages to DP system
	reader.writeSvVoltageFromStaticSysTopology(systemSP, systemDP);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : systemDP.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}


	//Simulation sim(simName, system, 1, 120, Domain::DP, Solver::Type::MNA, Logger::Level::info, true);
	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.doSteadyStateInit(true);
	sim.doHarmonicParallelization(false);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

