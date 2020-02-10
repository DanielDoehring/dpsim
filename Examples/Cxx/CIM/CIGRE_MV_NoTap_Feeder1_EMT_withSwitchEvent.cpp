/*
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
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;


/*
 * This example runs the MNA solver for the CIGRE MV benchmark system with dynamic phasor representation
 *  (neglecting the tap changers of the transformers)
 */

void addLoadStepToSystem(std::shared_ptr<EMT::Ph3::Switch> loadStep,
	Matrix addedPowerAsResistance,
	String nodeName,
	SystemTopology& system) {
	Matrix openResistance = Matrix::Zero(3, 3);
	openResistance <<
		1e9, 0, 0,
		0, 1e9, 0,
		0, 0, 1e9;


	loadStep->setParameters(openResistance, addedPowerAsResistance);
	for (auto baseNode : system.mNodes) {
		if (baseNode->name() == nodeName) {
			auto node_sys = std::dynamic_pointer_cast<CPS::Node<Real>>(baseNode);
			loadStep->connect({ CPS::Node<Real>::GND, node_sys });
			loadStep->openSwitch();
			loadStep->initialize(system.mFrequencies);
			system.mComponents.push_back(loadStep);
			system.mComponentsAtNode[node_sys].push_back(loadStep);
			break;
		}
	}
}


int main(int argc, char** argv){
	Real timeStep = 100e-6;
	Real finalTime = 10;
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

	String simName = "EMT_noDG_faultN2_100e-6";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	// doesn't work if not creating second reader object?
	CIM::Reader readerEMT(simName, Logger::Level::info, Logger::Level::off);
    SystemTopology systemEMT = readerEMT.loadCIM(system_freq, filenames, CPS::Domain::EMT, PhaseType::ABC);

	auto loggerPF = DPsim::DataLogger::make(simName+".pf");
	for (auto node : systemSP.mNodes)
	{
		loggerPF->addAttribute(node->name() + ".V", node->attribute("v"));
	}
	Simulation simPF(simName + ".pf", systemSP, 1, 2, Domain::SP, Solver::Type::NRP, Logger::Level::info, true);
	simPF.addLogger(loggerPF);
	simPF.run();

	// extract initial voltages to DP system
	reader.writeSvVoltageFromStaticSysTopology(systemSP, systemEMT);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : systemEMT.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}

	//auto stepLoad_n11 = EMT::Ph3::Switch::make("StepLoad_n11", Logger::Level::info);
	//Matrix closeResistance = Matrix::Zero(3, 3);
	//closeResistance <<
	//	2353, 0, 0,
	//	0, 2353, 0,
	//	0, 0, 2353;
	//addLoadStepToSystem(stepLoad_n11, closeResistance, "N11", systemEMT);
	//auto sw1 = SwitchEvent3Ph::make(7-timeStep, stepLoad_n11, true);
	auto fault_n2 = EMT::Ph3::Switch::make("fault_n2", Logger::Level::info);
	Matrix closeResistance = Matrix::Zero(3, 3);
	closeResistance <<
		10, 0, 0,
		0, 10, 0,
		0, 0, 10;
	addLoadStepToSystem(fault_n2, closeResistance, "N2", systemEMT);
	auto sw1 = SwitchEvent3Ph::make(7 - timeStep, fault_n2, true);
	auto sw2 = SwitchEvent3Ph::make(7.1 - timeStep, fault_n2, false);

	//Simulation sim(simName, system, 1, 120, Domain::DP, Solver::Type::MNA, Logger::Level::info, true);
	Simulation sim(simName, Logger::Level::off);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	//sim.doSteadyStateInit(true);
	sim.doHarmonicParallelization(false);

	sim.addEvent(sw1);
	sim.addEvent(sw2);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

