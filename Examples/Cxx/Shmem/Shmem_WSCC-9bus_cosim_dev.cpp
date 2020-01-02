/** CIM Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
	
	std::list<fs::path> filenames = DPsim::Utils::findFiles({
		"WSCC-09_RX_DI.xml",
		"WSCC-09_RX_EQ.xml",
		"WSCC-09_RX_SV.xml",
		"WSCC-09_RX_TP.xml"
	}, "Examples/CIM/WSCC-09_RX", "CIMPATH");

	String simName = "Shmem_WSCC-9bus_cosim";

	Logger::setLogDir("logs/" + simName);

	CIMReader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology sys = reader.loadCIM(60, filenames);
	
	std::cout << "BP1" <<  argc << std::endl;
	if (argc > 1){ 
		if (String(argv[1]) == "--cosim"){

			// COSIM CASE
			std::cout << "BP2" << std::endl;
			Interface intf("/dpsim-villas", "/villas-dpsim", nullptr, false);

			// Add current source that models the received distaix current
			auto ecs = CurrentSource::make("i_intf", Complex(0,0), Logger::Level::debug);
			ecs->connect({Node::GND, sys.node<Node>("BUS5")});
			ecs->setAttributeRef("I_ref", intf.importComplex(0));

			sys.addComponents({ecs});

			// Export attributes

			DPsim::UInt o = 0;

			auto compAttr = sys.node<Node>("BUS5")->attributeMatrixComp("v")->coeff(0,0);
			//auto magCompAttr = (compAttr->coeff(0,0)->mag());

			intf.exportReal(compAttr->mag(), o++);
			intf.exportReal(compAttr->phase(), o++);

			RealTimeSimulation sim(simName, sys, 1.0, 10,
			Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);


			std::ofstream of1(simName+"_topology_graph.svg");
			sys.topologyGraph().render(of1);

			// Add logger for comparison
			auto logger = DataLogger::make(simName);
			logger->addAttribute("v", sys.node<Node>("BUS5")->attribute("v"));
			
			//sim.doSplitSubnets(false);
			sim.addLogger(logger);

			// Set sync to false as there is only one interface using shmem
			sim.addInterface(&intf, false);
			sim.run();
		}
		else {
			std::cout << "ERROR! >" << String(argv[1]) << "< is no valid argument" << std::endl;
		}
	} 
	else {
		std::cout << "BP3" << std::endl;
		/////////////////////
		// Extend topology //
		/////////////////////

		// Nodes

		auto distNode1 = Node::make("DistNode1");
		auto distNode2 = Node::make("DistNode2");
		auto distNode3 = Node::make("DistNode3");
		
		// Cables

		auto distLine1 = CPS::DP::Ph1::PiLine::make("distLine1");
		auto distLine2 = CPS::DP::Ph1::PiLine::make("distLine2");
		auto distLine3 = CPS::DP::Ph1::PiLine::make("distLine3");

		distLine1->setParameters(5.29, 0.143128, -1.0, 0.000001);
		distLine2->setParameters(5.29, 0.143128, -1.0, 0.000001);
		distLine3->setParameters(5.29, 0.143128, -1.0, 0.000001);

		distLine1->connect({sys.node<Node>("BUS5"), distNode1});
		distLine2->connect({distNode1, distNode2});
		distLine3->connect({distNode2, distNode3});

		// Loads
		
		auto distLoad1 = CPS::DP::Ph1::RXLoad::make("distLoad1", 30000000, 10000000, 222222);
		auto distLoad2 = CPS::DP::Ph1::RXLoad::make("distLoad2", 30000000, 10000000, 222222);
		auto distLoad3 = CPS::DP::Ph1::RXLoad::make("distLoad3", 30000000, 10000000, 222222);

		distLoad1->connect({distNode1});
		distLoad2->connect({distNode2});
		distLoad3->connect({distNode3});

		// Add new components and nodes

		sys.addNodes({distNode1, distNode2, distNode3});
		
		sys.addComponents({distLoad1, distLoad2, distLoad3});
		sys.addComponents({distLine1, distLine2, distLine3});

		RealTimeSimulation sim(simName, sys, 1.0, 10,
			Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

		// Plot topology

		std::ofstream of1(simName+"_topology_graph.svg");
		sys.topologyGraph().render(of1);

		// Add logger for comparison
		auto logger = DataLogger::make(simName);
		logger->addAttribute("v", sys.node<Node>("BUS5")->attribute("v"));
		//logger->addAttribute("i", sys.node<Node>("BUS5")->attribute("i"));



		//sim.doSplitSubnets(false);
		sim.addLogger(logger);
		sim.run();
	}

	return 0;
}
