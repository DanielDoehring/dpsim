/** Benchmark scenario for cosimulation
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
    
    // Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "Examples/CIM/WSCC-09_RX", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

    String simName = "Shmem_WSCC-9bus_cosim_benchmark";
	Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::off);
	SystemTopology sys = reader.loadCIM(60, filenames);

	// Extend topology with switch
	auto sw = Ph1::Switch::make("StepLoad", Logger::Level::off);
	sw->setParameters(1e9, 0.1);
	sw->connect({ Node::GND, sys.node<Node>("BUS6") });
	sw->open();
	sys.addComponent(sw);

	auto swEvent1 = SwitchEvent::make(5.0, sw, true);
	auto swEvent2 = SwitchEvent::make(15.0, sw, false);
	
	// boolean --> busyWaiting
	Interface intf("/dpsim-villas", "/villas-dpsim", nullptr, true);

	// Add current source that models the received distaix current
	auto ecs = CurrentSource::make("ecs", Complex(0,0), Logger::Level::off);
	// Be careful in which direction ecs is connected as it should DRAW the
	// current instead of providing it!!
	ecs->connect({sys.node<Node>("BUS5"), Node::GND});
	ecs->setAttributeRef("I_ref", intf.importComplex(0));

	sys.addComponents({ecs});

	DPsim::UInt o = 0;

	// auto compAttr = node3->attributeMatrixComp("v")->coeff(0,0);
	auto compAttr = ecs->attributeMatrixComp("v_intf")->coeff(0,0);
	intf.exportComplex(compAttr, o++);
	
	std::ofstream of1(simName+"_topology_graph.svg");
	sys.topologyGraph().render(of1);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", sys.node<Node>("BUS1")->attribute("v"));
	logger->addAttribute("v2", sys.node<Node>("BUS2")->attribute("v"));
	logger->addAttribute("v3", sys.node<Node>("BUS3")->attribute("v"));
	logger->addAttribute("v4", sys.node<Node>("BUS4")->attribute("v"));
	logger->addAttribute("v5", sys.node<Node>("BUS5")->attribute("v"));
	logger->addAttribute("v6", sys.node<Node>("BUS6")->attribute("v"));
	logger->addAttribute("v7", sys.node<Node>("BUS7")->attribute("v"));
	logger->addAttribute("v8", sys.node<Node>("BUS8")->attribute("v"));
	logger->addAttribute("v9", sys.node<Node>("BUS9")->attribute("v"));
	//logger->addAttribute("wr_1", sys.component<Ph1::SynchronGeneratorTrStab>("GEN1")->attribute("w_r"));
	//logger->addAttribute("wr_2", sys.component<Ph1::SynchronGeneratorTrStab>("GEN2")->attribute("w_r"));
	//logger->addAttribute("wr_3", sys.component<Ph1::SynchronGeneratorTrStab>("GEN3")->attribute("w_r"));

	logger->addAttribute("LOAD5.v", sys.component<Ph1::RXLoad>("LOAD5")->attribute("v_intf"));
	logger->addAttribute("LOAD5.i", sys.component<Ph1::RXLoad>("LOAD5")->attribute("i_intf"));
	logger->addAttribute("LOAD6.v", sys.component<Ph1::RXLoad>("LOAD6")->attribute("v_intf"));
	logger->addAttribute("LOAD6.i", sys.component<Ph1::RXLoad>("LOAD6")->attribute("i_intf"));
	logger->addAttribute("ECS.v", ecs->attribute("v_intf"));
	logger->addAttribute("ECS.i", ecs->attribute("i_intf"));
	//logger->addAttribute("Switch.closed", sw->attribute("is_closed"));


	// TODO: Deactivate logging for benchmarks?
	Simulation sim(simName, sys, 0.01, 20,
		Domain::DP, Solver::Type::MNA, Logger::Level::info, true);

	sim.addEvent(swEvent1);
	sim.addEvent(swEvent2);
	sim.addLogger(logger);
	// boolean --> syncStart
	sim.addInterface(&intf, true);
	sim.run();

	return 0;

}