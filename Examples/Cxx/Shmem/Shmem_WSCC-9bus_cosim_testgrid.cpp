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
	
	for(int i = 0; i < argc; ++i){
		std::cout << argv[i] << std::endl;
	}
	
	String simName = "Shmem_WSCC-9bus_cosim_testgrid";

	Logger::setLogDir("logs/" + simName);

	//Complex voltage = Complex(21801.18, -15798.35);
	Complex voltage = Complex(0.0, 15588.457);

	if (argc > 1 && String(argv[1]) == "--cosim"){ 
		if (String(argv[1]) == "--cosim"){

			if(argc >= 4){
				voltage = Complex(std::stod(argv[2]),std::stod(argv[3]));
			}

			// TODO:COSIM CASE
			auto sys = SystemTopology(60);
			Interface intf("/dpsim-villas", "/villas-dpsim", nullptr, false);

			auto evs = VoltageSource::make("EVS");
			//evs->setParameters(Complex(25000.0,-18000.0));
			evs->setParameters(voltage);
			sys.addComponents({evs});

			// Nodes

			auto node1 = Node::make("Node1");
			auto node2 = Node::make("Node2");
			auto node3 = Node::make("Node3");
			auto helpNode = Node::make("HelpNode");

			sys.addNodes({node1, node2, node3, helpNode});

			evs->connect({Node::GND, node1});

			auto line1 = CPS::DP::Ph1::PiLine::make("Line1");
			auto line2 = CPS::DP::Ph1::PiLine::make("Line2");
			auto line3 = CPS::DP::Ph1::PiLine::make("Line3");

			line1->setParameters(5.29, 0.143128, -1.0, 0.000001);
			line2->setParameters(5.29, 0.143128, -1.0, 0.000001);
			line3->setParameters(5.29, 0.143128, -1.0, 0.000001);
		
			line1->connect({node1, node2});
			line2->connect({node2, node3});
			line3->connect({node3, helpNode});
			
			sys.addComponents({line1, line2, line3});
			
			// Loads
			auto load1 = CPS::DP::Ph1::RXLoad::make("Load1");
			auto load2 = CPS::DP::Ph1::RXLoad::make("Load2");
			auto load3 = CPS::DP::Ph1::RXLoad::make("Load3");

			load1->setParameters(33333, 11111, 27000);
			load2->setParameters(66666, 22222, 27000);
			load3->setParameters(99999, 33333, 27000);

			load1->connect({node1});
			load2->connect({node2});
			load3->connect({node3});

			sys.addComponents({load1, load2, load3});

			// Add current source that models the received distaix current
			auto ecs = CurrentSource::make("ecs", Complex(0,0), Logger::Level::debug);
			// Be careful in which direction ecs is connected as it should DRAW the
			// current instead of providing it!!
			ecs->connect({helpNode, Node::GND});
			ecs->setAttributeRef("I_ref", intf.importComplex(0));

			sys.addComponents({ecs});

			// // Export attributes

			DPsim::UInt o = 0;

			auto compAttr = node3->attributeMatrixComp("v")->coeff(0,0);
			intf.exportComplex(compAttr, o++);
			
			RealTimeSimulation sim(simName, sys, 1.0, 20,
				Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

			// // Simulation sim(simName, sys, 1.0, 100,
			// // 	Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);


			std::ofstream of1(simName+"_topology_graph.svg");
			sys.topologyGraph().render(of1);

			// Add logger for comparison
			auto logger = DataLogger::make(simName);
			logger->addAttribute("EVS_v", evs->attribute("v_intf"));
			logger->addAttribute("EVS_i", evs->attribute("i_intf"));

			logger->addAttribute("ECS_v", ecs->attribute("v_intf"));
			logger->addAttribute("ECS_i", ecs->attribute("i_intf"));
			//logger->addAttribute("ECS_I_ref", ecs->attribute("I_ref"));

			logger->addAttribute("Node3_v", node3->attribute("v"));

			// //sim.doSplitSubnets(false);
			sim.addLogger(logger);

			// Set sync to false as there is only one interface using shmem
			sim.addInterface(&intf, false);
			sim.run();
		}
	} 
	else {
		// std::cout << "BP3" << std::endl;
		/////////////////////
		// Extend topology //
		/////////////////////
		std::cout<<"what is happening?" << std::endl;
		if(argc >= 3){
			std::cout << "yes...?" << std::endl;
			voltage = Complex(std::stod(argv[1]),std::stod(argv[2]));
		}

		auto sys = SystemTopology(60);

		// FIXME: Set realistic voltage...
		auto evs = VoltageSource::make("EVS");
		//evs->setParameters(Complex(25000.0,-18000.0));
		evs->setParameters(voltage);
		sys.addComponents({evs});

		// Nodes

		auto node1 = Node::make("Node1");
		auto node2 = Node::make("Node2");
		auto node3 = Node::make("Node3");
		auto distNode4 = Node::make("distNode4");
		auto distNode5 = Node::make("distNode5");
		auto distNode6 = Node::make("distNode6");

		sys.addNodes({node1, node2, node3,
					distNode4, distNode5, distNode6});
		evs->connect({Node::GND, node1});
		
		// Cables

		auto line1 = CPS::DP::Ph1::PiLine::make("Line1");
		auto line2 = CPS::DP::Ph1::PiLine::make("Line2");
		auto line3 = CPS::DP::Ph1::PiLine::make("Line3");
		auto distLine4 = CPS::DP::Ph1::PiLine::make("distLine4");
		auto distLine5 = CPS::DP::Ph1::PiLine::make("distLine5");
		// auto line6 = CPS::DP::Ph1::PiLine::make("Line6");

		line1->setParameters(5.29, 0.143128, -1.0, 0.000001);
		line2->setParameters(5.29, 0.143128, -1.0, 0.000001);
		line3->setParameters(5.29, 0.143128, -1.0, 0.000001);
		distLine4->setParameters(5.29, 0.143128, -1.0, 0.000001);
		distLine5->setParameters(5.29, 0.143128, -1.0, 0.000001);

		line1->connect({node1, node2});
		line2->connect({node2, node3});
		line3->connect({node3, distNode4});
		distLine4->connect({distNode4, distNode5});
		distLine5->connect({distNode5, distNode6});

		sys.addComponents({line1, line2, line3});
		sys.addComponents({distLine4, distLine5});
		// Loads

		auto load1 = CPS::DP::Ph1::RXLoad::make("Load1");
		auto load2 = CPS::DP::Ph1::RXLoad::make("Load2");
		auto load3 = CPS::DP::Ph1::RXLoad::make("Load3");

		load1->setParameters(33333, 11111, 27000);
		load2->setParameters(66666, 22222, 27000);
		load3->setParameters(99999, 33333, 27000);

		load1->connect({node1});
		load2->connect({node2});
		load3->connect({node3});

		// distLoads
		Real load_p = 30000;
		Real load_q = 10000;
		Real load_v = 27000;

		auto distLoad4 = CPS::DP::Ph1::RXLoad::make("distLoad4");
		auto distLoad5 = CPS::DP::Ph1::RXLoad::make("distLoad5");
		auto distLoad6 = CPS::DP::Ph1::RXLoad::make("distLoad6");

		distLoad4->setParameters(load_p, load_q, load_v);
		distLoad5->setParameters(load_p, load_q, load_v);
		distLoad6->setParameters(load_p, load_q, load_v);

		distLoad4->connect({distNode4});
		distLoad5->connect({distNode5});
		distLoad6->connect({distNode6});

		// Add new components and nodes

		
		sys.addComponents({load1, load2, load3});
		sys.addComponents({distLoad4, distLoad5, distLoad6});

		// RealTimeSimulation sim(simName, sys, 1.0, 100,
		// 	Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

		Simulation sim(simName, sys, 1.0, 100,
			Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

		// Plot topology

		std::ofstream of1(simName+"_topology_graph.svg");
		sys.topologyGraph().render(of1);

		// Add logger for comparison
		auto logger = DataLogger::make(simName);
		logger->addAttribute("EVS_v", evs->attribute("v_intf"));
		logger->addAttribute("EVS_i", evs->attribute("i_intf"));

		logger->addAttribute("Line3_i", line3->attribute("i_intf"));
		logger->addAttribute("Line3_v", line3->attribute("v_intf"));

		// logger->addAttribute("Node1_v", node1->attribute("v"));
		// logger->addAttribute("Node2_v", node2->attribute("v"));
		logger->addAttribute("Node3_v", node3->attribute("v"));
		// logger->addAttribute("DistNode4_v", distNode4->attribute("v"));
		// logger->addAttribute("DistNode5_v", distNode5->attribute("v"));
		// logger->addAttribute("DistNode6_v", distNode6->attribute("v"));


		// logger->addAttribute("Load1_i", load1->attribute("i_intf"));
		// logger->addAttribute("Load2_i", load2->attribute("i_intf"));
		// logger->addAttribute("Load3_i", load3->attribute("i_intf"));
		// logger->addAttribute("DistLoad4_i", distLoad4->attribute("i_intf"));
		// logger->addAttribute("DistLoad5_i", distLoad5->attribute("i_intf"));
		// logger->addAttribute("DistLoad6_i", distLoad6->attribute("i_intf"));

		// logger->addAttribute("Load1_P", load1->attribute("P"));
		// logger->addAttribute("Load2_P", load2->attribute("P"));
		// logger->addAttribute("Load3_P", load3->attribute("P"));
		// logger->addAttribute("DistLoad4_P", distLoad4->attribute("P"));
		// logger->addAttribute("DistLoad5_P", distLoad5->attribute("P"));
		// logger->addAttribute("DistLoad6_P", distLoad6->attribute("P"));

		// logger->addAttribute("Load1_Q", load1->attribute("Q"));
		// logger->addAttribute("Load2_Q", load2->attribute("Q"));
		// logger->addAttribute("Load3_Q", load3->attribute("Q"));
		// logger->addAttribute("DistLoad4_Q", distLoad4->attribute("Q"));
		// logger->addAttribute("DistLoad5_Q", distLoad5->attribute("Q"));
		// logger->addAttribute("DistLoad6_Q", distLoad6->attribute("Q"));


		//sim.doSplitSubnets(false);
		sim.addLogger(logger);
		sim.run();

	}

	return 0;
}
