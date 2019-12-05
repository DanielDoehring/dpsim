/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

	String simName = "WSCC-9bus";
	Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::off);
	// SystemTopology sys = reader.loadCIM(60, filenames);
	SystemTopology sys = reader.loadCIM(50, filenames);	

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


	// auto distLoad1 = CPS::DP::Ph1::PQLoadCS::make("distLoad1");
	// distLoad1->setParameters(125,50,21800);

	//auto component = sys.component<CPS::DP::Ph1::PiLine>("Load5");
	// component->setParameters(100,200,300,400);
	// component->connect({distNode2, distNode3});


	sys.addNodes({distNode1, distNode2, distNode3});
	
	sys.addComponents({distLoad1, distLoad2, distLoad3});
	sys.addComponents({distLine1, distLine2, distLine3});


    std::ofstream of1("topology_graph.svg");
    sys.topologyGraph().render(of1);
    //auto g = sys.topologyGraph();
    //g.render(of1);

	
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

	// Logging of the exisiting loads to compare with CIM files...
	logger->addAttribute("P_dist1", sys.component<Ph1::RXLoad>("distLoad1")->attribute("P"));
	logger->addAttribute("Q_dist1", sys.component<Ph1::RXLoad>("distLoad1")->attribute("Q"));
	logger->addAttribute("Vnom_dist1", sys.component<Ph1::RXLoad>("distLoad1")->attribute("V_nom"));

	logger->addAttribute("P_dist2", sys.component<Ph1::RXLoad>("distLoad2")->attribute("P"));
	logger->addAttribute("Q_dist2", sys.component<Ph1::RXLoad>("distLoad2")->attribute("Q"));
	logger->addAttribute("Vnom_dist2", sys.component<Ph1::RXLoad>("distLoad2")->attribute("V_nom"));

	logger->addAttribute("P_dist3", sys.component<Ph1::RXLoad>("distLoad3")->attribute("P"));
	logger->addAttribute("Q_dist3", sys.component<Ph1::RXLoad>("distLoad3")->attribute("Q"));
	logger->addAttribute("Vnom_dist3", sys.component<Ph1::RXLoad>("distLoad3")->attribute("V_nom"));

	logger->addAttribute("Rdist_series", sys.component<Ph1::PiLine>("distLine1")->attribute("R_series"));
	logger->addAttribute("Ldist_series", sys.component<Ph1::PiLine>("distLine1")->attribute("L_series"));
	logger->addAttribute("Cdist_parallel", sys.component<Ph1::PiLine>("distLine1")->attribute("C_parallel"));
	logger->addAttribute("Gdist_parallel", sys.component<Ph1::PiLine>("distLine1")->attribute("G_parallel"));


	logger->addAttribute("Pload_5", sys.component<Ph1::RXLoad>("LOAD5")->attribute("P"));
	logger->addAttribute("Qload_5", sys.component<Ph1::RXLoad>("LOAD5")->attribute("Q"));
	logger->addAttribute("V_nom_5", sys.component<Ph1::RXLoad>("LOAD5")->attribute("V_nom"));


	// logger->addAttribute("Pload_6", sys.component<Ph1::RXLoad>("LOAD6")->attribute("P"));
	// logger->addAttribute("Qload_6", sys.component<Ph1::RXLoad>("LOAD6")->attribute("Q"));
	// logger->addAttribute("V_nom_6", sys.component<Ph1::RXLoad>("LOAD6")->attribute("V_nom"));

	// logger->addAttribute("Pload_8", sys.component<Ph1::RXLoad>("LOAD8")->attribute("P"));
	// logger->addAttribute("Qload_8", sys.component<Ph1::RXLoad>("LOAD8")->attribute("Q"));
	// logger->addAttribute("V_nom_8", sys.component<Ph1::RXLoad>("LOAD8")->attribute("V_nom"));

	// Logging of used PiLines to get parameters
	// logger->addAttribute("R64_series", sys.component<Ph1::PiLine>("LINE64")->attribute("R_series"));
	// logger->addAttribute("L64_series", sys.component<Ph1::PiLine>("LINE64")->attribute("L_series"));
	// logger->addAttribute("C64_parallel", sys.component<Ph1::PiLine>("LINE64")->attribute("C_parallel"));
	// logger->addAttribute("G64_parallel", sys.component<Ph1::PiLine>("LINE64")->attribute("G_parallel"));

	// logger->addAttribute("R96_series", sys.component<Ph1::PiLine>("LINE96")->attribute("R_series"));
	// logger->addAttribute("L96_series", sys.component<Ph1::PiLine>("LINE96")->attribute("L_series"));
	// logger->addAttribute("C96_parallel", sys.component<Ph1::PiLine>("LINE96")->attribute("C_parallel"));
	// logger->addAttribute("G96_parallel", sys.component<Ph1::PiLine>("LINE96")->attribute("G_parallel"));

	// logger->addAttribute("R89_series", sys.component<Ph1::PiLine>("LINE89")->attribute("R_series"));
	// logger->addAttribute("L89_series", sys.component<Ph1::PiLine>("LINE89")->attribute("L_series"));
	// logger->addAttribute("C89_parallel", sys.component<Ph1::PiLine>("LINE89")->attribute("C_parallel"));
	// logger->addAttribute("G89_parallel", sys.component<Ph1::PiLine>("LINE89")->attribute("G_parallel"));

	// logger->addAttribute("R78_series", sys.component<Ph1::PiLine>("LINE78")->attribute("R_series"));
	// logger->addAttribute("L78_series", sys.component<Ph1::PiLine>("LINE78")->attribute("L_series"));
	// logger->addAttribute("C78_parallel", sys.component<Ph1::PiLine>("LINE78")->attribute("C_parallel"));
	// logger->addAttribute("G78_parallel", sys.component<Ph1::PiLine>("LINE78")->attribute("G_parallel"));

	// logger->addAttribute("R75_series", sys.component<Ph1::PiLine>("LINE75")->attribute("R_series"));
	// logger->addAttribute("L75_series", sys.component<Ph1::PiLine>("LINE75")->attribute("L_series"));
	// logger->addAttribute("C75_parallel", sys.component<Ph1::PiLine>("LINE75")->attribute("C_parallel"));
	// logger->addAttribute("G75_parallel", sys.component<Ph1::PiLine>("LINE75")->attribute("G_parallel"));

	logger->addAttribute("R54_series", sys.component<Ph1::PiLine>("LINE54")->attribute("R_series"));
	logger->addAttribute("L54_series", sys.component<Ph1::PiLine>("LINE54")->attribute("L_series"));
	logger->addAttribute("C54_parallel", sys.component<Ph1::PiLine>("LINE54")->attribute("C_parallel"));
	logger->addAttribute("G54_parallel", sys.component<Ph1::PiLine>("LINE54")->attribute("G_parallel"));

	Simulation sim(simName, sys, 0.0001, 0.1,
		Domain::DP, Solver::Type::MNA, Logger::Level::info, true);

	sim.doSplitSubnets(false);
	sim.addLogger(logger);
	sim.run();

	return 0;
}
