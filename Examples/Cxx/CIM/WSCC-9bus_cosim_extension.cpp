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

    String simName = "WSCC-9bus_cosim_extension";
    Logger::setLogDir("logs/" + simName);


    // Voltage Source

    auto evs = VoltageSource::make("v_evs", Logger::Level::debug);
	evs->setParameters(Complex(15588.457,0.0));

    // Nodes

    auto distNode1 = Node::make("DistNode1");
    auto distNode2 = Node::make("DistNode2");
    auto distNode3 = Node::make("DistNode3");
    auto distNode4 = Node::make("DistNode4");
    
    evs->connect({Node::GND, distNode1});
    // Cables

    auto distLine1 = CPS::DP::Ph1::PiLine::make("distLine1");
    auto distLine2 = CPS::DP::Ph1::PiLine::make("distLine2");
    auto distLine3 = CPS::DP::Ph1::PiLine::make("distLine3");

    distLine1->setParameters(5.29, 0.143128, -1.0, 0.000001);
    distLine2->setParameters(5.29, 0.143128, -1.0, 0.000001);
    distLine3->setParameters(5.29, 0.143128, -1.0, 0.000001);

    distLine1->connect({distNode1, distNode2});
    distLine2->connect({distNode2, distNode3});
    distLine3->connect({distNode3, distNode4});

    // Loads
    
    // auto distLoad1 = CPS::DP::Ph1::RXLoad::make("distLoad1", 30000000, 10000000, 222222);
    // auto distLoad2 = CPS::DP::Ph1::RXLoad::make("distLoad2", 30000000, 10000000, 222222);
    // auto distLoad3 = CPS::DP::Ph1::RXLoad::make("distLoad3", 30000000, 10000000, 222222);

    auto distLoad1 = CPS::DP::Ph1::RXLoad::make("distLoad1");
    auto distLoad2 = CPS::DP::Ph1::RXLoad::make("distLoad2");
    auto distLoad3 = CPS::DP::Ph1::RXLoad::make("distLoad3");

    // distLoad1->setParameters(30000000, 10000000, 222222);
    // distLoad2->setParameters(30000000, 10000000, 222222);
    // distLoad3->setParameters(30000000, 10000000, 222222);


    distLoad1->setParameters(3000, 1000, 27000);
    distLoad2->setParameters(3000, 1000, 27000);
    distLoad3->setParameters(3000, 1000, 27000);

    distLoad1->connect({distNode2});
    distLoad2->connect({distNode3});
    distLoad3->connect({distNode4});    

    auto sys = SystemTopology(50,
        SystemNodeList{distNode1, distNode2, distNode3, distNode4}, 
        SystemComponentList{evs, distLine1, distLine2, distLine3,
            distLoad1, distLoad2, distLoad3});

    RealTimeSimulation sim (simName, sys, 1.0, 100,
        Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

    std::ofstream of1(simName + "_topology_graph.svg");
    sys.topologyGraph().render(of1);

    // Add Logging
    auto logger = DataLogger::make(simName);
    //logger->addAttribute("v_evs", evs->attribute("V_ref"));
    logger->addAttribute("v_node1", distNode1->attribute("v"));
    logger->addAttribute("v_node2", distNode2->attribute("v"));
    logger->addAttribute("v_node3", distNode3->attribute("v"));
    logger->addAttribute("v_node4", distNode4->attribute("v"));

    logger->addAttribute("I_Load1", distLoad1->attribute("i_intf"));
    logger->addAttribute("I_Load2", distLoad2->attribute("i_intf"));
    logger->addAttribute("I_Load3", distLoad3->attribute("i_intf"));

    logger->addAttribute("Vnom_Load1", distLoad1->attribute("V_nom"));
    logger->addAttribute("Vnom_Load2", distLoad2->attribute("V_nom"));
    logger->addAttribute("Vnom_Load3", distLoad3->attribute("V_nom"));

    logger->addAttribute("P_Load1", distLoad1->attribute("P"));
    logger->addAttribute("P_Load2", distLoad2->attribute("P"));
    logger->addAttribute("P_Load3", distLoad3->attribute("P"));

    logger->addAttribute("Q_Load1", distLoad1->attribute("Q"));
    logger->addAttribute("Q_Load2", distLoad2->attribute("Q"));
    logger->addAttribute("Q_Load3", distLoad3->attribute("Q"));

    logger->addAttribute("I_line1", distLine1->attribute("i_intf"));
    logger->addAttribute("I_line2", distLine2->attribute("i_intf"));
    logger->addAttribute("I_line3", distLine3->attribute("i_intf"));

    //sim.doSplitSubnets(false);
    sim.addLogger(logger);

    sim.run();
    return 0;
}