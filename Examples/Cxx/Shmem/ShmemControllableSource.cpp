/** Example of shared memory interface
 *
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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
	Real timeStep = 0.001;
	Real finalTime = 10;
	String simName = "ShmemControllableSource";

	Interface intf("/dpsim01", "/dpsim10");

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto ecs = CurrentSource::make("v_intf");
	ecs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);

	ecs->connect({ Node::GND, n1 });
	r1->connect({ Node::GND, n1 });

<<<<<<< HEAD:Examples/Cxx/Shmem/ShmemControllableSource.cpp
	ecs->setAttributeRef("I_ref", intf.importComplex(0));
	intf.exportComplex(ecs->attributeMatrixComp("v_intf")->coeff(0, 0), 0);
=======
	// Logging
	auto logger = CSVDataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i10", r1->attribute("i_intf"));
>>>>>>> examples: use new CSVDataLogger class:Examples/Cxx/Circuits/DP_CS_R1.cpp

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{ecs, r1});
	RealTimeSimulation sim(simName, sys, timeStep, finalTime,
		Domain::DP, Solver::Type::MNA, Logger::Level::info);

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
