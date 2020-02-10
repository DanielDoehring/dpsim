/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *         Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

int main(int argc, char* argv[]) {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_VS_R2L3";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = Node::make("n1", PhaseType::ABC);
	auto n2 = Node::make("n2", PhaseType::ABC);
	auto n3 = Node::make("n3", PhaseType::ABC);
	auto n4 = Node::make("n4", PhaseType::ABC);

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(10,0), 50);
	auto r1 = Resistor::make("r_1");
	Matrix r1_param = Matrix::Zero(3, 3);
	r1_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	r1->setParameters(r1_param);

	auto l1 = Inductor::make("l_1");
	Matrix l_param = Matrix::Zero(3, 3);
	l_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	l1->setParameters(0.02 * l_param);

	auto l2 = Inductor::make("l_2");
	Matrix l2_param = Matrix(3, 1);
	l2->setParameters(0.1 * l_param);

	auto l3 = Inductor::make("l_3");
	l3->setParameters(0.05 * l_param);

	auto r2 = Resistor::make("r_2");
	r2->setParameters(2 * r1_param);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	l1->connect(Node::List{ n2, n3 });
	l2->connect(Node::List{ n3, Node::GND });
	l3->connect(Node::List{ n3, n4 });
	r2->connect(Node::List{ n4, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4}, SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));


	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();

	return 0;
}
