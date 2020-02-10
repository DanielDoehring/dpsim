/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *         Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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
using namespace CPS::DP::Ph3;

int main(int argc, char* argv[]) {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_Ph3_VS_RC1";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = Node::make("n1", PhaseType::ABC);
	auto n2 = Node::make("n2", PhaseType::ABC);

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	Matrix r1_param = Matrix::Zero(3, 3);
	r1_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	r1->setParameters(r1_param);
	auto c1 = Capacitor::make("c_1");
	Matrix c_param = Matrix::Zero(3, 3);
	c_param <<
		0.001, 0, 0,
		0, 0.001, 0,
		0, 0, 0.001;
	c1->setParameters(c_param);

	// Topology
	vs->connect({ Node::GND, n1 });
	r1->connect({ n1, n2 });
	c1->connect({ n2, Node::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, r1, c1});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::DP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();

	return 0;
}
