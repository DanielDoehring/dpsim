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

	String simName = "Shmem_WSCC-9bus";

	CIMReader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology sys = reader.loadCIM(60, filenames);


	RealTimeSimulation sim(simName, sys, 0.001, 120,
		Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);
	
	Interface intf("/dpsim-villas", "/villas-dpsim", nullptr, false);

	auto evs = VoltageSource::make("v_t");
	evs->connect({Node::GND, sys.node<Node>("BUS5")});
	evs->setParameters(Complex(0,0));

	sys.addComponent(evs);	

	DPsim::UInt o = 0;
	intf.exportComplex(evs->attributeMatrixComp("i_intf")->coeff(0, 0), o++);
	//intf.exportReal(sys.component<Ph1::RXLoad>("LOAD5")->attributeReal("P"), o++);
	
	// intf.exportReal(sys.node<Node>("BUS5")->attributeReal("v")->mag(), o++);
	// intf.exportReal(sys.node<Node>("BUS5")->attributeComplex("v")->phase(), o++);

	Logger::setLogDir("logs/"+simName);
	auto logger = DataLogger::make(simName);
	logger->addAttribute("DATA", evs->attribute("i_intf"));
	sim.addLogger(logger);

	//auto busAttribute = bus->attributeComplex("v");

	// intf.exportComplex(sys.node<Node>("BUS5")->attributeComplex("v"), o);

	
	// Do the export manually
	


	// Register exportable node voltages
	// UInt o = 0;
	// for (auto n : sys.mNodes) {
	// 	auto v = n->attributeComplex("v");

	// 	intf.exportReal(v->mag(),   o+0);
	// 	intf.exportReal(v->phase(), o+1);

	// 	o += 2;
	// }

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
