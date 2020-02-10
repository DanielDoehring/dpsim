/** Reference Circuits
 *
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


#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph3;

int main(int argc, char* argv[]) {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.2;
	String simName = "DP_Ph3_VSI2_4BUS_SampleGrid_FilterIntegrated";
	Logger::setLogDir("logs/" + simName);

	Real Vnom = 220.;
	// Nodes
	std::vector<Complex> initialVoltage{ Complex(Vnom, 0), Complex(Vnom, 0), Complex(Vnom, 0) };


	auto n1 = Node::make("n1", PhaseType::ABC, initialVoltage);
	//auto n2 = Node::make("n2", PhaseType::ABC, initialVoltage);
	auto n3 = Node::make("n3", PhaseType::ABC, initialVoltage);
	auto n4 = Node::make("n4", PhaseType::ABC, initialVoltage);
	auto n5 = Node::make("n5", PhaseType::ABC, initialVoltage);
	auto n6 = Node::make("n6", PhaseType::ABC, initialVoltage);
	auto n7 = Node::make("n7", PhaseType::ABC, initialVoltage);
	auto n8 = Node::make("n8", PhaseType::ABC, initialVoltage);
	//auto n9 = Node::make("n9", PhaseType::ABC, initialVoltage);
	auto n10 = Node::make("n10", PhaseType::ABC, initialVoltage);
	auto n11 = Node::make("n11", PhaseType::ABC, initialVoltage);
	auto n12 = Node::make("n12", PhaseType::ABC, initialVoltage);
	auto n13 = Node::make("n13", PhaseType::ABC, initialVoltage);


	// Components
	auto vsi = AvVoltageSourceInverterDQ::make("vsi");
	Real Pref1 = 3000;
	Real Qref1 = 200;
	Real Pref2 = 4000;
	Real Qref2 = 1000;

	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(Vnom, 0));
	auto vsi2 = AvVoltageSourceInverterDQ::make("vsi2");

	//auto rf = SeriesResistor::make("rf");
	Real rf_param = 0.01;
	//rf->setParameters(rf_param);

	//auto rf2 = SeriesResistor::make("rf2");
	//rf2->setParameters(rf_param);

	auto lf = Inductor::make("lf");
	Real lf_param = 0.02;
	lf->setParameters(lf_param);

	auto lf2 = Inductor::make("lf2");
	lf2->setParameters(lf_param);

	auto cf = Capacitor::make("cf");
	Matrix cf_param = Matrix(3, 1);
	cf_param << 1e-6, 1e-6, 1e-6;
	cf->setParameters(cf_param);

	auto cf2 = Capacitor::make("cf2");
	cf2->setParameters(cf_param);

	auto rc = SeriesResistor::make("rc");
	Real rc_param = 0.05;
	rc->setParameters(rc_param);

	auto rc2 = SeriesResistor::make("rc2");
	rc2->setParameters(rc_param);


	auto rline = SeriesResistor::make("rline");
	Real rline_param = 0.04;
	rline->setParameters(rline_param);

	auto rline2 = SeriesResistor::make("rline2");
	rline2->setParameters(rline_param);

	auto rline3 = SeriesResistor::make("rline3");
	rline3->setParameters(rline_param);

	auto rline4 = SeriesResistor::make("rline4");
	rline4->setParameters(rline_param);


	auto rload = SeriesResistor::make("rload");
	Real rload_param_1kw_220v = 47.6568;
	rload->setParameters(rload_param_1kw_220v);

	auto rload2 = SeriesResistor::make("rload2");
	rload2->setParameters(rload_param_1kw_220v);

	auto Lload = Inductor::make("Lload");
	Real Lload_param_100var_220v =  0.01516;
	Lload->setParameters(Lload_param_100var_220v);

	auto Lload2 = Inductor::make("Lload2");
	Lload2->setParameters(Lload_param_100var_220v);

	vsi->setParameters(2 * M_PI * 50, Complex(Vnom, 0), Pref1, Qref1, 0.25, 2, 0.001, 0.08, 3.77, 1400, lf_param, cf_param(0, 0), rf_param, rc_param);
	vsi2->setParameters(2 * M_PI * 50, Complex(Vnom, 0), Pref2, Qref2, 0.25, 2, 0.001, 0.08, 3.77, 1400, lf_param, cf_param(0, 0), rf_param, rc_param);

	// 4Bus case study
	vsi->connect(Node::List{ n1 });
	//rf->connect(Node::List{ n1, n2 });
	lf->connect(Node::List{ n1, n3 });
	cf->connect(Node::List{ n3, Node::GND });
	rc->connect(Node::List{ n3, n4 });

	vs->connect(Node::List{ Node::GND, n5 });
	rline->connect(Node::List{ n5, n4 });

	rline2->connect(Node::List{ n4, n6 });

	rload->connect(Node::List{ n6, n12 });
	Lload->connect(Node::List{ n12, Node::GND });

	rline3->connect(Node::List{ n6, n7 });

	vsi2->connect(Node::List{ n10 });
	//rf2->connect(Node::List{ n10, n9 });
	lf2->connect(Node::List{ n10, n8 });
	cf2->connect(Node::List{ n8, Node::GND });
	rc2->connect(Node::List{ n8, n7 });

	rline4->connect(Node::List{ n7, n11 });
	rload2->connect(Node::List{ n11, n13 });
	Lload2->connect(Node::List{ n13, Node::GND });

	vsi->addMonitoredNodes( cf, lf);
	vsi2->addMonitoredNodes( cf2, lf2);

	// Define system topology
	//auto sys = SystemTopology(50, SystemNodeList{ n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13 },
	//	SystemComponentList{ vsi, vs, rf, lf, cf, rc, rline, rline2, rline3, rline4, rload, rload2,
	//						Lload, Lload2, vsi2, rf2, lf2, cf2, rc2 });
	auto sys = SystemTopology(50, SystemNodeList{ n1, n3, n4, n5, n6, n7, n8, n10, n11, n12, n13 },
		SystemComponentList{ vsi, vs, lf, cf, rc, rline, rline2, rline3, rline4, rload, rload2,
							Lload, Lload2, vsi2, lf2, cf2, rc2 });


	// Logging
	auto logger = DataLogger::make(simName);
	// currents
	logger->addAttribute("i_vs", vs->attribute("i_intf"));
	logger->addAttribute("i_vsi", vsi->attribute("i_intf"));
	logger->addAttribute("i_vsi2", vsi2->attribute("i_intf"));
	logger->addAttribute("iload", rload->attribute("i_intf"));
	logger->addAttribute("iqload", Lload->attribute("i_intf"));
	// nodal voltages
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v_slack", n5->attribute("v"));
	logger->addAttribute("vBus1", n4->attribute("v"));
	logger->addAttribute("vBus2", n6->attribute("v"));
	logger->addAttribute("vBus3", n7->attribute("v"));
	logger->addAttribute("v8", n8->attribute("v"));
	logger->addAttribute("vBus4", n11->attribute("v"));
	// power outputs
	logger->addAttribute("P", vsi->attribute("p"));
	logger->addAttribute("Q", vsi->attribute("q"));
	logger->addAttribute("P2", vsi2->attribute("p"));
	logger->addAttribute("Q2", vsi2->attribute("q"));
	logger->addAttribute("P_ref", vsi->attribute("P_ref"));
	logger->addAttribute("Q_ref", vsi->attribute("Q_ref"));
	logger->addAttribute("P_ref2", vsi2->attribute("P_ref"));
	logger->addAttribute("Q_ref2", vsi2->attribute("Q_ref"));
	// states of controller
	logger->addAttribute("theta", vsi->attribute("theta"));
	logger->addAttribute("phi_pll", vsi->attribute("phipll"));
	logger->addAttribute("phid", vsi->attribute("phid"));
	logger->addAttribute("phiq", vsi->attribute("phiq"));
	logger->addAttribute("gammad", vsi->attribute("gammad"));
	logger->addAttribute("gammaq", vsi->attribute("gammaq"));
	// frequency
	logger->addAttribute("freq_vsi1", vsi->attribute("freq"));
	logger->addAttribute("freq_vsi2", vsi2->attribute("freq"));
	// output voltages
	logger->addAttribute("vsi", vsi->attribute("v_intf"));
	logger->addAttribute("vsi2", vsi2->attribute("v_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
