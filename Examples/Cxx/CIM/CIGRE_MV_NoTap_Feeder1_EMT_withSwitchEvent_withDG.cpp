/*
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

#include "cps/CIM/Reader.h"
#include <DPsim.h>
#include <cps/CSVReader.h>


using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))


/*
 * This example runs the MNA solver for the CIGRE MV benchmark system with dynamic phasor representation
 *  (neglecting the tap changers of the transformers)
 */

void addDGtoSystem(std::shared_ptr<CPS::Node<Real>> node_DG,
	std::shared_ptr<Ph3::AvVoltageSourceInverterDQ> DistGen,
	String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system, Real Vnom_DG_node, Bool withTrafo = true) {

	Matrix rf_param = Matrix::Zero(3, 3);
	rf_param <<
		0.01, 0, 0,
		0, 0.01, 0,
		0, 0, 0.01;

	Matrix lf_param = Matrix(3, 3);
	lf_param <<
		0.002, 0, 0,
		0, 0.002, 0,
		0, 0, 0.002;

	Matrix cf_param = Matrix(3, 3);
	cf_param <<
		789.3e-6, 0, 0,
		0, 789.3e-6, 0,
		0, 0, 789.3e-6;

	Matrix rc_param = Matrix::Zero(3, 3);
	rc_param <<
		0.05, 0, 0,
		0, 0.05, 0,
		0, 0, 0.05;
	//auto DistGen = Ph3::AvVoltageSourceInverterDQ::make(DERType+"_"+nodeName, Logger::Level::info);
	DistGen->setParameters(2 * PI * 50, Complex(Vnom_DG_node, 0),
		// controller parameters
		Pref, Qref, 0.25, 2, 0.001, 0.08, 0.3, 10,
		// filter parameters
		lf_param, cf_param, rf_param, rc_param);

	MatrixComp node_DG_vInit = MatrixComp::Zero(3,1);

	node_DG_vInit <<
		Complex(Vnom_DG_node, 0),
		Complex(Vnom_DG_node, 0)* SHIFT_TO_PHASE_B,
		Complex(Vnom_DG_node, 0)* SHIFT_TO_PHASE_C;

	if (withTrafo) {

		node_DG->setInitialVoltage(node_DG_vInit);

		auto trans_DG = Ph3::Transformer::make("trans_" + DERType + "_" + nodeName, Logger::Level::off);

		trans_DG->setParameters(20000. / Vnom_DG_node, 0., Matrix::Zero(3,3), lf_param);
		//trans_DG->setBaseVoltages(20000., 220.);

		for (auto baseNode : system.mNodes) {
			if (baseNode->name() == nodeName) {
				auto node_sys = std::dynamic_pointer_cast<CPS::Node<Real>>(baseNode);
				trans_DG->connect({ node_sys, node_DG });
				system.mComponentsAtNode[baseNode].push_back(trans_DG);
				break;
			}
		}	
		DistGen->connect({ node_DG });
		//DistGen->initialize(system.mFrequencies);
		//trans_DG->initialize(system.mFrequencies);
		node_DG->initialize(system.mFrequencies);
		system.mNodes.push_back(node_DG);
		system.mComponents.push_back(trans_DG);
		system.mComponents.push_back(DistGen);
		system.mComponentsAtNode[node_DG].push_back(DistGen);
	}
	else {
		for (auto baseNode : system.mNodes) {
			if (baseNode->name() == nodeName) {
				auto node_sys = std::dynamic_pointer_cast<CPS::Node<Real>>(baseNode);
				DistGen->connect({ node_sys });
				system.mComponentsAtNode[baseNode].push_back(DistGen);
				break;
			}
		}
		//DistGen->initialize(system.mFrequencies);
		system.mComponents.push_back(DistGen);
	}
}

void addLoadStepToSystem(std::shared_ptr<EMT::Ph3::Switch> loadStep,
	Matrix addedPowerAsResistance,
	String nodeName,
	SystemTopology& system)
{
	Matrix openResistance = Matrix::Zero(3, 3);
	openResistance <<
		1e9, 0, 0,
		0, 1e9, 0,
		0, 0, 1e9;

	loadStep->setParameters(openResistance, addedPowerAsResistance);
	for (auto baseNode : system.mNodes) {
		if (baseNode->name() == nodeName) {
			auto node_sys = std::dynamic_pointer_cast<CPS::Node<Real>>(baseNode);
			loadStep->connect({ CPS::Node<Real>::GND, node_sys });
			loadStep->openSwitch();
			loadStep->initialize(system.mFrequencies);
			system.mComponents.push_back(loadStep);
			system.mComponentsAtNode[node_sys].push_back(loadStep);
			break;
		}
	}
}

namespace CPS {
	namespace SP {

		void addDGtoSystem(std::shared_ptr<CPS::Node<Complex>> node_DG,
			std::shared_ptr<Ph1::AvVoltageSourceInverterDQ>DistGen,
			String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system,
			Real Vnom_DG_node, Bool withTrafo = true) {
			Real system_freq = 50;

			DistGen->setParameters(2 * PI * 50, Complex(Vnom_DG_node, 0),
				// controller parameters
				Pref, Qref, 0.25, 2, 0.001, 0.08, 3.77, 1400,
				// filter parameters
				0.928e-3, 789e-6, 0.01, 0.5);
			if (withTrafo) {
					node_DG->setInitialVoltage(Complex(Vnom_DG_node, 0));
					auto trans_DG = Ph1::Transformer::make("trans_" + DERType + "_" + nodeName, Logger::Level::info);
					trans_DG->setParameters(20000., Vnom_DG_node, 5e6, 20000. / Vnom_DG_node, 0., 0, 0.928e-3, system_freq);
					trans_DG->setBaseVoltage(20000.);

					for (auto baseNode : system.mNodes) {
						if (baseNode->name() == nodeName) {
							auto node_sys = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
							trans_DG->connect({ node_sys, node_DG });
							system.mComponentsAtNode[node_DG].push_back(trans_DG);
							break;
						}
					}
					DistGen->connect({ node_DG });
					DistGen->initialize(system.mFrequencies);
					trans_DG->initialize(system.mFrequencies);

					node_DG->initialize(system.mFrequencies);
					system.mNodes.push_back(node_DG);
					system.mComponents.push_back(trans_DG);
					system.mComponents.push_back(DistGen);
					system.mComponentsAtNode[node_DG].push_back(DistGen);
			}
			else {
				for (auto baseNode : system.mNodes) {
					if (baseNode->name() == nodeName) {
						auto node_sys = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
						DistGen->connect({ node_sys });
						system.mComponentsAtNode[baseNode].push_back(DistGen);
						break;
					}
				}
				//DistGen->initialize(system.mFrequencies);
				system.mComponents.push_back(DistGen);
			}
		}



		SystemTopology calculatePowerFlow(String simName, Real system_freq, std::list<fs::path> filenames,
			Real penetrationLevel, std::vector<Real>& pvGenProfile) {
			CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
			SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);


			// network configuration
					// total load: 4319 kW, PV Gen: 400 kW per PV unit
					// it is hard-coded here because we specified a feeder zone
			Real power_factor_pv = 1;
			// add PVs
			for (Int n = 3; n <= 11; n++) {
				auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG");
				auto pv_at_node = Ph1::AvVoltageSourceInverterDQ::make("pv_n" + std::to_string(n), Logger::Level::off);
				//pv_at_node->addGenProfile(&pvGenProfile);
				Real V_nom_DG = 1500.;
				Real Pref_pv_at_node = (pvGenProfile.empty()) ? 400000 : pvGenProfile.front() ;
				Real Qref_pv_at_node = sqrt(std::pow(Pref_pv_at_node / power_factor_pv, 2) - std::pow(Pref_pv_at_node, 2));;
				addDGtoSystem(node_DG, pv_at_node, "N" + std::to_string(n), "pv",
					Pref_pv_at_node, Qref_pv_at_node, systemSP, V_nom_DG);
			}


			auto loggerPF = DPsim::DataLogger::make(simName + ".pf");
			for (auto node : systemSP.mNodes)
			{
				loggerPF->addAttribute(node->name() + ".V", node->attribute("v"));
			}
			Simulation simPF(simName + ".pf", systemSP, 1, 2, Domain::SP, Solver::Type::NRP, Logger::Level::info, true);
			simPF.addLogger(loggerPF);
			simPF.run();

			return systemSP;

		}
	}
}


int main(int argc, char** argv) {
	Real timeStep = 1e-3;
	Real finalTime = 20;
	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
			}, "Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "EMT_100Pen_ctrlEvents2_1e-3";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;
	Real penetrationLevel = 1;
	std::vector<Real> pvGenProfile{ 50000., 50000. , 50000. };
	Real PeakGen = 50000.;

	Real total_load = 5000. * 1000.;
	Int pv_units_number = Int(total_load * penetrationLevel / PeakGen);
	// 10 nodes in Feeder 1
	Int pv_units_per_node = pv_units_number / 9;
	Real power_factor_pv = 1;
	std::transform(pvGenProfile.begin(), pvGenProfile.end(), pvGenProfile.begin(),
		std::bind1st(std::multiplies<Real>(), pv_units_per_node));

	SystemTopology systemSP = CPS::SP::calculatePowerFlow(simName, system_freq, filenames, penetrationLevel, pvGenProfile);

	CIM::Reader readerEMT(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology systemEMT = readerEMT.loadCIM(system_freq, filenames, CPS::Domain::EMT, PhaseType::ABC);

	// extract initial voltages to DP system
	readerEMT.writeSvVoltageFromStaticSysTopology(systemSP, systemEMT);

	//CSVReader csvReader(simName, csvFilenames, Logger::Level::info);
	/*std::vector<Real> pvGenProfile =
		csvReader.readPQData(csvFilenames.front(),
			36000, timeStep, 36000 + finalTime, CSVReader::DataFormat::SECONDS);*/
			/*Real total_load = 0;
			for (auto comp : systemEMT.mComponents) {
				if (std::shared_ptr<DP::Ph3::RXLoad> load = std::dynamic_pointer_cast<DP::Ph3::RXLoad>(comp)) {
					total_load += load->attribute<Real>("P")->get();
				}
			}*/


	std::vector<shared_ptr<Ph3::AvVoltageSourceInverterDQ>> vsis;

	// add PVs
	for (Int n = 3; n <= 11; n++) {
		auto node_DG = CPS::Node<Real>::make("N" + std::to_string(n) + "_DG", PhaseType::ABC);
		auto pv_at_node = Ph3::AvVoltageSourceInverterDQ::make("pv_n" + std::to_string(n), Logger::Level::off);
		//pv_at_node->addGenProfile(&pvGenProfile);
		Real Vnom_pv = 1500.;
		Real Pref_pv_at_node = (pvGenProfile.empty()) ? 400000 : pvGenProfile.front();
		Real Qref_pv_at_node = sqrt(std::pow(Pref_pv_at_node / power_factor_pv, 2) - std::pow(Pref_pv_at_node, 2));;
		addDGtoSystem(node_DG, pv_at_node, "N" + std::to_string(n), "pv", Pref_pv_at_node, Qref_pv_at_node, systemEMT, Vnom_pv);
		vsis.push_back(pv_at_node);
		std::cout << " add pv to node N" << std::to_string(n) << " with Pref: " << Pref_pv_at_node << " W." << std::endl;
	}


	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : systemEMT.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}
	auto logger_pv11 = DPsim::DataLogger::make(simName + "_pvn11");
	for (auto vsi : vsis) {
		if (shared_ptr<Ph3::AvVoltageSourceInverterDQ> pv = std::dynamic_pointer_cast<Ph3::AvVoltageSourceInverterDQ>(vsi)) {
			if (pv->name() == "pv_n11") {
				logger_pv11->addAttribute("p", pv->attribute("p"));
				logger_pv11->addAttribute("q", pv->attribute("q"));
				logger_pv11->addAttribute("P_ref", pv->attribute("P_ref"));
				logger_pv11->addAttribute("Q_ref", pv->attribute("Q_ref"));
				logger_pv11->addAttribute("Vsdq", pv->attribute("Vsdq"));
				logger_pv11->addAttribute("igdq", pv->attribute("igdq"));
				logger_pv11->addAttribute("Vcdq", pv->attribute("Vcdq"));
				logger_pv11->addAttribute("freq", pv->attribute("freq"));
				logger_pv11->addAttribute("i", pv->attribute("i_intf"));
				break;
			}
		}
	}


	auto logger_l10_11 = DPsim::DataLogger::make(simName + "_L10_11");
	for (auto line : systemEMT.mComponents) {
		if (shared_ptr<Ph3::PiLine> l10_11 = std::dynamic_pointer_cast<Ph3::PiLine>(line)) {
			if (l10_11->name()=="L11-10") {
				logger_l10_11->addAttribute("i_L10_11", l10_11->attribute("i_intf"));
				break;
			}
		}
	}
	auto logger_load_11 = DPsim::DataLogger::make(simName + "_Load_11");
	for (auto load : systemEMT.mComponents) {
		if (shared_ptr<Ph3::RXLoad> load11 = std::dynamic_pointer_cast<Ph3::RXLoad>(load)) {
			if (load11->name() == "LOAD-H-11") {
				logger_load_11->addAttribute("i_Load_11", load11->attribute("i_intf"));
				break;
			}
		}
	}

	auto stepLoad_n11 = EMT::Ph3::Switch::make("StepLoad_n11", Logger::Level::info);
	Matrix closeResistance = Matrix::Zero(3, 3);
	closeResistance <<
		2353, 0, 0,
		0, 2353, 0,
		0, 0, 2353;
	addLoadStepToSystem(stepLoad_n11, closeResistance, "N11", systemEMT);
	auto sw1 = SwitchEvent3Ph::make(7-timeStep, stepLoad_n11, true);
	auto sw2 = SwitchEvent3Ph::make(7.1, stepLoad_n11, false);

	//Simulation sim(simName, system, 1, 120, Domain::DP, Solver::Type::MNA, Logger::Level::info, true);
	Simulation sim(simName, Logger::Level::off);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	//sim.doSteadyStateInit(true);
	sim.doHarmonicParallelization(false);

	sim.addEvent(sw1);
	sim.addEvent(sw2);

	sim.addLogger(logger);
	sim.addLogger(logger_pv11);
	sim.addLogger(logger_l10_11);
	/*
	sim.addLogger(logger_load_11);
	*/
	sim.run();

	return 0;
}

