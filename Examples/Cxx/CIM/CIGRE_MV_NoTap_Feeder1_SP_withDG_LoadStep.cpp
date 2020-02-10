/**
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
using namespace CPS::SP;
using namespace CPS::SP::Ph1;
//
//
/*
 * This example runs the MNA solver for the CIGRE MV benchmark system with dynamic phasor representation
 *  (neglecting the tap changers of the transformers)
 */
 //
void addDGtoSystem(std::shared_ptr<CPS::Node<Complex>> node_DG,
	std::shared_ptr<Ph1::AvVoltageSourceInverterDQ>DistGen,
	String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system, Real Vnom_DG_node, Bool withTrafo = true) {
	//auto node_DG = CPS::Node<Complex>::make(nodeName+"_DG");
	Real system_freq = 50;

	Real lf_param = 0.002;
	Real cf_param = 789.3e-6;
	Real rf_param = 0.01;
	Real rc_param = 0.05;

	//auto DistGen = Ph1::AvVoltageSourceInverterDQ::make(DERType+"_"+nodeName, Logger::Level::info);
	DistGen->setParameters(2 * PI * 50, Complex(Vnom_DG_node, 0),
		// controller parameters
		Pref, Qref, 0.25, 2, 0.001, 0.08, 0.3, 10,
		// filter parameters
		lf_param, cf_param, rf_param, rc_param);

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

// DP switch is ok with SP
void addLoadStepToSystem(std::shared_ptr<DP::Ph1::Switch> loadStep,
	Real addedPowerAsResistance,
	String nodeName,
	SystemTopology& system) {
	loadStep->setParameters(1e9, addedPowerAsResistance);
	for (auto baseNode : system.mNodes) {
		if (baseNode->name() == nodeName) {
			auto node_sys = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
			loadStep->connect({ CPS::Node<Complex>::GND, node_sys });
			loadStep->open();
			loadStep->initialize(system.mFrequencies);
			system.mComponents.push_back(loadStep);
			system.mComponentsAtNode[node_sys].push_back(loadStep);
			break;
		}
	}
}

int main(int argc, char** argv) {
	Real timeStep = 100e-6;
	Real finalTime = 10;
	// Find CIM files
	std::list<fs::path> filenames;
	std::list<fs::path> csvFilenames;

	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
			}, "Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");

		//csvFilenames = DPsim::Utils::findFiles({
		//	"PV_profile_MV_SOGNO-cloudlevel-1.csv",
		//	}, "../../../Examples/CSV", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "SP_100Pen_FaultN2_100e-6";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;
	Real penetrationLevel = 1;

	CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	// network configuration
	std::vector<Real> pvGenProfile{ 50000., 50000. , 50000. };
	/*CSVReader csvReader(simName, csvFilenames, Logger::Level::info);
	std::vector<Real> pvGenProfile =
		csvReader.readPQData(csvFilenames.front(),
			36000, 0.001, 36000 + 21600, CSVReader::DataFormat::SECONDS);*/
			// total load: 4319 kW, PV Gen: 400 kW per PV unit
			// it is hard-coded here because we specified a feeder zone
	Real total_load = 5000. * 1000.;
	Real PeakGen = 50000.;
	Int pv_units_number = Int(total_load * penetrationLevel / PeakGen);
	// 10 nodes in Feeder 1
	Int pv_units_per_node = pv_units_number / 9;

	Real power_factor_pv = 1;

	if (pv_units_per_node == 0)
		pv_units_per_node = 1;

	std::transform(pvGenProfile.begin(), pvGenProfile.end(), pvGenProfile.begin(),
		std::bind1st(std::multiplies<Real>(), pv_units_per_node));

	std::vector<shared_ptr<Ph1::AvVoltageSourceInverterDQ>> vsis;

	// add PVs
	for (Int n = 3; n <= 11; n++) {
		auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG");
		auto pv_at_node = Ph1::AvVoltageSourceInverterDQ::make("pv_n" + std::to_string(n), Logger::Level::off);
		//pv_at_node->addGenProfile(&pvGenProfile);
		Real Vnom_pv = 1500.;
		Real Pref_pv_at_node = (pvGenProfile.empty()) ? 400000 : pvGenProfile.front();
		Real Qref_pv_at_node = sqrt(std::pow(Pref_pv_at_node / power_factor_pv, 2) - std::pow(Pref_pv_at_node, 2));;
		addDGtoSystem(node_DG, pv_at_node, "N" + std::to_string(n), "pv", Pref_pv_at_node, Qref_pv_at_node,
			systemSP, Vnom_pv, true);
		vsis.push_back(pv_at_node);
		std::cout << " add pv to node N" << std::to_string(n) << " with Pref: " << Pref_pv_at_node << " W." << std::endl;
	}
	// Events
	auto fault_n2 = DP::Ph1::Switch::make("fault_n2", Logger::Level::info);
	addLoadStepToSystem(fault_n2, 10, "N2", systemSP);
	auto sw1 = SwitchEvent::make(7-timeStep, fault_n2, true);
	auto sw2 = SwitchEvent::make(7.1 - timeStep, fault_n2, false);

	auto loggerPF = DPsim::DataLogger::make(simName + ".pf");
	for (auto node : systemSP.mNodes)
	{
		loggerPF->addAttribute(node->name() + ".V", node->attribute("v"));
	}
	Simulation simPF(simName + ".pf", systemSP, 1, 2, Domain::SP, Solver::Type::NRP, Logger::Level::off, true);
	simPF.addLogger(loggerPF);
	simPF.run();

	for (auto node : systemSP.mNodes) {
		node->setInitialVoltage(
			std::dynamic_pointer_cast<CPS::Node<CPS::Complex>>((node))->singleVoltage());
	}
	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : systemSP.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}

	auto logger_pv11 = DPsim::DataLogger::make(simName + "_pvn11");

	for (auto pv : vsis) {
		if (pv->name() == "pv_n11") {
			logger_pv11->addAttribute("p", pv->attribute("p"));
			logger_pv11->addAttribute("q", pv->attribute("q"));
			logger_pv11->addAttribute("P_ref", pv->attribute("P_ref"));
			logger_pv11->addAttribute("Q_ref", pv->attribute("Q_ref"));
			logger_pv11->addAttribute("Vsdq", pv->attribute("Vsdq"));
			logger_pv11->addAttribute("igdq", pv->attribute("igdq"));
			logger_pv11->addAttribute("ifdq", pv->attribute("ifdq"));
			logger_pv11->addAttribute("Vcdq", pv->attribute("Vcdq"));
			logger_pv11->addAttribute("freq", pv->attribute("freq"));
			logger_pv11->addAttribute("i", pv->attribute("i_intf"));
			break;
		}
	}
	auto logger_l10_11 = DPsim::DataLogger::make(simName + "_L10_11");
	for (auto line : systemSP.mComponents) {
		if (shared_ptr<Ph1::PiLine> l10_11 = std::dynamic_pointer_cast<Ph1::PiLine>(line)) {
			if (l10_11->name() == "L11-10") {
				logger_l10_11->addAttribute("i_L10_11", l10_11->attribute("i_intf"));
				break;
			}
		}
	}
	auto logger_load_11 = DPsim::DataLogger::make(simName + "_Load_11");
	for (auto load : systemSP.mComponents) {
		if (shared_ptr<Ph1::Load> load11 = std::dynamic_pointer_cast<Ph1::Load>(load)) {
			if (load11->name() == "LOAD-H-11") {
				logger_load_11->addAttribute("i_Load_11", load11->attribute("i_intf"));
				break;
			}
		}
	}

	Simulation sim(simName, Logger::Level::off);
	sim.setDomain(CPS::Domain::SP);
	sim.setSystem(systemSP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	//sim.doSteadyStateInit(true);
	sim.doHarmonicParallelization(false);
	// add Events
	sim.addEvent(sw1);
	sim.addEvent(sw2);


	sim.addLogger(logger);
	sim.addLogger(logger_pv11);
	sim.addLogger(logger_l10_11);

	sim.run();

	return 0;
}

