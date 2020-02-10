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
using namespace CPS::DP;
using namespace CPS::DP::Ph1;
//
//
/*
 * This example runs the MNA solver for the CIGRE MV benchmark system with dynamic phasor representation
 *  (neglecting the tap changers of the transformers)
 */
//
void addDGtoSystem(std::shared_ptr<CPS::Node<Complex>> node_DG,
	std::shared_ptr<Ph1::AvVoltageSourceInverterDQ> DistGen,
	String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system, Real Vnom_DG_node, Bool withTrafo = true) {

	Real lf_param = 0.928e-3;
	Real cf_param = 789.3e-6;
	Real rf_param = 0.01;
	Real rc_param = 0.05;
	DistGen->setParameters(2 * PI * 50, Complex(Vnom_DG_node, 0),
		// controller parameters
		Pref, Qref, 0.25, 2, 0.001, 0.08, 0.3, 10,
		// filter parameters
		0.002, cf_param, rf_param, rc_param);

	if (withTrafo) {
		node_DG->setInitialVoltage(Complex(Vnom_DG_node, 0));

		auto trans_DG = Ph1::Transformer::make("trans_" + DERType + "_" + nodeName, Logger::Level::off);
		trans_DG->setParameters(20000. / Vnom_DG_node, 0., 0., lf_param);
		//trans_DG->setBaseVoltages(20000., 220.);

		for (auto baseNode : system.mNodes) {
			if (baseNode->name() == nodeName) {
				auto node_sys = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
				trans_DG->connect({ node_sys, node_DG });
				system.mComponentsAtNode[baseNode].push_back(trans_DG);
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
		DistGen->initialize(system.mFrequencies);
		system.mComponents.push_back(DistGen);
	}
}


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
namespace CPS {
	namespace SP {

		void addDGtoSystem(std::shared_ptr<CPS::Node<Complex>> node_DG,
			std::shared_ptr<Ph1::AvVoltageSourceInverterDQ>DistGen,
			String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system,
			Real Vnom_DG_node, Bool withTrafo = true) {
			//auto node_DG = CPS::Node<Complex>::make(nodeName+"_DG");
			Real system_freq=50;
			//Real Vnom_DG_node = 800.;
			//auto DistGen = Ph1::AvVoltageSourceInverterDQ::make(DERType+"_"+nodeName, Logger::Level::info);
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
			std::list<fs::path> loadFilenames,
		 Real penetrationLevel, std::vector<Real>& pvGenProfile, Real power_factor_pv) {
			CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
			SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);


			// network configuration
			//CSVReader csvReader(simName, csvFilenames, Logger::Level::info);
			/*std::vector<Real> pvGenProfile =
				csvReader.readPQData(csvFilenames.front(),
					36000, timeStep, 36000 + finalTime, CSVReader::DataFormat::SECONDS);*/
					/*Real total_load = 0;
					for (auto comp : systemDP.mComponents) {
						if (std::shared_ptr<DP::Ph1::RXLoad> load = std::dynamic_pointer_cast<DP::Ph1::RXLoad>(comp)) {
							total_load += load->attribute<Real>("P")->get();
						}
					}*/

					// total load: 4319 kW, PV Gen: 400 kW per PV unit
					// it is hard-coded here because we specified a feeder zone

			std::vector<shared_ptr<Ph1::AvVoltageSourceInverterDQ>> vsis;
			std::vector<shared_ptr<Ph1::AvVoltageSourceInverterDQ>> loads;
		// add PVs
			for (Int n = 7; n <= 11; n++) {
				auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG");
				auto pv_at_node = Ph1::AvVoltageSourceInverterDQ::make("pv_n" + std::to_string(n), Logger::Level::off);
				pv_at_node->addGenProfile(&pvGenProfile);
				Real Vnom_pv = 1500.;
				Real Pref_pv_at_node = (pvGenProfile.empty()) ? 550000 : pvGenProfile.front();
				Real Qref_pv_at_node = sqrt(std::pow(Pref_pv_at_node / power_factor_pv, 2) - std::pow(Pref_pv_at_node, 2));;
				addDGtoSystem(node_DG, pv_at_node, "N" + std::to_string(n), "pv", Pref_pv_at_node, Qref_pv_at_node, systemSP, Vnom_pv);
				vsis.push_back(pv_at_node);
				std::cout << " add pv to node N" << std::to_string(n) << " with Pref: " << Pref_pv_at_node << " W." << std::endl;
			}

			for (Int n = 3; n <= 11; n++) {
				auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG_load");
				auto load_at_node = Ph1::AvVoltageSourceInverterDQ::make("load_n" + std::to_string(n), Logger::Level::off);
				load_at_node->coveeCtrled(false);
				load_at_node->makeLoad(true);
				// load_at_node->setProfileUpdateRate(UInt(n*100));
				//load_at_node->addGenProfile(&load_profile);
				Real Vnom_load = 1500.;
				Real Pref_load_at_node = -20000;
				Real Qref_load_at_node = 0;
				addDGtoSystem(node_DG, load_at_node, "N" + std::to_string(n), "load", Pref_load_at_node, Qref_load_at_node, systemSP, Vnom_load, true);
				loads.push_back(load_at_node);
				std::cout << " add controled load to node N" << std::to_string(n) << " with Pref: " << Pref_load_at_node << " W." << std::endl;
			}
			std::map<String,String> assignList = {
			// {assignee mRID, file name}
			{"load_n3", "Load_H_3"},
			{"load_n4", "Load_H_4"},
			{"load_n5", "Load_H_5"},
			{"load_n6", "Load_H_6"},
			{"load_n8", "Load_H_8"},
			{"load_n10", "Load_H_10"},
			{"load_n11", "Load_H_11"},
			{"load_n12", "Load_H_12"},
			{"load_n14", "Load_H_14"},
			{"load_n7", "Load_I_7"},
			{"load_n9", "Load_I_9"}};

			Real load_scale =0.5;
			CSVReader loadProfileReader(simName, loadFilenames, assignList, Logger::Level::info);
			loadProfileReader.assignLoadProfileSP(loads, 61200+3600, 1, 61200+3600+3600, load_scale, CSVReader::Mode::MANUAL);

			// modify base load (constant impedance load)
			/*
			for (auto comp : systemDP.mComponents) {
				if (std::shared_ptr<DP::Ph1::RXLoad> load = std::dynamic_pointer_cast<DP::Ph1::RXLoad>(comp)) {
					if(load->name().find("11")==std::string::npos){
						load->terminals()[0]->setPower(Complex(0.5*load->terminals()[0]->singlePower().real(),0.5*load->terminals()[0]->singlePower().imag()));	
						std::cout<<"load at "<<load->name()<<" increased"<<std::endl;
					}
					else
					{
						load->terminals()[0]->setPower(Complex(0.2*load->terminals()[0]->singlePower().real(),0.2*load->terminals()[0]->singlePower().imag()));
					}
				}
			}
			*/
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

int main(int argc, char** argv){
	CommandLineArgs args(argc, argv);
	// Find CIM files
	std::list<fs::path> filenames;
	std::list<fs::path> pvGenFilenames;
	std::list<fs::path> loadFilenames;

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

	String simName = "DP_50Pen_loadStepN8_2_50ms";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;

	for (auto file: filenames){
		if(file.string().find("Load")!=std::string::npos)
			loadFilenames.push_back(file);
		if(file.string().find("PV_profile")!=std::string::npos)
			pvGenFilenames.push_back(file);
	}
	CSVReader csvReader(simName, pvGenFilenames, Logger::Level::info);
	 std::vector<Real> pvGenProfile =
	 	csvReader.readPQData(pvGenFilenames.back(),
	 		36000+7200, 0.001, 36000+7200+10800, CSVReader::DataFormat::SECONDS);

	// total load: 4319 kW, PV Gen: 4 kW per PV unit
	// it is hard-coded here because we specified a feeder zone
	Real penetrationLevel = 0.5;
	Real number_PVs = 5;
	Real PVGenPeak = 50000.;
	Real total_load = 5000. * 1000 *1.5;
	Int pv_units_number = Int(total_load * penetrationLevel / PVGenPeak);
	// 10 nodes in Feeder 1
	Int pv_units_per_node = pv_units_number / number_PVs;
	Real power_factor_pv = 1;

	std::transform(pvGenProfile.begin(), pvGenProfile.end(),pvGenProfile.begin(),
		std::bind1st(std::multiplies<Real>(),pv_units_per_node));

	SystemTopology systemSP = CPS::SP::calculatePowerFlow(simName, system_freq, filenames, loadFilenames,
	 	penetrationLevel, pvGenProfile, power_factor_pv);

	// doesn't work if not creating second reader object?
	CIM::Reader reader2(simName, Logger::Level::info, Logger::Level::off);
    SystemTopology systemDP = reader2.loadCIM(system_freq, filenames, CPS::Domain::DP);

	for(auto file : filenames)
		std::cout<<file<<std::endl;

	// extract initial voltages to DP system
	//reader2.writeSvVoltageFromStaticSysTopology(systemSP, systemDP);

	std::vector<shared_ptr<Ph1::AvVoltageSourceInverterDQ>> vsis;
	std::vector<shared_ptr<Ph1::AvVoltageSourceInverterDQ>> loads;

	// add PVs
	for (Int n = 7; n <= 11; n++) {
		auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG");
		auto pv_at_node = Ph1::AvVoltageSourceInverterDQ::make("pv_n" + std::to_string(n), Logger::Level::off);
		pv_at_node->addGenProfile(&pvGenProfile);
		Real Vnom_pv = 1500.;
		Real Pref_pv_at_node = (pvGenProfile.empty()) ? 550000 : pvGenProfile.front();
		Real Qref_pv_at_node = sqrt(std::pow(Pref_pv_at_node / power_factor_pv, 2) - std::pow(Pref_pv_at_node, 2));;
		addDGtoSystem(node_DG, pv_at_node, "N" + std::to_string(n), "pv", Pref_pv_at_node, Qref_pv_at_node, systemDP, Vnom_pv);
		vsis.push_back(pv_at_node);
		std::cout << " add pv to node N" << std::to_string(n) << " with Pref: " << Pref_pv_at_node << " W." << std::endl;
	}

	for (Int n = 3; n <= 11; n++) {
		auto node_DG = CPS::Node<Complex>::make("N" + std::to_string(n) + "_DG_load");
		auto load_at_node = Ph1::AvVoltageSourceInverterDQ::make("load_n" + std::to_string(n), Logger::Level::off);
		load_at_node->coveeCtrled(false);
		load_at_node->makeLoad(true);
		load_at_node->setProfileUpdateRate(UInt(998));
		//load_at_node->addGenProfile(&load_profile);
		Real Vnom_load = 1500.;
		Real Pref_load_at_node = -20000;
		Real Qref_load_at_node = 0;
		addDGtoSystem(node_DG, load_at_node, "N" + std::to_string(n), "load", Pref_load_at_node, Qref_load_at_node, systemDP, Vnom_load, true);
		loads.push_back(load_at_node);
		std::cout << " add controled load to node N" << std::to_string(n) << " with Pref: " << Pref_load_at_node << " W." << std::endl;
	}
	std::map<String,String> assignList = {
	// {assignee mRID, file name}
	{"load_n3", "Load_H_3"},
	{"load_n4", "Load_H_4"},
	{"load_n5", "Load_H_5"},
	{"load_n6", "Load_H_6"},
	{"load_n8", "Load_H_8"},
	{"load_n10", "Load_H_10"},
	{"load_n11", "Load_H_11"},
	{"load_n12", "Load_H_12"},
	{"load_n14", "Load_H_14"},
	{"load_n7", "Load_I_7"},
	{"load_n9", "Load_I_9"}};

	Real load_scale =0.5;
	CSVReader loadProfileReader(simName, loadFilenames, assignList, Logger::Level::info);
	loadProfileReader.assignLoadProfileDP(loads, 61200+3600, 1, 61200+3600+3600, load_scale, CSVReader::Mode::MANUAL);

	// modify base load (constant impedance load)
	/*
	for (auto comp : systemDP.mComponents) {
		if (std::shared_ptr<DP::Ph1::RXLoad> load = std::dynamic_pointer_cast<DP::Ph1::RXLoad>(comp)) {
			if(load->name().find("11")==std::string::npos){
				load->terminals()[0]->setPower(Complex(0.5*load->terminals()[0]->singlePower().real(),0.5*load->terminals()[0]->singlePower().imag()));	
				std::cout<<"load at "<<load->name()<<" increased"<<std::endl;
			}
			else
			{
				load->terminals()[0]->setPower(Complex(0.2*load->terminals()[0]->singlePower().real(),0.2*load->terminals()[0]->singlePower().imag()));
			}
		}
	}
	 */

	Interface intf("/dpsim1-villas", "/villas-dpsim1",nullptr,false,(UInt)50);
	Interface intf2("/dpsim2-villas", "/villas-dpsim2",nullptr,false,(UInt)50);
	Simulation sim(simName, args.logLevel);

	// add event
	/*
	auto fault_n2 = DP::Ph1::Switch::make("fault_n2", Logger::Level::info);
	addLoadStepToSystem(fault_n2, 10, "N2", systemDP);
	auto sw1 = SwitchEvent::make(100-2*args.timeStep, fault_n2, true);
	auto sw2 = SwitchEvent::make(103-2*args.timeStep, fault_n2, false);
	 */
	auto stepLoad_n8 = DP::Ph1::Switch::make("StepLoad_n8", Logger::Level::info);
	addLoadStepToSystem(stepLoad_n8, 1333, "N8", systemDP);
	auto sw1 = SwitchEvent::make(100-2*args.timeStep, stepLoad_n8, true);
	auto sw2 = SwitchEvent::make(110-2*args.timeStep, stepLoad_n8, false);

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : systemDP.mNodes) {
		UInt i;
		if(n->name().find("DG")!= std::string::npos)
			continue;
		if (sscanf(n->name().c_str(), "N%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		auto n_stat = std::dynamic_pointer_cast<CPS::DP::Node>(n);
		auto vMag = n_stat->attributeMatrixComp("v")->coeffMag(0, 0);
		auto vPhase = n_stat->attributeMatrixComp("v")->coeffPhase(0, 0);

		//auto v = n->attributeMatrixComp("v")->coeff(0, 0);
		//std::cout << "Signal " << i << ": " << n->name() << std::endl;

		// try
		//intf.exportReal(vmag, i); o++;

		std::cout << "Signal " << (i*2)+0 << ": Mag  " << n->name() << std::endl;
		std::cout << "Signal " << (i*2)+1 << ": Phase " << n->name() << std::endl;

		intf.exportReal(vMag, (i*2)+0); o++;
		intf.exportReal(vPhase, (i*2)+1); o++;
	}
		// continue with the same o
	for(auto pv:vsis){
		auto q_out=pv->attribute<Real>("q");
		std::cout << "Signal " << o << ": Q   " << pv->name() << std::endl;
		intf.exportReal(q_out, o); o++;

	}


	o = 0;
	for(auto pv : vsis){
		intf2.exportReal(pv->attribute<Real>("Q_ref"), UInt(o));
		std::cout << "Signal " << o << ": " << "Q_ref_"+ pv->name() << std::endl;o++;
	}


    // Register power control imports
	UInt n_pv=0;
	for(auto pv: vsis){
		pv->ctrlReceiver(intf2.importReal(n_pv));
		std::cout << "Add sample recipient at position" << n_pv << ": " << pv->name() << std::endl;
		n_pv++;
	}

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : systemDP.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}
	
	// log P
	auto logger_p = DPsim::DataLogger::make(simName + "_p");
	for (auto pv : vsis) {
		logger_p->addAttribute("p_"+pv->name(), pv->attribute("p"));
	}

	// log Q
	auto logger_q = DPsim::DataLogger::make(simName + "_q");
	for (auto pv : vsis) {
		logger_q->addAttribute("q"+pv->name(), pv->attribute("q"));
	}
	// log set points
	auto logger_ref = DPsim::DataLogger::make(simName + "_ref");
	for (auto pv : vsis) {
		logger_ref->addAttribute("P_ref"+pv->name(), pv->attribute("P_ref"));
		logger_ref->addAttribute("Q_ref"+pv->name(), pv->attribute("Q_ref"));
	}

	auto logger_load = DPsim::DataLogger::make(simName + "_load");
	for (auto load : loads) {
		logger_load->addAttribute("p"+load->name(), load->attribute("p"));
		logger_load->addAttribute("q"+load->name(), load->attribute("q"));
		logger_load->addAttribute("P_ref"+load->name(), load->attribute("P_ref"));
		logger_load->addAttribute("Q_ref"+load->name(), load->attribute("Q_ref"));
		break;
	}


	sim.addInterface(&intf, false);
	sim.addInterface(&intf2, false);
	sim.addLogger(logger);
	sim.addEvent(sw1);
	sim.addEvent(sw2);
	sim.addLogger(logger_p);
	sim.addLogger(logger_q);
	sim.addLogger(logger_ref);
	sim.addLogger(logger_load);
	sim.setSystem(systemDP);
	sim.setTimeStep(args.timeStep);
	sim.setFinalTime(args.duration);
	sim.setDomain(args.solver.domain);
	sim.doSteadyStateInit(true);
	//sim.run(std::chrono::seconds(5));
	sim.run();
	return 0;
}
