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
		String nodeName, String DERType, Real Pref, Real Qref, SystemTopology& system) {
		//auto node_DG = CPS::Node<Complex>::make(nodeName+"_DG");
		Real system_freq=50;
		Real Vnom_DG_node = 220;
		//auto DistGen = Ph1::AvVoltageSourceInverterDQ::make(DERType+"_"+nodeName, Logger::Level::info);
		DistGen->setParameters(2 * PI * 50, Complex(Vnom_DG_node, 0),
			// controller parameters
			Pref, Qref, 0.25, 2, 0.001, 0.08, 3.77, 1400,
			// filter parameters
			0.02, 1e-6, 0.01, 0.05);
		node_DG->setInitialVoltage(Complex(Vnom_DG_node, 0));
		auto trans_DG = Ph1::Transformer::make("trans_" + DERType + "_" + nodeName, Logger::Level::info);
		trans_DG->setParameters(20000., Vnom_DG_node, 5e6, 20000. / Vnom_DG_node, 0., 0, 0.928e-3, system_freq);
		trans_DG->setBaseVoltage(20000.);


		for (auto baseNode : system.mNodes) {
			if (baseNode->name() == nodeName) {
				auto node_sys = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
				trans_DG->connect({ node_sys, node_DG });
				system.mComponentsAtNode[baseNode].push_back(trans_DG);
				break;
			}
		}
		DistGen->connect({ node_DG });

		node_DG->initialize(system.mFrequencies);
		system.mNodes.push_back(node_DG);
		system.mComponents.push_back(trans_DG);
		system.mComponents.push_back(DistGen);
		system.mComponentsAtNode[node_DG].push_back(DistGen);
	}



int main(int argc, char** argv){
	CommandLineArgs args(argc, argv);
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

		csvFilenames = DPsim::Utils::findFiles({
			"PV_profile_MV_SOGNO-cloudlevel-1.csv",
			}, "../../../Examples/CSV", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "Shmem_CIGRE_MV_NoTap_Feeder1_withDG_PF_Covee";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;
	Real penetrationLevel = 1;


	CIM::Reader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology systemSP = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	
	// network configuration
	std::vector<Real> pvGenProfile{ 4000., 4000., 4000. };
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

			// total load: 4319 kW, PV Gen: 4 kW per PV unit
			// it is hard-coded here because we specified a feeder zone
	Real total_load = 4320;
	Int pv_units_number = Int(total_load * penetrationLevel / 4.);
	// 10 nodes in Feeder 1
	Int pv_units_per_node = pv_units_number / 10;
	Real power_factor_pv = 1;

	// ##### add PV to N2 ####
	auto n2_DG = CPS::Node<Complex>::make("N2_DG");
	auto pv_n2 = Ph1::AvVoltageSourceInverterDQ::make("pv_n2", Logger::Level::off);
	std::vector<Real> pv_profile_n2 = pvGenProfile;
	Real Pref_n2_pv = (pv_profile_n2.empty()) ? 20000 : pv_profile_n2.front() * pv_units_per_node;
	Real Qref_n2_pv = sqrt(std::pow(Pref_n2_pv / power_factor_pv, 2) - std::pow(Pref_n2_pv, 2));;
	addDGtoSystem(n2_DG, pv_n2, "N2", "pv", Pref_n2_pv, Qref_n2_pv, systemSP);
	pv_n2->addAggregatedGenProfile(&pv_profile_n2, pv_units_per_node);

	// ##### add PV to N3 ####
	auto n3_DG = CPS::Node<Complex>::make("N3_DG");
	auto pv_n3 = Ph1::AvVoltageSourceInverterDQ::make("pv_n3", Logger::Level::off);
	std::vector<Real> pv_profile_n3 = pvGenProfile;
	Real Pref_n3_pv = (pv_profile_n3.empty()) ? 20000 : pv_profile_n3.front() * pv_units_per_node;
	Real Qref_n3_pv = sqrt(std::pow(Pref_n3_pv / power_factor_pv, 2) - std::pow(Pref_n3_pv, 2));
	addDGtoSystem(n3_DG, pv_n3, "N3", "pv", Pref_n3_pv, Qref_n3_pv, systemSP);
	pv_n3->addAggregatedGenProfile(&pv_profile_n3, pv_units_per_node);

	// ##### add PV to N4 ####
	auto n4_DG = CPS::Node<Complex>::make("N4_DG");
	auto pv_n4 = Ph1::AvVoltageSourceInverterDQ::make("pv_n4", Logger::Level::off);
	std::vector<Real> pv_profile_n4 = pvGenProfile;
	Real Pref_n4_pv = (pv_profile_n4.empty()) ? 20000 : pv_profile_n4.front() * pv_units_per_node;
	Real Qref_n4_pv = sqrt(std::pow(Pref_n4_pv / power_factor_pv, 2) - std::pow(Pref_n4_pv, 2));
	addDGtoSystem(n4_DG, pv_n4, "N4", "pv", Pref_n4_pv, Qref_n4_pv, systemSP);
	pv_n4->addAggregatedGenProfile(&pv_profile_n4, pv_units_per_node);

	// ##### add PV to N5 ####
	auto n5_DG0 = CPS::Node<Complex>::make("N5_DG0");
	auto pv_n5 = Ph1::AvVoltageSourceInverterDQ::make("pv_n5", Logger::Level::off);
	std::vector<Real> pv_profile_n5 = pvGenProfile;
	Real Pref_n5_pv = (pv_profile_n5.empty()) ? 20000 : pv_profile_n5.front() * pv_units_per_node;
	Real Qref_n5_pv = sqrt(std::pow(Pref_n5_pv / power_factor_pv, 2) - std::pow(Pref_n5_pv, 2));
	addDGtoSystem(n5_DG0, pv_n5, "N5", "pv", Pref_n5_pv, Qref_n5_pv, systemSP);
	pv_n5->addAggregatedGenProfile(&pv_profile_n5, pv_units_per_node);

	// ##### add PV to N6 ####
	auto n6_DG0 = CPS::Node<Complex>::make("N6_DG0");
	auto pv_n6 = Ph1::AvVoltageSourceInverterDQ::make("pv_n6", Logger::Level::off);
	std::vector<Real> pv_profile_n6 = pvGenProfile;
	Real Pref_n6_pv = (pv_profile_n6.empty()) ? 20000 : pv_profile_n6.front() * pv_units_per_node;
	Real Qref_n6_pv = sqrt(std::pow(Pref_n6_pv / power_factor_pv, 2) - std::pow(Pref_n6_pv, 2));
	addDGtoSystem(n6_DG0, pv_n6, "N6", "pv", Pref_n6_pv, Qref_n6_pv, systemSP);
	pv_n6->addAggregatedGenProfile(&pv_profile_n6, pv_units_per_node);

	// ##### add PV to N7 ####
	auto n7_DG0 = CPS::Node<Complex>::make("N7_DG0");
	auto pv_n7 = Ph1::AvVoltageSourceInverterDQ::make("pv_n7", Logger::Level::off);
	std::vector<Real> pv_profile_n7 = pvGenProfile;
	Real Pref_n7_pv = (pv_profile_n7.empty()) ? 20000 : pv_profile_n7.front() * pv_units_per_node;
	Real Qref_n7_pv = sqrt(std::pow(Pref_n7_pv / power_factor_pv, 2) - std::pow(Pref_n7_pv, 2));
	addDGtoSystem(n7_DG0, pv_n7, "N7", "pv", Pref_n7_pv, Qref_n7_pv, systemSP);
	pv_n7->addAggregatedGenProfile(&pv_profile_n7, pv_units_per_node);

	// ##### add PV to N8 ####
	auto n8_DG0 = CPS::Node<Complex>::make("N8_DG0");
	auto pv_n8 = Ph1::AvVoltageSourceInverterDQ::make("pv_n8", Logger::Level::off);
	std::vector<Real> pv_profile_n8 = pvGenProfile;
	Real Pref_n8_pv = (pv_profile_n8.empty()) ? 20000 : pv_profile_n8.front() * pv_units_per_node;
	Real Qref_n8_pv = sqrt(std::pow(Pref_n8_pv / power_factor_pv, 2) - std::pow(Pref_n8_pv, 2));
	addDGtoSystem(n8_DG0, pv_n8, "N8", "pv", Pref_n8_pv, Qref_n8_pv, systemSP);
	pv_n8->addAggregatedGenProfile(&pv_profile_n8, pv_units_per_node);


	// ##### add PV to n9 ####
	auto n9_DG0 = CPS::Node<Complex>::make("N9_DG0");
	auto pv_n9 = Ph1::AvVoltageSourceInverterDQ::make("pv_n9", Logger::Level::off);
	std::vector<Real> pv_profile_n9 = pvGenProfile;
	Real Pref_n9_pv = (pv_profile_n9.empty()) ? 20000 : pv_profile_n9.front() * pv_units_per_node;
	Real Qref_n9_pv = sqrt(std::pow(Pref_n9_pv / power_factor_pv, 2) - std::pow(Pref_n9_pv, 2));
	addDGtoSystem(n9_DG0, pv_n9, "N9", "pv", Pref_n9_pv, Qref_n9_pv, systemSP);
	pv_n9->addAggregatedGenProfile(&pv_profile_n9, pv_units_per_node);


	// ##### add PV to n10 ####
	auto n10_DG0 = CPS::Node<Complex>::make("N10_DG0");
	auto pv_n10 = Ph1::AvVoltageSourceInverterDQ::make("pv_n10", Logger::Level::off);
	std::vector<Real> pv_profile_n10 = pvGenProfile;
	Real Pref_n10_pv = (pv_profile_n10.empty()) ? 20000 : pv_profile_n10.front() * pv_units_per_node;
	Real Qref_n10_pv = sqrt(std::pow(Pref_n10_pv / power_factor_pv, 2) - std::pow(Pref_n10_pv, 2));
	addDGtoSystem(n10_DG0, pv_n10, "N10", "pv", Pref_n10_pv, Qref_n10_pv, systemSP);
	pv_n10->addAggregatedGenProfile(&pv_profile_n10, pv_units_per_node);


	// ##### add PV to n11 ####
	auto n11_DG0 = CPS::Node<Complex>::make("N11_DG0");
	auto pv_n11 = Ph1::AvVoltageSourceInverterDQ::make("pv_n11", Logger::Level::off);
	std::vector<Real> pv_profile_n11 = pvGenProfile;
	Real Pref_n11_pv = (pv_profile_n11.empty()) ? 20000 : pv_profile_n11.front() * pv_units_per_node;
	Real Qref_n11_pv = sqrt(std::pow(Pref_n11_pv / power_factor_pv, 2) - std::pow(Pref_n11_pv, 2));
	addDGtoSystem(n11_DG0, pv_n11, "N11", "pv", Pref_n11_pv, Qref_n11_pv, systemSP);
	pv_n11->addAggregatedGenProfile(&pv_profile_n11, pv_units_per_node);

    RealTimeSimulation sim(simName, systemSP,
		args.timeStep, args.duration,
		args.solver.domain, args.solver.type, args.logLevel, false);
	Interface intf("/dpsim1-villas", "/villas-dpsim1",nullptr,false);
	Interface intf2("/dpsim2-villas", "/villas-dpsim2",nullptr,false);

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : systemSP.mNodes) {
		UInt i;
		if(n->name().find("DG")!= std::string::npos)
			continue;
		if (sscanf(n->name().c_str(), "N%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		auto n_stat = std::dynamic_pointer_cast<CPS::SP::Node>(n);
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
	// debug
	// intf 2
	o = 0;
	intf2.exportReal(pv_n3->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn3" << std::endl;o++;
	intf2.exportReal(pv_n4->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn4" << std::endl;o++;
	intf2.exportReal(pv_n5->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn5" << std::endl;o++;
	intf2.exportReal(pv_n6->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn6" << std::endl;o++;
	intf2.exportReal(pv_n7->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn7" << std::endl;o++;
	intf2.exportReal(pv_n8->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn8" << std::endl;o++;
	intf2.exportReal(pv_n9->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn9" << std::endl;o++;
	intf2.exportReal(pv_n10->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn10" << std::endl;o++;
	intf2.exportReal(pv_n11->attribute<Real>("Q_ref"), UInt(o));
	std::cout << "Signal " << o << ": " << "DEBUG_pvn11" << std::endl;o++;


    // Register power control imports
    pv_n3->ctrlReceiver(intf2.importReal(0));
    pv_n4->ctrlReceiver(intf2.importReal(1));
    pv_n5->ctrlReceiver(intf2.importReal(2));
    pv_n6->ctrlReceiver(intf2.importReal(3));
    pv_n7->ctrlReceiver(intf2.importReal(4));
    pv_n8->ctrlReceiver(intf2.importReal(5));
    pv_n9->ctrlReceiver(intf2.importReal(6));
    pv_n10->ctrlReceiver(intf2.importReal(7));
    pv_n11->ctrlReceiver(intf2.importReal(8));

	sim.addInterface(&intf, false);
	sim.addInterface(&intf2, false);
	sim.run(std::chrono::seconds(5));

	return 0;
}