/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "cps/CIM/Reader.h"
#include <DPsim.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){
	// Find CIM files

	std::list<fs::path> filenames;

	/*
	filenames = DPsim::Utils::findFiles({
		"1_meine_geo_EQ.xml",
		"1_meine_geo_TP.xml",
		"1_meine_geo_SSH.xml",
		"1_meine_geo_SV.xml"
		//}, "hv-grid-zloads", "CIMPATH");
		}, "O:/Projekte/alle Projekte/2018_OVRTuere/Projektbearbeitung/AP 1/HS_Netze/hs_netz/1_3", "CIMPATH");
	*/

	/*
	filenames = DPsim::Utils::findFiles({
		"case14.xml"
		//}, "hv-grid-zloads", "CIMPATH");
	}, "C:/DPsim/dpsim-scenarios/dpsim-scenarios/build/_deps/cim-data-src/Matpower_cases", "CIMPATH");
	*/

	///*
	filenames = DPsim::Utils::findFiles({
		"Netz Einfach_meine_geo_EQ.xml",
		"Netz Einfach_meine_geo_TP.xml",
		"Netz Einfach_meine_geo_SSH.xml",
		"Netz Einfach_meine_geo_SV.xml"
		//}, "hv-grid-zloads", "CIMPATH");
		//}, "C:/DPsim/dpsim-scenarios/dpsim-scenarios/hv-grid-basic/Last", "CIMPATH");
		}, "C:/DPsim/dpsim-scenarios/dpsim-scenarios/hv-grid-basic/Alles/220 110 kV/Last", "CIMPATH");
	//*/


	String simName = "hv-grid-basic-Ytest";
	Logger::setLogDir("logs/" + simName);
	CPS::Real system_freq = 50;

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::debug);
	SystemTopology sys = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : sys.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}

	Simulation sim(simName, sys, 1, 1, Domain::SP, Solver::Type::NRP, Logger::Level::debug, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

