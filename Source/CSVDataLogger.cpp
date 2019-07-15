/**
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

#include <iomanip>

#include <dpsim/CSVDataLogger.h>
#include <cps/Logger.h>

using namespace DPsim;

CSVDataLogger::CSVDataLogger(String name, Bool enabled, UInt downsampling) :
	DataLogger(name, enabled, downsampling) {

	mLogFileName = CPS::Logger::logDir() + "/" + name + ".csv";

	if (mLogFileName.has_parent_path() && !fs::exists(mLogFileName.parent_path()))
		fs::create_directory(mLogFileName.parent_path());

	mLogFile.setstate(std::ios_base::badbit);

	open();
}

void CSVDataLogger::open() {
	if (!mEnabled)
		return;

	mLogFile = std::ofstream(mLogFileName, std::ios_base::out|std::ios_base::trunc);
	if (!mLogFile.is_open()) {
		throw std::runtime_error(fmt::format("Cannot open log file {}", mLogFileName.native()));
	}
}

void CSVDataLogger::close() {
	if (mLogFile.is_open())
		mLogFile.close();
}

void CSVDataLogger::log(Real time, Int timeStepCount) {
	if (!mEnabled || !(timeStepCount % mDownsampling == 0))
		return;

	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto it : mAttributes)
			mLogFile << ", " << std::right << std::setw(13) << it.first;
		mLogFile << '\n';
	}

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (auto it : mAttributes)
		mLogFile << ", " << std::right << std::setw(13) << it.second->toString();
	mLogFile << '\n';
}

void CSVDataLogger::setColumnNames(std::vector<String> names) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto name : names) {
			mLogFile << ", " << std::right << std::setw(13) << name;
		}
		mLogFile << '\n';
	}
}

void CSVDataLogger::logDataLine(Real time, Real data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	mLogFile << ", " << std::right << std::setw(13) << data;
	mLogFile << '\n';
}

void CSVDataLogger::logDataLine(Real time, const Matrix& data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void CSVDataLogger::logDataLine(Real time, const MatrixComp& data) {
	if (!mEnabled)
		return;
	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void CSVDataLogger::logPhasorNodeValues(Real time, const Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;
		for (Int i = 0; i < data.rows(); i++) {
			std::stringstream name;
			if (i < data.rows() / 2)
				name << "node" << std::setfill('0') << std::setw(5) << i << ".re";
			else
				name << "node" << std::setfill('0') << std::setw(5) << (i - data.rows() / 2) << ".im";
			names.push_back(name.str());
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}

void CSVDataLogger::logEMTNodeValues(Real time, const Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;
		for (Int i = 0; i < data.rows(); i++) {
			std::stringstream name;
			name << "node" << std::setfill('0') << std::setw(5) << i;
			names.push_back(name.str());
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}
