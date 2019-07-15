/*
 * @file
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

#pragma once

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <dpsim/DataLogger.h>

namespace DPsim {

	class CSVDataLogger : public DataLogger, public SharedFactory<CSVDataLogger> {
	protected:
		std::ofstream mLogFile;
		fs::path mLogFileName;

		void logDataLine(Real time, Real data);
		void logDataLine(Real time, const Matrix& data);
		void logDataLine(Real time, const MatrixComp& data);

	public:
		using Ptr = std::shared_ptr<CSVDataLogger>;

		CSVDataLogger(String name, Bool enabled = true, UInt downsampling = 1);

		void logPhasorNodeValues(Real time, const Matrix& data);
		void logEMTNodeValues(Real time, const Matrix& data);
		void setColumnNames(std::vector<String> names);

		virtual void close();
		virtual void open();

		virtual void log(Real time, Int timeStepCount);
	};
};
