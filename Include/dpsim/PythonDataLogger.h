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

#include <queue>
#include <mutex>
#include <map>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#define NO_IMPORT_ARRAY
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <dpsim/DataLogger.h>

namespace DPsim {

	class PythonSubDataLoggerBase {

	public:
		using Ptr = std::shared_ptr<PythonSubDataLoggerBase>

		PythonSubDataLoggerBase(CPS::AttributeBase::Ptr attr);

		void drain();

		void log(Real time, Int timeStepCount);

		PyObject * data();
	};

	template<typename T>
	class PythonSubDataLogger : public PythonSubDataLoggerBase {

	public:

		template<typename T>
		class Chunk {
		public:

			Chunk() {

			}

			~Chunk() {

			}

		protected:

		};



	protected:

		Chunk::Ptr mCurrentChunk;
		Chunk::Queue mChunks;

		UInt mChunkSize;

		/// Mutex to protect mChunks queue
		std::mutex mMutex;

		PyArrayObject *mData;
	};

	class PythonDataLogger : public DataLogger, public SharedFactory<PythonDataLogger> {
	protected:

		std::map<CPS::AttributeBase::Ptr, PythonSubDataLoggerBase::Ptr> mLoggers;

	public:
		using Ptr = std::shared_ptr<PythonDataLogger>;

		PythonDataLogger(String name, Bool enabled = true, UInt downsampling = 1, UInt chunkSize = 100);

		virtual void log(Real time, Int timeStepCount) {
			for (auto it : mAttributes) {
				mLoggers[it.second]->log(time, timeStepCount);
			}
		}

		PyObject * data(CPS::AttributeBase::Ptr attr) {
			return mLoggers[attr]->data();
		}
	};
};
