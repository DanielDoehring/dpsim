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

#include <dpsim/PythonDataLogger.h>

#include <cps/AttributeList.h>

using namespace DPsim;

PythonDataLogger::PythonDataLogger(String name, Bool enabled, UInt downsampling, UInt chunkSize) :
	DataLogger(name, enabled, downsampling) {

	mChunkSize = chunkSize;

	open();
}

void PythonSubDataLogger::log(Real time, Int timeStepCount) {
	if (!mEnabled)
		return;

	if (!(timeStepCount % mDownsampling == 0))
		return;

	if (!mCurrentChunk)
		mCurrentChunk = std::make_shared<Chunk>(*this);

	mCurrentChunk->append(mAttributes);

	assert(mPos < mSize);

	for (auto it : mAttributes) {
		auto name = it.first;
		auto attr = std::dynamic_pointer_cast<CPS::Attribute<Real>>(it.second);

		assert(attr);

		auto *nd = mCurrentChunk->data[attr];

		Real *dst = (Real *) PyArray_GETPTR2(nd, mPos, 0);

		*dst = attr->get();
	}

	mPos++;

	if (mPos >= mSize) {
		std::lock_guard<std::mutex> lock(mMutex);

		mQueue.push(std::move(mCurrentChunk));
	}
}

void PythonSubDataLogger::drain() {
	std::lock_guard<std::mutex> lock(mMutex);

	while (!mChunks.empty()) {
		Chunk c = mChunks.front();

		for (auto it : c.mData) {
			auto attr = it.first;
			auto pao = it.second;

			void *src =
		}

		mChunks.pop()
	}
}
