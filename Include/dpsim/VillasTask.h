/** Abstract base class for schedulers using std::thread and the self-written
 * Barrier and Counter synchronization primitives.
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <thread>
#include <experimental/filesystem>

#include <dpsim/Definitions.h>

namespace fs = std::experimental::filesystem;

namespace DPsim {
	class VillasTask {
	public:
		VillasTask(fs::path config);
		~VillasTask();

	protected:

	private:
		std::thread mThread;

		fs::path config_path;
		char config_path_c[200];

		Bool mJoining = false;
	};
}
