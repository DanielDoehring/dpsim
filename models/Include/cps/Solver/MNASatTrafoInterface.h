/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	/// \brief MNA interface to be used by switching devices.
	class MNASatTrafoInterface : public MNAInterface {
	public:
		typedef std::shared_ptr<MNASatTrafoInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA section ####
		/// Open switch
		//virtual void mnaOpen() { }
		/// Close switch
		//virtual void mnaClose() { }
		/// Check if switch is closed
		//virtual Bool mnaIsClosed() = 0;
		/// Close switch if true and open switch if false
		//virtual void mnaSetClosed(Bool value) { }
		/// Stamps system matrix considering the defined switch position
		//virtual void mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) { }
		
		virtual Bool inductanceChange() = 0;
	};
}