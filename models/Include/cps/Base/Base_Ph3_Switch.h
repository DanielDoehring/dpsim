/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph3 {
	/// Dynamic Phasor Three-Phase Switch
	class Switch {
	protected:
		/// Resistance if switch is open [ohm]
		Matrix mOpenResistance;
		/// Resistance if switch is closed [ohm]
		Matrix mClosedResistance;
		/// Defines if Switch is open or closed
		Bool mSwitchClosed;
		Bool mValueChanged = false;
	public:
		///
		void setParameters(Matrix openResistance, Matrix closedResistance, Bool closed = false) {
			mOpenResistance = openResistance;
			mClosedResistance = closedResistance;
			mSwitchClosed = closed;
		}
		void closeSwitch() { mSwitchClosed = true; }
		void openSwitch() { mSwitchClosed = false; }
		void setValueChange(Bool value) { mValueChanged = value; };
	};
}
}
}
