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
namespace Ph1 {
	class SVC {
	protected:
		/// Inductance [H]
		Real mInductance;
		/// Maximium susceptance
		Real mBMax;
		/// Minimium susceptance
		Real mBMin;
		/// Time Constant
		Real mTr;
		/// Gain
		Real mKr;
		/// Reference Voltage
		Real mRefVolt;
		/// Nominal Voltage
		Real mNomVolt;
		// Time step values
		Real mPrevTimeStep = 0;
		Real mDeltaT;

		Bool mInductanceChange = false;

		// save for numerical integration
		Real mPrevVoltage;
		Real mDeltaV = 0;
		Real mBPrev;

	public:
		/// Sets model specific parameters
		void setParameters(Real Bmax, Real Bmin, Real nomVolt, Real RefVolt = 0) {
			// initial inductance very high 10^6 [Ohm] @ 50 Hz
			mInductance = 3183.1;
			mBMax = Bmax;
			mBMin = Bmin;
			mNomVolt = nomVolt;
			mRefVolt = (RefVolt > 0) ? RefVolt : mNomVolt;
		}

		void setControllerParameters(Real T, Real K) {
			// Pt1 controller
			mTr = T;
			mKr = K;
		};
	};
}
}
}
