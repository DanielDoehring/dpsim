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
	class Transformer {
	protected:
		/// Transformer ratio
		Complex mRatio;
		/// Resistance [Ohm]
		Real mResistance;
		/// Inductance [H]
		Real mInductance;

		/// NEW for OLTC functionality
		// was ratio updated
		Bool mRatioChange = false;

		/// #### OLTC parameters ####
		// deadband where no regulation takes place in p. u. (0.05 -> from 95 % to 105 % nothing happens)
		Real mDeadband = 0.05;
		// Number of taps
		Real mNumTaps;
		// Current TapPos
		Real mCurrTapPos;
		// Current ratio
		Complex mRatioInitial;
		// max Voltage (abs)
		Real mMaxVoltage = 0.1;
		// Time Delay between action [s]
		Real mTapChangeTimeDelay = 30;
		// Voltage Change in one Step in p. u.
		Real mDeltaVTapChange;
		// reference voltage
		Real mRefLV;
		// Counter for durance of voltage violation
		Real mViolationCounter = 0;

		Real mVLV;
		/// New attributes for saturation effects
		// parameters of flux(current)-curve
		Real mLambdaM = 0;
		Real mLambdaK = 0;
		Real mLA = 0;
		Real mIM = 0;
		Real mLm = 0;

		Real mInitialFlux = 0;
		Real mCurrentFlux;

		Real mVm = 0;

		Real mPrevStepTime = 0;
		Bool mSatConstantsSet = false;
		Real mSatConstA;
		Real mSatConstB;
		Real mSatConstC;
		Real mSatConstD;
		

	public:
		///
		void setParameters(Real ratioAbs, Real ratioPhase, Real resistance, Real inductance) {
			mRatio = std::polar<Real>(ratioAbs, ratioPhase);
			mResistance = resistance;
			mInductance = inductance;
		}

		void setOLTCParamteres(Real NumTaps, Real VRefLV, Real DeltaVTapChange = 0) {
			/// input parameters
			mNumTaps = NumTaps;
			mRefLV = VRefLV;

			/// set/calculate other parameter
			// initial tap is 0
			mCurrTapPos = 0;
			mRatioInitial = mRatio;

			// voltage diff pf one tap change in p. u.
			if (DeltaVTapChange)
				mDeltaVTapChange = DeltaVTapChange;
			else
			{
				mDeltaVTapChange = (mMaxVoltage - mDeadband) / NumTaps;
			}
		}

		void setOLTCDeadband(Real deadband) { mDeadband = deadband; };
		void setOLTCTimeDelay(Real delay) { mTapChangeTimeDelay = delay; };


		void setParametersSaturation(Real lambdaM, Real lambdaK, Real LA, Real LM, Real IM) {
			mLambdaM = lambdaM;
			mLambdaK = lambdaK;
			mLA = LA;
			mLm = LM;
			mIM = IM;
		}

		void setSaturationConstants() {
			// some constants for better calc of saturation
			mSatConstA = mLA / (mLambdaK * mLambdaK);

			mSatConstB = (mLA * mIM - mLambdaM) / mLambdaK;

			mSatConstC = mIM * (mLA * mIM - mLambdaM + mLambdaK);

			mSatConstD = (-mSatConstB - sqrt( (mSatConstB*mSatConstB) - 4*mSatConstA*mSatConstC )) / (2 * mSatConstA);

			mSatConstantsSet = true;
		}
	};
}
}
}
