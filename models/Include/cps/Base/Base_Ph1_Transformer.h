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
		Bool mOLTCActive = false;
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
		// use emt values for calculation
		Bool mCalcSatDP = true;

		// parameters of flux(current)-curve
		Real mLambdaM = 0;
		Real mLambdaK = 0;
		Real mLA = 0;
		Real mIM = 0;
		Real mLm = 0;

		// use this if mLm is only a mathemtical parameter and not for the real element 
		Real mLmElement = 0;

		Real mVm = 0;
		Real mIMag = 0;

		Real mDeltaT = 0;
		Real mPrevStepTime = 0;

		Bool mSatConstantsSet = false;
		Bool mSatConstantsSetDP = false;

		// constants for calculation of magnetizing current from flux
		Real mSatConstA;
		Real mSatConstB;
		Real mSatConstC;
		Real mSatConstD;

		// EMT-calc specific parameters
		Real mInitialFlux = 0;
		Real mCurrentFlux = 0;
		Real mDeltaFlux = 0;
		Real mVmAngle = 0;
		Real mLMagCurrentReal = 0;

		// DP-calc specific parameters
		Complex mCurrentFluxDP = Complex(0, 0);
		Complex mHphi = Complex(0, 0);
		Complex mBphi = Complex(0, 0);
		Complex mAphi = Complex(0, 0);
		Complex mVmDP = Complex(0, 0);
		Complex mTD   = Complex(0.5, 0);

	public:
		///
		void setParameters(Real ratioAbs, Real ratioPhase, Real resistance, Real inductance) {
			mRatio = std::polar<Real>(ratioAbs, ratioPhase);
			mResistance = resistance;
			mInductance = inductance;

			// initial tap is 0
			mRatioInitial = mRatio;
		}

		void setOLTCParamters(Real NumTaps, Real VRefLV, Real DeltaVTapChange = 0) {
			mOLTCActive = true;
			/// input parameters
			mNumTaps = NumTaps;
			mRefLV = VRefLV;

			/// set/calculate other parameter
			mCurrTapPos = 0;

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
			// set paramteres of saturation curve
			mLambdaM = lambdaM;
			mLambdaK = lambdaK;
			mLA = LA;
			mLm = LM;
			mIM = IM;
		}

		void setParametersSaturationDefault(Real HV, Real LV) {
			// set Saturation Parameters for standard Transformer depending on low and high voltage level
			if (HV == 220000 && LV == 66000) {
				mLambdaM = 686.611426;
				mLambdaK = 1311.63923;
				mLA = 2.85312113;
				mLm = 266.412156;
				mIM = 0.10031921;
			}
			else if (HV == 20000 && LV == 660)
			{
				mLambdaM = 90.032;
				mLambdaK = 96.954;
				mLA = 2.51;
				mLm = 225.11;
				mIM = 0.165;
			}
			else if (HV == 115000 && LV == 21000)
			{
				mLambdaM = 517.682;
				mLambdaK = 498.93;
				mLA = 17.86;
				mLm = 221.71;
				mIM = 2.335;
			}
			else if (HV == 11000 && LV == 660)
			{
				mLambdaM = 49.517;
				mLambdaK = 59.42;
				mLA = 0.45;
				mLm = 278.19;
				mIM = 0.178;
			}
		}

		void setSaturationConstants() {
			// some static constants defined here for simpler calc of saturation
			mSatConstA = mLA / (mLambdaK * mLambdaK);

			mSatConstB = (mLA * mLm - mLambdaM) / mLambdaK;

			mSatConstC = mIM * (mLA * mIM - mLambdaM + mLambdaK);

			mSatConstD = (mSatConstB - sqrt( (mSatConstB*mSatConstB) - 4*mSatConstA*mSatConstC )) / (2 * mSatConstA);

			mSatConstantsSet = true;
		}

		void setSaturationCalculationMethod(Bool CalcDPDomain) {
			mCalcSatDP = CalcDPDomain;
		}

		void setOLTCActive(Bool active) {
			mOLTCActive = active;
		}

		void setMagnetizingInductance(Real Lm) {
			mLmElement = Lm;
		}
	};
}
}
}
