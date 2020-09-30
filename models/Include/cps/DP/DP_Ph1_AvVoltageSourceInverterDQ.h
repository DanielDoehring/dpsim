/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once


#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Definitions.h>
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/DP/DP_Ph1_Capacitor.h>
#include <cps/DP/DP_Ph1_ControlledVoltageSource.h>
#include <cps/DP/DP_Ph1_Transformer.h>
#include <cps/DP/DP_Ph1_Switch.h>
#include <cps/Base/Base_AvVoltageSourceInverterDQ.h>
#include <cps/PowerProfile.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	class AvVoltageSourceInverterDQ :
		public Base::AvVoltageSourceInverterDQ,
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<AvVoltageSourceInverterDQ> {
	protected:

		// ### Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<DP::Ph1::ControlledVoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<DP::Ph1::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<DP::Ph1::Transformer> mConnectionTransformer;

		// ### inputs ###
		///
		Matrix mVcdq = Matrix::Zero(2, 1);
		///
		Matrix mIrcdq = Matrix::Zero(2, 1);

		// ### outputs ###
		///
		Matrix mVsdq = Matrix::Zero(2, 1);

		/// instantaneous omega
		Real mOmegaInst=0;
		/// instantaneous frequency
		Real mFreqInst=0;
		///
		Bool mCtrlOn = true;
		///
		Bool mCoveeCtrled=true;
		///
		Bool mIsLoad=false;
		///
		Bool mWithConnectionTransformer=false;
		/// in case variable time step simulation should be developed in the future
		Real mTimeStep;

		///
		std::vector<Real>* mGenProfile = nullptr;
		///
		std::vector<Real>::iterator mCurrentPower;
		///
		std::vector<PQData>::iterator mCurrentLoad;
		///
		UInt mProfileUndateRate = 1000;
		///
		Attribute<Real>::Ptr mQRefInput;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

		/// New for protection
		std::shared_ptr<DP::Ph1::Switch> mSubProtectionSwitch;
		Bool mSwitchActive;
		// has state changed? (allow this only once)
		// could be redundant with internal status of switch element (mnaIsClosed)
		Real mSwitchStateChange = false;
		Real mSwitchROpen   = 1e9;
		Real mSwitchRClosed = 1e-9;

		/// ### new for QU Control ###
		// update after some set value of seconds
		Real mUpdateCounter = 0;
		Real mUpdateCounterValue = 0.05;
		// wait a time period after votlage has recoverd to reset state
		Real mRecoveryCounter = 0;
		Real mRecoveryValue = 0.1;
		// save time step between to simulation steps
		Real mDeltaT;
		// is VSI in fault state? (voltage dependend)
		Bool mFaultState = false;
		Real mFaultStartTime = 0;
		// count time for which fault is active
		Real mFaultCounter = 0;
		// time value of previous simulation step
		Real mPrevTime = 0;
		// save static values of P and Q for control
		Real mQRefStatic = 0;
		Real mPRefStatic = 0;
		// was P reduced during fault?
		Bool mPReduced = false;
		Real mPNewPrev;

		//
		Real mVpcc = 0;
		Real mDeltaVNom = 0;
		Real mDeltaV = 0;
		Real mDeltaVPrev = 0;
		Real mVppPrev = 0;
		Real mVmeasPrev = 0;
		Real mDeltaIPrev = 0;
		Real mDeltaIpu = 0;
		Real mQRefStaticpu = 0;
		Bool mResetQ = false;

		Bool mDisconnected = false;
		Real mFaultStartDelay = 0;

	public:
		///
		std::vector<PQData> mLoadProfile;
		// #### constructors ####
		///
		AvVoltageSourceInverterDQ(String name,
			Logger::Level logLevel = Logger::Level::off) :AvVoltageSourceInverterDQ(name, name, logLevel) {}
		///
		AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false, Bool switchActive = false);
		///
		SimPowerComp<Complex>::Ptr clone(String copySuffix);

		///
		void initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		///
		void updateInputStateSpaceModel(const Matrix& leftVector, Real time);
		///
		void updateStates();
		///
		void updateBMatrixStateSpaceModel();

		/// convert between two rotating frames
		Complex rotatingFrame2to1(Complex f, Real theta1, Real theta2);
		///
		void step(Real time, Int timeStepCount);
		///
		void updatePowerGeneration();
		// #### General ####
		///
		void addGenProfile(std::vector<Real>* genProfile);
		///
		void addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber);
		///
		void updateSetPoint(Real time);
		///
		void initialize(Matrix frequencies);
		///
		void initializeFromPowerflow(Real frequency);
		// #### interface with villas node ####
		void ctrlReceiver(Attribute<Real>::Ptr qref);
		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// interface with power control service
		void coveeCtrled(Bool ctrled){mCoveeCtrled=ctrled;};
		///
		void makeLoad(Bool isLoad){mIsLoad=isLoad;};
		///
		Bool isLoad(){return mIsLoad;};
		///
		void setProfileUpdateRate(UInt rate){mProfileUndateRate=rate;};


		/// ### new for protection ###

		/// new getter for internal switch
		std::shared_ptr<DP::Ph1::Switch>& getProtectionSwitch() { return mSubProtectionSwitch; };

		/// new update switch state function
		//void updateSwitchState(const Matrix& leftVector, Real time);
		void updateSwitchState(Real time);
		Bool checkFRTGuidelineValue(Real time, Real voltageLevel);

		/// ### new for Q-Control ###
		Real PT1ControlStep(Real u, Real u_prev, Real y_prev, Real K, Real T, Real deltaT);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("P_ref"));
				mPrevStepDependencies.push_back(AvVoltageSourceInverterDQ.attribute("i_intf"));
				mPrevStepDependencies.push_back(AvVoltageSourceInverterDQ.attribute("v_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("v_intf"));
				if (AvVoltageSourceInverterDQ.mSwitchActive)
					mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubProtectionSwitch->attribute("right_vector"));

			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class AddBStep : public Task {
		public:
			AddBStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".AddBStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubInductorF->attribute("right_vector"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCapacitorF->attribute("right_vector"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("right_vector"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("right_vector"));
				if (mAvVoltageSourceInverterDQ.mWithConnectionTransformer) {
					mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mConnectionTransformer->attribute("right_vector"));
				}
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class CtrlStep : public Task {
		public:
			CtrlStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".CtrlStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mQRefInput);
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};



		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ, Attribute<Matrix>::Ptr leftVector) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPostStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ), mLeftVector(leftVector)
			{
				mAttributeDependencies.push_back(leftVector);
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mQRefInput);
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorF->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubInductorF->attribute("i_intf"));
				if (mAvVoltageSourceInverterDQ.mWithConnectionTransformer) {
					mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mConnectionTransformer->attribute("i_intf"));
				}
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorC->attribute("i_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("i_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("v_intf"));
				if (AvVoltageSourceInverterDQ.mSwitchActive)
					mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubProtectionSwitch->attribute("i_intf"));

			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
