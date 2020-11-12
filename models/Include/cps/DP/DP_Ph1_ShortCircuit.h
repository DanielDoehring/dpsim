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
//#include <cps/Solver/MNAVarElemInterface.h>
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/DP/DP_Ph1_Switch.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Models a short circuit between a node and ground
	class ShortCircuit :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<ShortCircuit> {
	protected:
		/// Nominal voltage [V]
		Real mNomVoltage;
		/// Actual voltage [V]
		Complex mVoltage;
		/// Actual voltage [V]
		Complex mCurrent;
		/// Resistance [Ohm]
		Real mResistance;
		/// Conductance [S]
		Real mConductance;
		/// Reactance [Ohm]
		Real mReactance;
		/// Internal resistance
		std::shared_ptr<DP::Ph1::Resistor> mSubResistor;

		/// new for protectopn
		std::shared_ptr<DP::Ph1::Switch> mSubProtectionSwitch;
		// has state changed? (allow this only once)
		// could be redundant with internal status of switch element (mnaIsClosed)
		Real mSwitchStateChange = false;
		//Bool mSwitchStateChanged = false;
		Real mSwitchROpen = 1e9;
		Real mSwitchRClosed = 1e-9;

		// load shedding
		Real mStartTime = 0;
		Real mEndTime = 0;
		Bool mEventSet = false;
		Bool mEventActive = false;


	public:
		/// Defines UID, name and logging level
		ShortCircuit(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		ShortCircuit(String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		ShortCircuit(String name,
			Real activePower, Real reactivePower, Real volt,
			Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		/// Sets model specific parameters
		void setParameters(Real volt);

		/// Delay time for action by switch
		void setNomVoltage(Real Voltage) { mNomVoltage = Voltage; };

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);

		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateVoltage(const Matrix& leftVector);

		// #### Internal Switch ####
		/// new getter for internal switch
		std::shared_ptr<DP::Ph1::Switch>& getProtectionSwitch() { return  mSubProtectionSwitch; };

		/// new update switch state function
		//void updateSwitchState(const Matrix& leftVector, Real time);
		void updateSwitchState(Real time);
		//Bool ValueChanged() { return mSwitchStateChange; };
		// defines load shedding event
		void setEvent(Real start, Real end);

		Bool mnaStateChanged() { return mSwitchStateChange; };

		class MnaPreStep : public Task {
		public:
			MnaPreStep(ShortCircuit& sc) :
				Task(sc.mName + ".MnaPreStep"), mSC(sc) {
				if (sc.mSubResistor)
					mAttributeDependencies.push_back(sc.mSubResistor->attribute("right_vector"));
				if (sc.mSubProtectionSwitch)
					mAttributeDependencies.push_back(sc.mSubProtectionSwitch->attribute("right_vector"));
				mModifiedAttributes.push_back(sc.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			ShortCircuit& mSC;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(ShortCircuit& sc, Attribute<Matrix>::Ptr leftVector) :
				Task(sc.mName + ".MnaPostStep"), mSC(sc), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				if (sc.mSubResistor)
					mAttributeDependencies.push_back(sc.mSubResistor->attribute("i_intf"));
				if (sc.mSubProtectionSwitch)
					mAttributeDependencies.push_back(sc.mSubProtectionSwitch->attribute("i_intf"));
				mModifiedAttributes.push_back(sc.attribute("i_intf"));
				mModifiedAttributes.push_back(sc.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			ShortCircuit& mSC;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
