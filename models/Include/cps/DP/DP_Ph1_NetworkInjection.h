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
#include <cps/Solver/DAEInterface.h>
#include <cps/PowerProfile.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Ideal Voltage source model
	///
	/// This model uses modified nodal analysis to represent an ideal voltage source.
	/// For a voltage source between nodes j and k, a new variable
	/// (current across the voltage source) is added to the left side vector
	/// as unkown and it is taken into account for the equation of node j as
	/// positve and for the equation of node k as negative. Moreover
	/// a new equation ej - ek = V is added to the problem.
	class NetworkInjection :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public DAEInterface,
		public SharedFactory<NetworkInjection> {
	private:
		///
		void updateVoltage(Real time, Int timeStepCount);
		///
		Attribute<Complex>::Ptr mVoltageRef;
		///
		Attribute<Real>::Ptr mSrcFreq;
	public:
		/// Load profile data
		VoltageProfile mSourceProfile;
		/// Use the assigned load profile
		bool use_profile = false;
		/// Defines UID, name, component parameters and logging level
		NetworkInjection(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines UID, name, component parameters and logging level
		NetworkInjection(String name, Logger::Level logLevel = Logger::Level::off)
			: NetworkInjection(name, name, logLevel) { }
		/// Defines name, component parameters and logging level
		NetworkInjection(String name,
			Complex voltage, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		///
		void setSourceValue(Complex voltage);
		///
		void initialize(Matrix frequencies);
		///
		void setParameters(Complex voltageRef, Real srcFreq = -1);

		// #### MNA Section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(NetworkInjection& NetworkInjection) :
				Task(NetworkInjection.mName + ".MnaPreStep"), mNetworkInjection(NetworkInjection) {
				mAttributeDependencies.push_back(NetworkInjection.attribute("V_ref"));
				mModifiedAttributes.push_back(mNetworkInjection.attribute("right_vector"));
				mModifiedAttributes.push_back(mNetworkInjection.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			NetworkInjection& mNetworkInjection;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(NetworkInjection& NetworkInjection, Attribute<Matrix>::Ptr leftVector) :
				Task(NetworkInjection.mName + ".MnaPostStep"),
				mNetworkInjection(NetworkInjection), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mNetworkInjection.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			NetworkInjection& mNetworkInjection;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPreStepHarm : public CPS::Task {
		public:
			MnaPreStepHarm(NetworkInjection& NetworkInjection) :
				Task(NetworkInjection.mName + ".MnaPreStepHarm"),
				mNetworkInjection(NetworkInjection) {
				mAttributeDependencies.push_back(NetworkInjection.attribute("V_ref"));
				mModifiedAttributes.push_back(mNetworkInjection.attribute("right_vector"));
				mModifiedAttributes.push_back(mNetworkInjection.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			NetworkInjection& mNetworkInjection;
		};

		class MnaPostStepHarm : public CPS::Task {
		public:
			MnaPostStepHarm(NetworkInjection& NetworkInjection, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
				Task(NetworkInjection.mName + ".MnaPostStepHarm"),
				mNetworkInjection(NetworkInjection), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mNetworkInjection.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			NetworkInjection& mNetworkInjection;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};

		// #### DAE Section ####
		/// Residual function for DAE Solver
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off);
		///Voltage Getter
		Complex daeInitialize();
	};
}
}
}
