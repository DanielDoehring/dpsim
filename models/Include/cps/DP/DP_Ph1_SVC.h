/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
//#include <cps/Solver/MNATearInterface.h>
#include <cps/Solver/MNAVarElemInterface.h>
#include <cps/Base/Base_Ph1_SVC.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief SVC
	///
	/// SVC represented by an Inductor
	/// The inductor is represented by a DC equivalent circuit which corresponds to
	/// one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in parallel with a current source.
	/// The resistance is constant for a defined time step and system
	/// frequency and the current source changes for each iteration.
	class SVC :
		public Base::Ph1::SVC,
		public MNAVarElemInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<SVC> {
	protected:
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;
		///
		void initVars(Real timeStep);
	public:
		/// Defines UID, name and log level
		SVC(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		SVC(String name, Logger::Level logLevel = Logger::Level::off)
			: SVC(name, name, logLevel) { }

		// #### General ####
		/// Return new instance with the same parameters
		SimPowerComp<Complex>::Ptr clone(String name);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// Initializes states from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Update interface current from MNA system results
		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateCurrentHarm();

		// #### Tearing methods ####
		void mnaTearInitialize(Real omega, Real timestep);
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Complex voltage, Complex current);

		Bool ValueChanged() { return mInductanceChange; };

		void updateSusceptance();
		void updateVars();

		class MnaPreStep : public Task {
		public:
			MnaPreStep(SVC& svc) :
				Task(svc.mName + ".MnaPreStep"), mSVC(svc) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(svc.attribute("right_vector"));
				mPrevStepDependencies.push_back(svc.attribute("v_intf"));
				mPrevStepDependencies.push_back(svc.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SVC& mSVC;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SVC& svc, Attribute<Matrix>::Ptr leftVector) :
				Task(svc.mName + ".MnaPostStep"),
				mSVC(svc), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mSVC.attribute("v_intf"));
				mModifiedAttributes.push_back(mSVC.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SVC& mSVC;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPreStepHarm : public Task {
		public:
			MnaPreStepHarm(SVC& svc)
				: Task(svc.mName + ".MnaPreStepHarm"),
				mSVC(svc) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(svc.attribute("right_vector"));
				mPrevStepDependencies.push_back(svc.attribute("v_intf"));
				mPrevStepDependencies.push_back(svc.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SVC& mSVC;
		};

		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(SVC& svc, std::vector<Attribute<Matrix>::Ptr> leftVectors)
				: Task(svc.mName + ".MnaPostStepHarm"),
				mSVC(svc), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mSVC.attribute("v_intf"));
				mModifiedAttributes.push_back(mSVC.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SVC& mSVC;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};


	};
}
}
}
