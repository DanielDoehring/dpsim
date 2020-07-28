/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/AttributeList.h>
#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Task.h>

namespace CPS {
	/// Interface to be implemented by all models used by the MNA solver.
	class MNAInterface : virtual public AttributeList {
	public:
		typedef std::shared_ptr<MNAInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA Base Functions ####
		/// Initializes variables of components
		virtual void mnaInitialize(Real omega, Real timeStep) {
			mMnaTasks.clear();
		}
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
			mnaInitialize(omega, timeStep);
		}
		/// Stamps system matrix
		virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) { }
		/// Stamps right side (source) vector
		virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) { }
		/// Update interface voltage from MNA system result
		virtual void mnaUpdateVoltage(const Matrix& leftVector) { }
		/// Update interface current from MNA system result
		virtual void mnaUpdateCurrent(const Matrix& leftVector) { }

		// #### MNA Harmonic Base Functions ####
		/// Initializes variables of components
		virtual void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) {
			mnaInitialize(omega, timeStep);
		}
		/// Stamps system matrix considering the frequency index
		virtual void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) { }
		/// Stamps right side (source) vector considering the frequency index
		virtual void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector) { }
		virtual void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) { }
		/// Return list of MNA tasks
		const Task::List& mnaTasks() {
			return mMnaTasks;
		}

	protected:
		/// Every MNA component modifies its source vector attribute.
		MNAInterface() {
			addAttribute<Matrix>("right_vector", &mRightVector, Flags::read);
		}

		/// List of tasks that relate to using MNA for this component (usually pre-step and/or post-step)
		Task::List mMnaTasks;
		/// This component's contribution ("stamp") to the right-side vector.
		/// TODO performance improvements from a sparse representation, at least during copying / summation?
		Matrix mRightVector;
	};
}
