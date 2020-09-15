/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_SVC.h>

using namespace CPS;

DP::Ph1::SVC::SVC(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	mSLog->info("Create {} {}", this->type(), name);
	setTerminalNumber(1);
	setVirtualNodeNumber(2);

	addAttribute<Real>("B", &mBPrev, Flags::read | Flags::write);
	addAttribute<Real>("DeltaV", &mDeltaV, Flags::read | Flags::write);
	addAttribute<Real>("Vpcc", &mVpcc, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::SVC::clone(String name) {
	auto copy = SVC::make(name, mLogLevel);
	copy->setParameters(mBMax, mBMin, mBN, mNomVolt, mRefVolt);
	return copy;
}

void DP::Ph1::SVC::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);
}

void DP::Ph1::SVC::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// initial state is both switches are open
	Real omega = 2. * PI * frequency;
	// init L and C with small/high values (both have high impedance)
	Real LInit = 1e6 / omega;
	Real CInit = 1e-6 / omega;
	mLPrev = LInit;
	mCPrev = CInit;

	// impedances of both branches
	Complex LImpedance = { mSwitchROpen, omega * LInit };
	Complex CImpedance = { mSwitchROpen, -1/(omega * CInit) };
	Complex impedance = LImpedance * CImpedance / (LImpedance + CImpedance);

	mIntfVoltage(0, 0) = initialSingleVoltage(0);
	mIntfCurrent(0,0)  = mIntfVoltage(0,0) / impedance;

	mBPrev = 0;
	mPrevVoltage = mIntfVoltage(0, 0).real();
	mVmeasPrev = mPrevVoltage;

	mSLog->info(
		"\n --- Parameters ---"
		"\n Controller: T = {} K = {}"
		"\n Reference Voltage  {} [kV]"
		"\n Bmax = {} Bmin = {} [p.u.]"
		"\n Initial B: {}",
		mTr, mKr, mRefVolt, mBMax, mBMin, mBPrev);

	// set voltages at virtual nodes
	Complex VLSwitch = mIntfVoltage(0, 0) - LImpedance * mIntfCurrent(0, 0);
	mVirtualNodes[0]->setInitialVoltage(VLSwitch);
	Complex VCSwitch = mIntfVoltage(0, 0) - CImpedance * mIntfCurrent(0, 0);
	mVirtualNodes[1]->setInitialVoltage(VCSwitch);

	// create elements
	// Inductor with Switch
	mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(LInit);
	mSubInductor->connect({ SimNode::GND, mVirtualNodes[0] });
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromPowerflow(frequency);

	mSubInductorProtectionSwitch = std::make_shared<DP::Ph1::Switch>(mName + "_Lswitch", mLogLevel);
	mSubInductorProtectionSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
	mSubInductorProtectionSwitch->connect({ mVirtualNodes[0], mTerminals[0]->node() });
	mSubInductorProtectionSwitch->initialize(mFrequencies);
	mSubInductorProtectionSwitch->initializeFromPowerflow(frequency);

	// Capacitor with Switch
	mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap", mLogLevel);
	mSubCapacitor->setParameters(CInit);
	mSubCapacitor->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCapacitor->initialize(mFrequencies);
	mSubCapacitor->initializeFromPowerflow(frequency);

	mSubCapacitorProtectionSwitch = std::make_shared<DP::Ph1::Switch>(mName + "_Cswitch", mLogLevel);
	mSubCapacitorProtectionSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
	mSubCapacitorProtectionSwitch->connect({ mVirtualNodes[1], mTerminals[0]->node() });
	mSubCapacitorProtectionSwitch->initialize(mFrequencies);
	mSubCapacitorProtectionSwitch->initializeFromPowerflow(frequency);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nImpedance: {}"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		impedance,
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}

// #### MNA functions ####

void DP::Ph1::SVC::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	auto subComponents = MNAInterface::List({ mSubInductor , mSubCapacitor, mSubInductorProtectionSwitch, mSubCapacitorProtectionSwitch });

	for (auto comp : subComponents) {
		comp->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : comp->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex());
}

void DP::Ph1::SVC::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorProtectionSwitch->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorProtectionSwitch->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::SVC::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
	mSubCapacitorProtectionSwitch->mnaApplyRightSideVectorStamp(rightVector);
	mSubInductorProtectionSwitch->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::SVC::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSVC.mnaApplyRightSideVectorStamp(mSVC.mRightVector);
	if (time > 0.1)
		mSVC.updateSusceptance();
}

void DP::Ph1::SVC::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSVC.mnaUpdateVoltage(*mLeftVector);
	mSVC.mnaUpdateCurrent(*mLeftVector);
	mSVC.mDeltaT = time - mSVC.mPrevTimeStep;
	mSVC.mPrevTimeStep = time;
	mSVC.mValueChange = false;
}

void DP::Ph1::SVC::mnaUpdateVoltage(const Matrix& leftVector) {
	mVpcc = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, 0).real();
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::SVC::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = 0;
	mIntfCurrent(0, 0) += mSubInductor->intfCurrent()(0, 0);
	mIntfCurrent(0, 0) += mSubCapacitor->intfCurrent()(0, 0);
}

void DP::Ph1::SVC::updateSusceptance() {
	// calculate new B value
	// summarize some constants
	Real Fac1 = mDeltaT / (2 * mTr);
	Real Fac2 = mDeltaT * mKr / (2 * mTr);

	Complex vintf = mIntfVoltage(0, 0);
	Real V = Math::abs(mIntfVoltage(0, 0).real());

	// Pt1 with trapez rule for voltage measurement
	Real Fac3 = mDeltaT / (2 * mTm);
	Real Vmeas = (1 / (1 + Fac3)) * (V + mPrevVoltage - mVmeasPrev);

	mDeltaV = (Vmeas - mRefVolt) / mNomVolt;
	Real deltaVPrev = (mVmeasPrev - mRefVolt) / mNomVolt;

	// calc new B with trapezoidal rule
	//Real B = (1/(1+Fac1)) * (Fac2 * (mDeltaV + deltaVPrev) + (1-Fac1) * mBPrev);
	Real B = (1 / (1 + Fac1)) * (Fac2 * (mDeltaV + deltaVPrev) + (1 - Fac1) * mBPrev);
	//mSLog->info("New B value: percent={}, absolute={}", 100 * B, B * mBN);

	// check bounds
	if (B > mBMax) {
		B =  mBMax;
		mSLog->info("New B value exceeds Bmax");
	}
	else if(B < mBMin)
	{
		B = mBMin;
		mSLog->info("New B value exceeds Bmin");
	}

	// set new B if it has a new value and difference is big enough
	if (B != mBPrev) {
	//if (B != mBPrev && mBSetCounter > 0.001){
		//mValueChange = true;
		//mBSetCounter = 0;
		Real omega = 2 * M_PI*mFrequencies(0, 0);

		if (B > 0) {
			// model inductive behaviour (decrease voltage)
			Real inductance = 1 / (omega * B * mBN);
			//check if change in reactance is sufficient to trigger a change
			if (Math::abs(1 - inductance / mLPrev) > 0.01)
			{
				mInductiveMode = true;
				mSubInductor->updateInductance(inductance, mDeltaT);
				mSLog->info("Inductive Mode: New Inductance: L = {} [H]", inductance);
				mLPrev = inductance;

				mValueChange = true;
				mBSetCounter = 0;
			}
		}
		else
		{
			// model capacitive behaviour (increase voltage)
			Real capacitance = B * mBN / (-omega);
			//check if change in reactance is sufficient to trigger a change
			if (Math::abs(1 - capacitance / mCPrev) > 0.01)
			{
				mInductiveMode = false;
				mSubCapacitor->updateCapacitance(capacitance, mDeltaT);
				mSLog->info("Capacitive Mode: New Capacitance: C = {} [F]", capacitance);
				mCPrev = capacitance;

				mValueChange = true;
				mBSetCounter = 0;
			}
		}

		// update inductance model
		setSwitchState();
	}
	else {
		mBSetCounter = mBSetCounter + mDeltaT;
	}

	// save values
	mBPrev = B;
	mPrevVoltage = V;
	mVmeasPrev = Vmeas;
}

void DP::Ph1::SVC::setSwitchState() {
	// set switches according to current mode of svc
	if (mInductiveMode) {
		if (!mSubInductorProtectionSwitch->mnaIsClosed()) {
			mSLog->info("Closed Inductor Switch");
			mSubInductorProtectionSwitch->close();
		}
		if (mSubCapacitorProtectionSwitch->mnaIsClosed()) {
			mSubCapacitorProtectionSwitch->open();
			mSLog->info("Opened Capacitor Switch");
		}
	}
	else
	{
		if (mSubInductorProtectionSwitch->mnaIsClosed()) {
			mSubInductorProtectionSwitch->open();
			mSLog->info("Openend Inductor Switch");
		}
		if (!mSubCapacitorProtectionSwitch->mnaIsClosed()) {
			mSubCapacitorProtectionSwitch->close();
			mSLog->info("Closed Capcitor Switch");
		}
	}
}
