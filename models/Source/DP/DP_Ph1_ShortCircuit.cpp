/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_ShortCircuit.h>

using namespace CPS;

DP::Ph1::ShortCircuit::ShortCircuit(String uid, String name,
	Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	setTerminalNumber(1);
	setVirtualNodeNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);
	addAttribute<Real>("SwitchStateChange", &mSwitchStateChange, Flags::read | Flags::write);
}

DP::Ph1::ShortCircuit::ShortCircuit(String name,
	Logger::Level logLevel)
	: ShortCircuit(name, name, logLevel) {
}

DP::Ph1::ShortCircuit::ShortCircuit(String name,
	Real activePower, Real reactivePower, Real volt,
	Logger::Level logLevel)
	: ShortCircuit(name, logLevel) {
	mNomVoltage = volt;
}

SimPowerComp<Complex>::Ptr DP::Ph1::ShortCircuit::clone(String name) {
	// TODO: Is this change is needed when "everything set by initializePOwerflow"??
	// everything set by initializeFromPowerflow
	//return RXLoad::make(name, mLogLevel);
	auto copy = ShortCircuit::make(name, mLogLevel);
	copy->setParameters(mNomVoltage);
	return copy;
}

void DP::Ph1::ShortCircuit::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);
}

void DP::Ph1::ShortCircuit::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();
	if(!parametersSet){
		mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());
		mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
	}

	// NEW for correct initialization
	mIntfVoltage(0, 0) = mTerminals[0]->initialSingleVoltage();
	mIntfCurrent(0, 0) = std::conj(Complex(0, 0) / mIntfVoltage(0, 0));

	Complex vLoadInit = mTerminals[0]->initialSingleVoltage() - mIntfCurrent(0, 0) * mSwitchRClosed;
	mVirtualNodes[0]->setInitialVoltage(vLoadInit);
	

	// NEW for connecting ProtectionSwitch
	mSubProtectionSwitch = std::make_shared<DP::Ph1::Switch>(mName + "_switch", mLogLevel);
	// initially open
	mSubProtectionSwitch->setParameters(mSwitchROpen, mSwitchRClosed);
	mSubProtectionSwitch->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubProtectionSwitch->initialize(mFrequencies);
	mSubProtectionSwitch->initializeFromPowerflow(frequency);

	mResistance = 1e-9;
	mConductance = 1.0 / mResistance;
	mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubResistor->setParameters(mResistance);
	mSubResistor->connect({ SimNode::GND, mVirtualNodes[0] });
	mSubResistor->initialize(mFrequencies);
	mSubResistor->initializeFromPowerflow(frequency);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nResistance: {:f}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mResistance);
}

void DP::Ph1::ShortCircuit::setParameters(Real volt){
	mNomVoltage = volt;
	mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
	parametersSet = true;
}


void DP::Ph1::ShortCircuit::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	if (mSubResistor) {
		mSubResistor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubResistor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	// New for protection switch
	mSubProtectionSwitch->mnaInitialize(omega, timeStep, leftVector);
	for (auto task : mSubProtectionSwitch->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::ShortCircuit::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mSubResistor)
		mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	// NEW for protection switch
	mSubProtectionSwitch->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::ShortCircuit::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	//mSubProtectionSwitch->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::ShortCircuit::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSC.mSubProtectionSwitch->setValueChange(false);
	mSC.mnaApplyRightSideVectorStamp(mSC.mRightVector);
	// NEW for protection switch
	if (mSC.mEventSet) {
		mSC.updateSwitchState(time);
	}
}

void DP::Ph1::ShortCircuit::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSC.mnaUpdateVoltage(*mLeftVector);
	mSC.mnaUpdateCurrent(*mLeftVector);
	mSC.mSubProtectionSwitch->setValueChange(false);
}

void DP::Ph1::ShortCircuit::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::ShortCircuit::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = 0;
	if (mSubResistor)
		mIntfCurrent(0, 0) += mSubResistor->intfCurrent()(0,0);
}

/// new for protection Switch
void DP::Ph1::ShortCircuit::updateSwitchState(Real time) {
	if (mEventSet)
	{
		if (time > mStartTime && !mEventActive)
		{
			mEventActive = true;
			mSwitchStateChange = true;
			mSubProtectionSwitch->close();
			mSLog->info("Closed Switch at {}", (float)time);
			mSubProtectionSwitch->setValueChange(true);
		}
		else if (time > mEndTime)
		{
			mSwitchStateChange = true;
			mSubProtectionSwitch->open();
			mSLog->info("Opened Switch at {}", (float)time);
			mSubProtectionSwitch->setValueChange(true);
			// no next event -> stop evaluating
			mEventSet = false;
		}
	}
}

void DP::Ph1::ShortCircuit::setEvent(Real start, Real end) {
	mStartTime = start;
	mEndTime = end;
	mEventSet = true;
}
