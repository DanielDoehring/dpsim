/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_RXLoad.h>

using namespace CPS;

DP::Ph1::RXLoad::RXLoad(String uid, String name,
	Logger::Level logLevel, Bool SwitchActive)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	setTerminalNumber(1);

	// NEW for protection
	mSwitchActive = SwitchActive;
	setVirtualNodeNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("P", &mActivePower, Flags::read | Flags::write);
	addAttribute<Real>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);

	// NEW attribute for debugging
	addAttribute<Real>("SwitchStateChange", &mSwitchStateChange, Flags::read | Flags::write);
}

DP::Ph1::RXLoad::RXLoad(String name,
	Logger::Level logLevel)
	: RXLoad(name, name, logLevel, false) {
}

DP::Ph1::RXLoad::RXLoad(String name,
	Real activePower, Real reactivePower, Real volt,
	Logger::Level logLevel)
	: RXLoad(name, logLevel) {
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = { mActivePower, mReactivePower };
	mNomVoltage = volt;
}

SimPowerComp<Complex>::Ptr DP::Ph1::RXLoad::clone(String name) {
	// TODO: Is this change is needed when "everything set by initializePOwerflow"??
	// everything set by initializeFromPowerflow
	//return RXLoad::make(name, mLogLevel);
	auto copy = RXLoad::make(name, mLogLevel);
	copy->setParameters(mActivePower, mReactivePower, mNomVoltage);
	return copy;
}

void DP::Ph1::RXLoad::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);
}

void DP::Ph1::RXLoad::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();
	if(!parametersSet){
		mActivePower = mTerminals[0]->singleActivePower();
		mReactivePower = mTerminals[0]->singleReactivePower();
		mPower = { mActivePower, mReactivePower };
		mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

		mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mActivePower, mReactivePower);
		mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
	}

	// NEW for correct initialization
	mIntfVoltage(0, 0) = mTerminals[0]->initialSingleVoltage();
	mIntfCurrent(0, 0) = std::conj(Complex(mActivePower, mReactivePower) / mIntfVoltage(0, 0));

	Complex vLoadInit = mTerminals[0]->initialSingleVoltage() - mIntfCurrent(0, 0) * mSwitchRClosed;
	if (Math::abs(vLoadInit) != Math::abs(vLoadInit)) {
		mVirtualNodes[0]->setInitialVoltage(mNomVoltage);
	}
	else
	{
		mVirtualNodes[0]->setInitialVoltage(vLoadInit);
	}

	// NEW for connecting ProtectionSwitch
	mSubProtectionSwitch = std::make_shared<DP::Ph1::Switch>(mName + "_switch", mLogLevel);
	mSubProtectionSwitch->setParameters(mSwitchROpen, mSwitchRClosed, true);
	mSubProtectionSwitch->connect({ mVirtualNodes[0], mTerminals[0]->node() });
	mSubProtectionSwitch->initialize(mFrequencies);
	mSubProtectionSwitch->initializeFromPowerflow(frequency);

	if (mActivePower != 0) {
		mResistance = std::pow(mNomVoltage, 2) / mActivePower;
		mConductance = 1.0 / mResistance;
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mVirtualNodes[0] });
		//mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
	}

	if (mReactivePower != 0)
		mReactance = std::pow(mNomVoltage, 2) / mReactivePower;
	else
		mReactance = 0;

	if (mReactance > 0) {
		mInductance = mReactance / (2 * PI*frequency);

		mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mVirtualNodes[0] });
		//mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromPowerflow(frequency);
	} else if (mReactance < 0) {
		mCapacitance = -1 / (2*PI*frequency) / mReactance;

		mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mVirtualNodes[0] });
		//mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromPowerflow(frequency);
	}

	/* old 
	mIntfVoltage(0, 0) = mTerminals[0]->initialSingleVoltage();
	mIntfCurrent(0, 0) = std::conj(Complex(mActivePower, mReactivePower) / mIntfVoltage(0, 0));
	*/

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nResistance: {:f}"
		"\nReactance: {:f}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mResistance,
		mReactance);
}

void DP::Ph1::RXLoad::setParameters(Real activePower, Real reactivePower, Real volt){
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = { mActivePower, mReactivePower};
	mNomVoltage = volt;

	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mActivePower, mReactivePower);
	mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
	parametersSet = true;
}


void DP::Ph1::RXLoad::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
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
	if (mSubProtectionSwitch) {
		mSubProtectionSwitch->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubProtectionSwitch->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubInductor) {
		mSubInductor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubInductor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubCapacitor) {
		mSubCapacitor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubCapacitor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::RXLoad::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mSubResistor)
		mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	// NEW for protection switch
	if (mSubProtectionSwitch)
		mSubProtectionSwitch->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubInductor)
		mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::RXLoad::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubInductor)
		mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::RXLoad::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaApplyRightSideVectorStamp(mLoad.mRightVector);
	// NEW for protection switch
	if(mLoad.mSwitchActive)
		mLoad.updateSwitchState(time);
}

void DP::Ph1::RXLoad::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaUpdateVoltage(*mLeftVector);
	mLoad.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::RXLoad::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::RXLoad::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = 0;
	if (mSubResistor)
		mIntfCurrent(0, 0) += mSubResistor->intfCurrent()(0,0);
	if (mSubInductor)
		mIntfCurrent(0, 0) += mSubInductor->intfCurrent()(0,0);
	if (mSubCapacitor)
		mIntfCurrent(0, 0) += mSubCapacitor->intfCurrent()(0,0);
}

/// new for protection Switch
void DP::Ph1::RXLoad::updateSwitchState(Real time) {
	mSLog->info("Switch Status: {}", (float)mSubProtectionSwitch->attribute<Bool>("is_closed")->get());
	if (time > 1 && !mSwitchStateChange) {
		Real VRef = Math::abs(mNomVoltage);
		Real V = Math::abs(mIntfVoltage(0, 0));

		//Real deltaV = Math::abs((V - VRef) / VRef);

		Real deltaV = 1;
		if (deltaV > 0.1)
		{
			mSwitchStateChange = true;
			mSubProtectionSwitch->open();
			mSLog->info("Opened Switch at {}", (float)time);
		}
	}
}
