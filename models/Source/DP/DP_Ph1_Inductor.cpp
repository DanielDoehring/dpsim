/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Inductor.h>

using namespace CPS;

DP::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	mEquivCurrent = { 0, 0 };
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);
	setTerminalNumber(2);

	addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(mInductance);
	return copy;
}

void DP::Ph1::Inductor::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);

	mEquivCurrent = MatrixComp::Zero(mNumFreqs, 1);
	mEquivCond = MatrixComp::Zero(mNumFreqs, 1);
	mPrevCurrFac = MatrixComp::Zero(mNumFreqs, 1);
}

void DP::Ph1::Inductor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real omega = 2. * PI * frequency;
	Complex impedance = { 0, omega * mInductance };
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

// #### MNA functions ####

void DP::Ph1::Inductor::initVars(Real timeStep) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real a = timeStep / (2. * mInductance);
		Real b = timeStep * 2.*PI * mFrequencies(freq,0) / 2.;

		Real equivCondReal = a / (1. + b * b);
		Real equivCondImag =  -a * b / (1. + b * b);
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real preCurrFracReal = (1. - b * b) / (1. + b * b);
		Real preCurrFracImag =  (-2. * b) / (1. + b * b);
		mPrevCurrFac(freq,0) = { preCurrFracReal, preCurrFracImag };

		// TODO: check if this is correct or if it should be only computed before the step
		mEquivCurrent(freq,0) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);

		mSLog->info(
			"\n--- Init Vars for frequency {} ---"
			"\na: {}"
			"\nb: {}"
			"\nequivCondReal: {}"
			"\nequivCondImag: {}"
			"\npreCurrFracReal: {}"
			"\npreCurrFracImag: {}"
			"\n--- Initialization from powerflow finished ---",
			mFrequencies(freq, 0),a,b, equivCondReal, equivCondImag, preCurrFracReal, preCurrFracImag);
	}
}

void DP::Ph1::Inductor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	initVars(timeStep);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\nEquiv. current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::complexToString(mEquivCurrent(0,0)));
}

void DP::Ph1::Inductor::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	initVars(timeStep);

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::Inductor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freq,0), mNumFreqs, freq);
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freq,0), mNumFreqs, freq);
		}

		/*
		mSLog->info("-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0))
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(-mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(1));
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(-mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(0));
		}
		*/
	}
}

void DP::Ph1::Inductor::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freqIdx,0));
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freqIdx,0));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freqIdx,0));
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freqIdx,0));
		}

		/*
		mSLog->info("-- Stamp frequency {:d} ---", freqIdx);
		if (terminalNotGrounded(0))
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
		}
		*/
}

void DP::Ph1::Inductor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// Calculate equivalent current source for next time step
		/*
		mSLog->info("Equivalent CurreNt: {} before calc for next time step", Logger::complexToString(mEquivCurrent(freq, 0)));
		mSLog->info(
			"\n--- Calculation parameters ---"
			"\nEquivCond {:s}"
			"\nmIntfVoltage {:s}"
			"\nmPrevCurrFa {:s}"
			"\nmIntfCurrent {:s}",
			Logger::phasorToString(mEquivCond(freq, 0)),
			Logger::phasorToString(mIntfVoltage(0, freq)),
			Logger::phasorToString(mPrevCurrFac(freq, 0)),
			Logger::complexToString(mIntfCurrent(0, 0)));
		*/

		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * mIntfVoltage(0,freq)
			+ mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);

		if (terminalNotGrounded(0)) {
			//mSLog->info("Stamping Equivalent CurreNt: {} to Index {} in RHS Vector\n", Logger::complexToString(mEquivCurrent(freq, 0)), matrixNodeIndex(0));
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq, 0), mNumFreqs, freq);
		}
		if (terminalNotGrounded(1)) {
			//mSLog->info("Stamping Equivalent CurreNt: {} to Index {} in RHS Vector\n", Logger::complexToString(-mEquivCurrent(freq, 0)), matrixNodeIndex(1));
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq, 0), mNumFreqs, freq);
		}



		SPDLOG_LOGGER_DEBUG(mSLog, "MNA EquivCurrent {:s}", Logger::complexToString(mEquivCurrent(freq,0)));
		if (terminalNotGrounded(0)) {
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
				Logger::complexToString(mEquivCurrent(freq, 0)), matrixNodeIndex(0));
			//mSLog->info("Add {:s} to source vector at {:d}",
				//Logger::complexToString(mEquivCurrent(freq, 0)), matrixNodeIndex(0));
		}
		if (terminalNotGrounded(1)) {
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
				Logger::complexToString(-mEquivCurrent(freq, 0)), matrixNodeIndex(1));
			//mSLog->info("Add {:s} to source vector at {:d}",
				//Logger::complexToString(-mEquivCurrent(freq, 0)), matrixNodeIndex(1));
		}
	}
}

void DP::Ph1::Inductor::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// Calculate equivalent current source for next time step
		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * mIntfVoltage(0,freq)
			+ mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), 1, 0, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), 1, 0, freq);
	}
}

void DP::Ph1::Inductor::MnaPreStep::execute(Real time, Int timeStepCount) {
	//mInductor.mSLog->info("PreStep (executing RHS stamp) at time: {} Step {}", time, timeStepCount);
	mInductor.mnaApplyRightSideVectorStamp(mInductor.mRightVector);
}

void DP::Ph1::Inductor::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mInductor.mnaApplyRightSideVectorStampHarm(mInductor.mRightVector);
}

void DP::Ph1::Inductor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mInductor.mnaUpdateVoltage(*mLeftVector);
	mInductor.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::Inductor::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mInductor.mNumFreqs; freq++)
		mInductor.mnaUpdateVoltageHarm(*mLeftVectors[freq], freq);
	mInductor.mnaUpdateCurrentHarm();
}

void DP::Ph1::Inductor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfVoltage(0,freq) = 0;
		if (terminalNotGrounded(1))
			mIntfVoltage(0,freq) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
		if (terminalNotGrounded(0))
			mIntfVoltage(0,freq) = mIntfVoltage(0,freq) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0,freq)));
	}
}

void DP::Ph1::Inductor::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	// v1 - v0
	mIntfVoltage(0,freqIdx) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,freqIdx) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,freqIdx) = mIntfVoltage(0,freqIdx) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0,freqIdx)));
}

void DP::Ph1::Inductor::mnaUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}

void DP::Ph1::Inductor::mnaUpdateCurrentHarm() {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}

// #### Tear Methods ####
void DP::Ph1::Inductor::mnaTearInitialize(Real omega, Real timeStep) {
	initVars(timeStep);
}

void DP::Ph1::Inductor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, 1./mEquivCond(0,0));
}

void DP::Ph1::Inductor::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mEquivCurrent(0,0) = mEquivCond(0,0) * mIntfVoltage(0,0) + mPrevCurrFac(0,0) * mIntfCurrent(0,0);
	Math::addToVectorElement(voltageVector, mTearIdx, mEquivCurrent(0,0) / mEquivCond(0,0));
}

void DP::Ph1::Inductor::mnaTearPostStep(Complex voltage, Complex current) {
	mIntfVoltage(0, 0) = voltage;
	mIntfCurrent(0, 0) = mEquivCond(0,0) * voltage + mEquivCurrent(0,0);

}

void DP::Ph1::Inductor::updateInductance(Real inductance, Real deltaT) {
	mInductance = inductance;
	updateVars(deltaT);
}

void DP::Ph1::Inductor::updateVars(Real deltaT) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real a = deltaT / (2. * mInductance);
		Real b = deltaT * 2.*PI * mFrequencies(freq, 0) / 2.;

		Real equivCondReal = a / (1. + b * b);
		Real equivCondImag = -a * b / (1. + b * b);
		mEquivCond(freq, 0) = { equivCondReal, equivCondImag };
		Real preCurrFracReal = (1. - b * b) / (1. + b * b);
		Real preCurrFracImag = (-2. * b) / (1. + b * b);
		mPrevCurrFac(freq, 0) = { preCurrFracReal, preCurrFracImag };

		// TODO: check if this is correct or if it should be only computed before the step
		mEquivCurrent(freq, 0) = mEquivCond(freq, 0) * mIntfVoltage(0, freq) + mPrevCurrFac(freq, 0) * mIntfCurrent(0, freq);
		mIntfCurrent(0, freq) = mEquivCond(freq, 0) * mIntfVoltage(0, freq) + mEquivCurrent(freq, 0);
	}
}
