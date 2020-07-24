/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Transformer.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel, Bool withResistiveLosses)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	if (withResistiveLosses) 
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);
	
	setTerminalNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Real>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Real>("L", &mInductance, Flags::write | Flags::read);
	addAttribute<Real>("Rsnubber", &mSnubberResistance, Flags::write | Flags::read);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(std::abs(mRatio), std::arg(mRatio), mResistance, mInductance);
	return copy;
}

void DP::Ph1::Transformer::setParameters(Real ratioAbs, Real ratioPhase,
	Real resistance, Real inductance) {

	Base::Ph1::Transformer::setParameters(ratioAbs, ratioPhase, resistance, inductance);
	
	mSLog->info("Resistance={} [Ohm] Inductance={} [Ohm] (referred to primary side)", resistance, inductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", ratioAbs, ratioPhase);

	parametersSet = true;
}

void DP::Ph1::Transformer::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);
}

void DP::Ph1::Transformer::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// Component parameters are referred to high voltage side.
	// Switch terminals if transformer is connected the other way around.
	if (Math::abs(mRatio) < 1.) {
		mRatio = 1. / mRatio;
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(1) * mRatio );

	// Static calculations from load flow data
	Real omega = 2.*PI* frequency;
	Complex impedance = { mResistance, omega * mInductance };
	mSLog->info("Reactance={} [Ohm] (referred to primary side)", omega * mInductance );
	mIntfVoltage(0,0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	// Create series sub components
	mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({node(0), mVirtualNodes[2]});
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
		mSubInductor->connect({mVirtualNodes[2], mVirtualNodes[0]});
	} else {
		mSubInductor->connect({node(0), mVirtualNodes[0]});
	}
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromPowerflow(frequency);

	// Create parallel sub components
	// A snubber conductance is added on the low voltage side (resistance approximately scaled with LV side voltage)
	mSnubberResistance = std::abs(node(1)->initialSingleVoltage())*1e6;
	mSubSnubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_snub_res", mLogLevel);
	mSubSnubResistor->setParameters(mSnubberResistance);
	mSubSnubResistor->connect({ node(1), DP::SimNode::GND });
	mSubSnubResistor->initialize(mFrequencies);
	mSubSnubResistor->initializeFromPowerflow(frequency);
	mSLog->info("Snubber Resistance={} [Ohm] (connected to LV side)", mSnubberResistance);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({mSubInductor, mSubSnubResistor});
	if (mSubResistor)
		subComponents.push_back(mSubResistor);
	for (auto comp : subComponents) {
		comp->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : comp->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void DP::Ph1::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1.0, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1.0, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex(), mRatio);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1), -mRatio);
	}

	// Add inductive part to system matrix
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(),  mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex());
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}

void DP::Ph1::Transformer::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::Transformer::MnaPreStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaApplyRightSideVectorStamp(mTransformer.mRightVector);
	// NEW for OLTC
	mTransformer.mRatioChange = false;
	mTransformer.updateTapRatio(time, timeStepCount);
}

void DP::Ph1::Transformer::MnaPostStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaUpdateVoltage(*mLeftVector);
	mTransformer.mnaUpdateCurrent(*mLeftVector);
	//mTransformer.updateTapRatio(time, timeStepCount);
}

void DP::Ph1::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mSubInductor->intfCurrent()(0, 0);
}

void DP::Ph1::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0, 0) = 0;
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0, 0)));
}

void DP::Ph1::Transformer::updateTapRatio(Real time, Int timeStepCount) {
	// current lv voltage
	Real lvVoltage = Math::abs(mSubSnubResistor->intfVoltage()(0, 0));

	// calculate voltage diff
	Real deltaV = (mRefLV - lvVoltage) / mRefLV;
	mSLog->info("Voltage difference Vref - V: {}", deltaV * 100);
	mSLog->info(" V: {}", lvVoltage);

	if (timeStepCount)
	{
		// check if this diff is greater than deadband
		if (Math::abs(deltaV) > mDeadband)
		{
			// TODO better calculation of deltat. Could be variable in Simulation
			Real deltaT = time / timeStepCount;
			if (mViolationCounter > mTapChangeTimeDelay)
			{
				// over or under voltage
				if (deltaV < 0)
				{
					// Overvoltage
					if (Math::abs(mCurrTapPos) < mNumTaps)
					{
						mCurrTapPos = mCurrTapPos + 1;
					}
					
				}
				else
				{
					// undervoltage
					if (Math::abs(mCurrTapPos) < mNumTaps)
					{
						mCurrTapPos = mCurrTapPos - 1;
					}
				}
				// calculate new tap position
				mRatio.real(Math::abs(mRatioInitial) * (1 + mCurrTapPos * mDeltaVTapChange));
				mRatioChange = true;
				mViolationCounter = 0;
			}
			else
			{
				// increase counter
				mViolationCounter = mViolationCounter + deltaT;
			}
		}
		else
		{
			// reset counter
			mViolationCounter = 0;
		}
	}
	/*
	if (timeStep == 10000) {
		mRatio = mRatio * 1.1;
		mRatioChange = true;
	}
	*/
		
}
