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
	Logger::Level logLevel, Bool withResistiveLosses, Bool WithSaturation)
	: SimPowerComp<Complex>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	if (withResistiveLosses) 
		setVirtualNodeNumber(5);
	else
		setVirtualNodeNumber(3);
	
	setTerminalNumber(2);

	mWithSaturation = WithSaturation;

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Real>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Real>("L", &mInductance, Flags::write | Flags::read);
	addAttribute<Real>("Rsnubber", &mSnubberResistance, Flags::write | Flags::read);

	addAttribute<Real>("flux", &mCurrentFlux, Flags::write | Flags::read);
	addAttribute<Real>("deltaFlux", &mDeltaFlux, Flags::write | Flags::read);
	addAttribute<Real>("Vm", &mVm, Flags::write | Flags::read);
	addAttribute<Complex>("ISrcRef", &mISrcRef, Flags::write | Flags::read);
	addAttribute<Real>("IMag", &mIMag, Flags::write | Flags::read);
	addAttribute<Real>("ILMag", &mLMagCurrentReal, Flags::write | Flags::read);
	addAttribute<Real>("VmAngle", &mVmAngle, Flags::write | Flags::read);

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
	/// ?
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(1) * mRatio);

	// Static calculations from load flow data
	// use t-config
	// [0]     
	// [0] -- R/2 --[V2]-- L/2 -- [V3] -- L/2 --[V4]-- R/2 -- ooo -- [1]
	//							   |
	//                             Lm
	//					           |
	//--------------------------------------------------------GND

	Real omega = 2.*PI* frequency;
	//Complex impedance = { mResistance, omega * mInductance };
	mSLog->info("Leakage Reactance={} [Ohm]", omega * mInductance);
	mSLog->info("Magnetizing Reactance = {} [Ohm]", omega * mLm);

	mIntfVoltage(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	// series impedance made of half of the whole Rloss und Lleakage
	Complex seriesImpedance = Complex(mResistance / 2, omega * mInductance / 2);

	// since elements are symmetric voltage drop will be exactly half at the middle
	Complex VmInit = mVirtualNodes[0]->initialSingleVoltage() - mIntfVoltage(0, 0) / Complex(2, 0);
	mVm = VmInit.real();

	// Current flowing into HV side
	Complex IHVInit = (initialSingleVoltage(0) - VmInit) / seriesImpedance;
	// Current flowing into Lv side
	Complex ILVInit = (mVirtualNodes[0]->initialSingleVoltage() - VmInit) / seriesImpedance;
	// initial magnetizing current
	Complex ImInit = VmInit / Complex(0., omega * mLm);

	// init of virtual voltages
	Complex VRHVInit = initialSingleVoltage(0) - IHVInit * Complex(mResistance / 2, .0);
	Complex VRLVInit = mVirtualNodes[0]->initialSingleVoltage() - ILVInit * Complex(mResistance / 2, .0);

	mVirtualNodes[2]->setInitialVoltage(VRHVInit);
	mVirtualNodes[3]->setInitialVoltage(VmInit);
	mVirtualNodes[4]->setInitialVoltage(VRLVInit);

	// Create series sub components
	// leakage inductances
	mSubLeakageInductorHV = std::make_shared<DP::Ph1::Inductor>(mName + "_indHV", mLogLevel);
	mSubLeakageInductorHV->setParameters(mInductance / 2);
	mSubLeakageInductorLV = std::make_shared<DP::Ph1::Inductor>(mName + "_indLV", mLogLevel);
	mSubLeakageInductorLV->setParameters(mInductance / 2);
	// loss resistances
	mSubLossResistorHV = std::make_shared<DP::Ph1::Resistor>(mName + "_resLossHV", mLogLevel);
	mSubLossResistorHV->setParameters(mResistance / 2);
	mSubLossResistorLV = std::make_shared<DP::Ph1::Resistor>(mName + "_resLossLV", mLogLevel);
	mSubLossResistorLV->setParameters(mResistance / 2);

	/*
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
	*/

	// connection and init of elements
	// series elements
	mSubLeakageInductorHV->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubLeakageInductorHV->initialize(mFrequencies);
	mSubLeakageInductorHV->initializeFromPowerflow(frequency);
	mSLog->info("Connected Series Leakage Inductance (HV) = {} [H]", mInductance / 2);
	mSubLeakageInductorLV->connect({ mVirtualNodes[3], mVirtualNodes[4] });
	mSubLeakageInductorLV->initialize(mFrequencies);
	mSubLeakageInductorLV->initializeFromPowerflow(frequency);
	mSLog->info("Connected Series Leakage Inductance (LV) = {} [H]", mInductance / 2);

	mSubLossResistorHV->connect({mVirtualNodes[2], node(0)});
	mSubLossResistorHV->initialize(mFrequencies);
	mSubLossResistorHV->initializeFromPowerflow(frequency);
	mSLog->info("Connected Series Resistance (HV) = {} [Ohm]", mResistance / 2);
	mSubLossResistorLV->connect({ mVirtualNodes[4], mVirtualNodes[0] });
	mSubLossResistorLV->initialize(mFrequencies);
	mSubLossResistorLV->initializeFromPowerflow(frequency);
	mSLog->info("Connected Series Resistance (LV) = {} [Ohm]", mResistance / 2);

	// Create parallel sub components at LV side
	// A snubber conductance is added on the low voltage side (resistance approximately scaled with LV side voltage)
	mSnubberResistance = std::abs(node(1)->initialSingleVoltage())*1e6;
	mSubSnubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_snub_res", mLogLevel);
	mSubSnubResistor->setParameters(mSnubberResistance);
	mSubSnubResistor->connect({ node(1), DP::SimNode::GND });
	mSubSnubResistor->initialize(mFrequencies);
	mSubSnubResistor->initializeFromPowerflow(frequency);
	mSLog->info("Snubber Resistance={} [Ohm] (connected to LV side)", mSnubberResistance);

	if (mWithSaturation) {
		// parallel components
		mSLog->info("Modeling saturation effects");

		// current source for saturation
		mSubSatCurrentSrc = std::make_shared<DP::Ph1::CurrentSource>(mName + "_sat_current_src", mLogLevel);
		// init with zero value. Could be improved
		mSubSatCurrentSrc->setParameters(Complex(0, 0));
		mSubSatCurrentSrc->connect({mVirtualNodes[3], SimNode::GND});
		mSubSatCurrentSrc->initialize(mFrequencies);
		mSubSatCurrentSrc->initializeFromPowerflow(frequency);
		mSLog->info("Connected Current Source for saturation");

		// magnetizing inductance
		mSubMagnetizingInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_indMagnetizing", mLogLevel);
		mSubMagnetizingInductor->setParameters(mLm);

		mSubMagnetizingInductor->connect({ mVirtualNodes[3], SimNode::GND });
		mSubMagnetizingInductor->initialize(mFrequencies);
		mSubMagnetizingInductor->initializeFromPowerflow(frequency);
		mSLog->info("Connected parallel magnetizing Inductance (HV) = {} [H]", mLm);

		if (!mSatConstantsSet) {
			setSaturationConstants();
			mSatConstantsSet = true;
		}

		// log saturation constants
		mSLog->info("Saturation Constant A: {} ", mSatConstA);
		mSLog->info("Saturation Constant B: {} ", mSatConstB);
		mSLog->info("Saturation Constant C: {} ", mSatConstC);
		mSLog->info("Saturation Constant D: {} ", mSatConstD);
	}


	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVoltage across magnetizing impedance: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVm),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({mSubLeakageInductorHV, mSubLeakageInductorLV , mSubLossResistorHV, mSubLossResistorLV, mSubSnubResistor});
	//if (mSubResistor)
		//subComponents.push_back(mSubResistor);
	if (mWithSaturation) {
		subComponents.push_back(mSubSatCurrentSrc);
		subComponents.push_back(mSubMagnetizingInductor);
	}
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
	mSubLeakageInductorHV->mnaApplySystemMatrixStamp(systemMatrix);
	mSubLeakageInductorLV->mnaApplySystemMatrixStamp(systemMatrix);

	mSubLossResistorHV->mnaApplySystemMatrixStamp(systemMatrix);
	mSubLossResistorLV->mnaApplySystemMatrixStamp(systemMatrix);

	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mWithSaturation) {
		mSLog->info("Stamping Current Source");
		mSubSatCurrentSrc->mnaApplySystemMatrixStamp(systemMatrix);
		mSubMagnetizingInductor->mnaApplySystemMatrixStamp(systemMatrix);
	}
	/*
	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}
	*/

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
	//mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	mSubLeakageInductorHV->mnaApplyRightSideVectorStamp(rightVector);
	mSubLeakageInductorLV->mnaApplyRightSideVectorStamp(rightVector);
	if (mWithSaturation)
	{
		mSubSatCurrentSrc->mnaApplyRightSideVectorStamp(rightVector);
		mSubMagnetizingInductor->mnaApplyRightSideVectorStamp(rightVector);
	}

}

void DP::Ph1::Transformer::MnaPreStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaApplyRightSideVectorStamp(mTransformer.mRightVector);
	// NEW for OLTC
	mTransformer.mRatioChange = false;
	mTransformer.updateTapRatio(time, timeStepCount);
	mTransformer.updateSatCurrentSrc(time);
	if (mTransformer.mWithSaturation)
	{
		mTransformer.updateSatCurrentSrc(time);
	}
	
}

void DP::Ph1::Transformer::MnaPostStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaUpdateVoltage(*mLeftVector);
	mTransformer.mnaUpdateCurrent(*mLeftVector);
	if (mTransformer.mWithSaturation)
	{
		mTransformer.updateSatCurrentSrc(time, *mLeftVector);
	}
}

void DP::Ph1::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	// HV level 
	mIntfCurrent(0,0) = mSubLeakageInductorHV->intfCurrent()(0, 0);
}

void DP::Ph1::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0, 0) = 0;
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0, 0)));

	/*
	// log virtual node voltages
	mSLog->info("Voltage at VNode[0]: {}", Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex()));
	mSLog->info("Voltage at VNode[1]: {}", Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex()));
	mSLog->info("Voltage at Node 1 (LV): {}", Math::complexFromVectorElement(leftVector, matrixNodeIndex(1)));
	*/
}

void DP::Ph1::Transformer::updateTapRatio(Real time, Int timeStepCount) {
	// current lv voltage
	Real lvVoltage = Math::abs(mSubSnubResistor->intfVoltage()(0, 0));

	// calculate voltage diff
	Real deltaV = (mRefLV - lvVoltage) / mRefLV;

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

						mSLog->info("\nIncreasing tap Position (Overvoltage) to: {}", mCurrTapPos);
						mSLog->info("Voltage difference Vref - V: {} [%]", deltaV * -100);
						mSLog->info("V: {}", lvVoltage);
					}
					
				}
				else
				{
					// undervoltage
					if (Math::abs(mCurrTapPos) < mNumTaps)
					{
						mCurrTapPos = mCurrTapPos - 1;

						mSLog->info("\nDecreasing tap Position (Undervoltage) to: {}", mCurrTapPos);
						mSLog->info("Voltage difference Vref - V: {} [%]", deltaV * -100);
						mSLog->info("V: {}", lvVoltage);
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
void DP::Ph1::Transformer::updateSatCurrentSrc(Real time) {
	// first update flux
	updateFlux(time, leftVector);

	Real omega = 2. * PI * mFrequencies(0, 0);

	// now calculate correct magnetizing current
	/*
	// correct for negative values
	Real negativeFlux = (mCurrentFlux < 0) ? -1 : 1;
	Real iMag_sqrt = sqrt((Math::abs(mCurrentFlux) - mLambdaK) * (Math::abs(mCurrentFlux) - mLambdaK) + 4. * mSatConstD * mLA);
	mIMag = negativeFlux * ( ((iMag_sqrt + Math::abs(mCurrentFlux) - mLambdaK) / (2 * mLA)) - (mSatConstD / mLambdaK) );
	*/

	Real iMag_sqrt = sqrt( (mCurrentFlux - mLambdaK) * (mCurrentFlux - mLambdaK) + 4 * mSatConstD * mLA );
	mIMag = ( (iMag_sqrt + mCurrentFlux - mLambdaK) / (2 * mLA) ) - (mSatConstD / mLambdaK);
	mSLog->info("\nCurrent Flux of {} leads to magnetizing current of {}", mCurrentFlux, mIMag);

	// calc new ref value for current source
	// I_currSrc = Imagnetizing - I_Lm
	// -> get real value of current through magnetizing inductance
	//Real iSrc = mIMag - mCurrentFlux / mLm;
	//Complex lMagCurrent = mSubMagnetizingInductor->intfCurrent()(0, 0);
	//mLMagCurrentReal = Math::abs(lMagCurrent) * cos(omega*time + Math::phase(lMagCurrent));
	mLMagCurrentReal = mCurrentFlux / mLm;
	Real iSrc = mIMag - mLMagCurrentReal;
	mSLog->info("Current Source Real part = Im - Ilm |\n {} = {} - {} at {}", iSrc, mIMag, mLMagCurrentReal, time);

	// Now this needs to be again transformed into DP-Domain
	// multiply with e^-jw_s*t
	mISrcRef = Complex(iSrc*cos(omega*time), -iSrc*sin(omega*time));

	// set ref value for internal current source
	mSubSatCurrentSrc->setRefCurrent(mISrcRef);
}

void DP::Ph1::Transformer::updateFlux(Real time, const Matrix& leftVector) {
	if (time > 0) {
		// update flux value through integration of voltage
		Real deltaT = time - mPrevStepTime;

		// transform values from dp to emt
		Real omega = 2. * PI * mFrequencies(0, 0);

		// transform DP to EMT
		// Real part of voltage drom DP domain
		//Complex currentVoltage = mSubMagnetizingInductor->intfVoltage()(0, 0);
	    //Complex currentVoltage = mSubSatCurrentSrc->intfVoltage()(0, 0);
		//Complex currentVoltage = mIntfVoltage(0, 0) - mSubLeakageInductorHV->intfCurrent()(0, 0) * Complex(mResistance / 2, omega * mInductance / 2);

		Complex currentVoltage = Math::complexFromVectorElement(leftVector, mVirtualNodes[3]->matrixNodeIndex());

		Real currVolAbs = Math::abs(currentVoltage);
		Real currVolAngl = Math::phase(currentVoltage);
		mVmAngle = currVolAngl;

		// Re{|Voltage| * e^(j*angle) * e^(jw_s*t)}
		Real currentVoltageReal = Math::abs(currentVoltage) * cos(omega*time + Math::phase(currentVoltage));

		// using trapez rule of integration
		// Use actual value so multiply rms value with sqrt(2)
		mDeltaFlux = (deltaT / 2) * (mVm + currentVoltageReal);

		// update magnetizing voltage
		mVm = currentVoltageReal;
		mSLog->info("Current Voltage across magnetizing branch: {} \n", mVm);

		// update flux
		mCurrentFlux = mCurrentFlux + mDeltaFlux;
	}
	mPrevStepTime = time;
}
