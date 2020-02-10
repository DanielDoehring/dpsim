/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * 			Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/DP/DP_Ph3_Resistor.h>

using namespace CPS;

DP::Ph3::Resistor::Resistor(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(3,1);
	mIntfCurrent = MatrixComp::Zero(3,1);

	addAttribute<Matrix>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph3::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(mResistance);
	return copy;
}

void DP::Ph3::Resistor::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);
}

void DP::Ph3::Resistor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real voltMag = Math::abs(mIntfVoltage(0, 0));
	Real voltPhase = Math::phase(mIntfVoltage(0, 0));
	mIntfVoltage(1, 0) = Complex(
		voltMag * cos(voltPhase - 2. / 3. * M_PI),
		voltMag * sin(voltPhase - 2. / 3. * M_PI));
	mIntfVoltage(2, 0) = Complex(
		voltMag * cos(voltPhase + 2. / 3. * M_PI),
		voltMag * sin(voltPhase + 2. / 3. * M_PI));
	mConductance = mResistance.inverse();
	mIntfCurrent = mConductance * mIntfVoltage;

	mSLog->info("\n--- Initialization from powerflow ---"
		"\nVoltage across amplitude and phase: \n{:s}"
		"\nCurrent amplitude and phase: \n{:s}"
		"\nTerminal 0 voltage amplitude and phase: \n{:s}"
		"\nTerminal 1 voltage amplitude and phase: \n{:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorMatrixToString(mIntfVoltage),
		Logger::phasorMatrixToString(mIntfCurrent),
		Logger::phasorMatrixToString(initialVoltage(0)),
		Logger::phasorMatrixToString(initialVoltage(1)));
}

void DP::Ph3::Resistor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph3::Resistor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {

	//// Set diagonal entries
	//if (terminalNotGrounded(0))
	//	Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(0), conductance);
	//if (terminalNotGrounded(1))
	//	Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(1), conductance);
	//// Set off diagonal entries
	//if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
	//	Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(1), -conductance);
	//	Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(0), -conductance);
	//}
	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), Complex(mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 1), Complex(mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 2), Complex(mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 0), Complex(mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), Complex(mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 2), Complex(mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 0), Complex(mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 1), Complex(mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), Complex(mConductance(2, 2), 0));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), Complex(mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 1), Complex(mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 2), Complex(mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 0), Complex(mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), Complex(mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 2), Complex(mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 0), Complex(mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 1), Complex(mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), Complex(mConductance(2, 2), 0));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -Complex(mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 1), -Complex(mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 2), -Complex(mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 0), -Complex(mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -Complex(mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 2), -Complex(mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 0), -Complex(mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 1), -Complex(mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -Complex(mConductance(2, 2), 0));


		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -Complex(mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 1), -Complex(mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 2), -Complex(mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 0), -Complex(mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -Complex(mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 2), -Complex(mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 0), -Complex(mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 1), -Complex(mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -Complex(mConductance(2, 2), 0));
	}

	//if (terminalNotGrounded(0))
	//	mSLog->info("Add {} to {}, {}", conductance, simNode(0,0), simNode(0,0));
	//if (terminalNotGrounded(1))
	//	mSLog->info("Add {} to {}, {}", conductance, simNode(1,0), simNode(1,0));
	//if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
	//	mSLog->info("Add {} to {}, {}", -conductance, simNode(0,0), simNode(1,0));
	//	mSLog->info("Add {} to {}, {}", -conductance, simNode(1,0), simNode(0,0));
	//}
}

void DP::Ph3::Resistor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mResistor.mnaUpdateVoltage(*mLeftVector);
	mResistor.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph3::Resistor::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = MatrixComp::Zero(3,1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1,0));
		mIntfVoltage(1,0) = Math::complexFromVectorElement(leftVector, simNode(1,1));
		mIntfVoltage(2,0) = Math::complexFromVectorElement(leftVector, simNode(1,2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0,0));
		mIntfVoltage(1,0) = mIntfVoltage(1,0) - Math::complexFromVectorElement(leftVector, simNode(0,1));
		mIntfVoltage(2,0) = mIntfVoltage(2,0) - Math::complexFromVectorElement(leftVector, simNode(0,2));
	}

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}", std::abs(mIntfVoltage(0,0)), std::arg(mIntfVoltage(0,0)));
}

void DP::Ph3::Resistor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mConductance * mIntfVoltage;

	SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}", std::abs(mIntfCurrent(0,0)), std::arg(mIntfCurrent(0,0)));
}
