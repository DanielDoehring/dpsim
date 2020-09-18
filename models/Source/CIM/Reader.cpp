/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <CIMModel.hpp>
#include <IEC61970.hpp>
#include <memory>
#include <CIMExceptions.hpp>

#define READER_CPP
#include <cps/CIM/Reader.h>

using namespace CPS;
using namespace CPS::CIM;
using namespace IEC61970::Base::Core;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Equivalents;
using namespace IEC61970::Base::Wires;
using namespace IEC61970::Base::StateVariables;

namespace fs = std::experimental::filesystem;

Reader::Reader(String name, Logger::Level logLevel, Logger::Level componentLogLevel) {
	mSLog = Logger::get(name + "_CIM", logLevel);

	mModel = new CIMModel();
	mModel->setDependencyCheckOff();
	mComponentLogLevel = componentLogLevel;
}

Reader::~Reader() {
	delete mModel;
}

Real Reader::unitValue(Real value, UnitMultiplier mult) {
	switch (mult) {
	case UnitMultiplier::p:
		value *= 1e-12;
		break;
	case UnitMultiplier::n:
		value *= 1e-9;
		break;
	case UnitMultiplier::micro:
		value *= 1e-6;
		break;
	case UnitMultiplier::m:
		value *= 1e-3;
		break;
	case UnitMultiplier::c:
		value *= 1e-2;
		break;
	case UnitMultiplier::d:
		value *= 1e-1;
		break;
	case UnitMultiplier::k:
		value *= 1e3;
		break;
	case UnitMultiplier::M:
		value *= 1e6;
		break;
	case UnitMultiplier::G:
		value *= 1e9;
		break;
	case UnitMultiplier::T:
		value *= 1e12;
		break;
	default:
		break;
	}
	return value;
}

TopologicalPowerComp::Ptr Reader::mapComponent(BaseClass* obj) {
	if (ACLineSegment *line = dynamic_cast<ACLineSegment*>(obj))
		return mapACLineSegment(line);
	if (EnergyConsumer *consumer = dynamic_cast<EnergyConsumer*>(obj))
		return mapEnergyConsumer(consumer);
	if (PowerTransformer *trans = dynamic_cast<PowerTransformer*>(obj))
		return mapPowerTransformer(trans);
	if (SynchronousMachine *syncMachine = dynamic_cast<SynchronousMachine*>(obj))
		return mapSynchronousMachine(syncMachine);
	if (ExternalNetworkInjection *extnet = dynamic_cast<ExternalNetworkInjection*>(obj))
		return mapExternalNetworkInjection(extnet);
	if (EquivalentShunt *shunt = dynamic_cast<EquivalentShunt*>(obj))
		return mapEquivalentShunt(shunt);
	if (LinearShuntCompensator *shunt = dynamic_cast<LinearShuntCompensator*>(obj))
		return mapLinearShuntCompensator(shunt);
	if (StaticVarCompensator *svc = dynamic_cast<StaticVarCompensator*>(obj))
		return mapStaticVarCompensator(svc);
	return nullptr;
}

///
Matrix Reader::singlePhaseParameterToThreePhase(Real parameter) {
	Matrix param_3ph = Matrix::Zero(3, 3);
	param_3ph <<
		parameter, 0., 0.,
		0., parameter, 0.,
		0, 0., parameter;
	return param_3ph;
}

void Reader::addFiles(const fs::path &filename) {
	if (!mModel->addCIMFile(filename.string()))
		mSLog->error("Failed to read file {}", filename);
}

void Reader::addFiles(const std::list<fs::path> &filenames) {
	for (auto filename : filenames)
		addFiles(filename);
}

void Reader::parseFiles() {
	try {
		mModel->parseFiles();
	}
	catch (...) {
		mSLog->error("Failed to parse CIM files");
		return;
	}

	mSLog->info("#### List of TopologicalNodes, associated Terminals and Equipment");
	for (auto obj : mModel->Objects) {
		if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
			if (mDomain == Domain::EMT)
				processTopologicalNode<Real>(topNode);
			else
				processTopologicalNode<Complex>(topNode);
		}
	}

	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mSLog->info("#### List of Node voltages and Terminal power flow data");
	for (auto obj : mModel->Objects) {
		// Check if object is of class SvVoltage
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			processSvVoltage(volt);
		}
		// Check if object is of class SvPowerFlow
		else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			processSvPowerFlow(flow);
		}
	}

	mSLog->info("#### Create other components");
	for (auto obj : mModel->Objects) {

		// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
		if (!dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)
			&& !dynamic_cast<SvVoltage*>(obj)
			&& !dynamic_cast<SvPowerFlow*>(obj)) {

			if (IEC61970::Base::Core::IdentifiedObject* idObj = dynamic_cast<IEC61970::Base::Core::IdentifiedObject*>(obj)) {

				// Check if object is already in equipment list
				if (mPowerflowEquipment.find(idObj->mRID) == mPowerflowEquipment.end()) {
					TopologicalPowerComp::Ptr comp = mapComponent(obj);
					if (comp)
						mPowerflowEquipment.insert(std::make_pair(comp->uid(), comp));
				}
			}
		}
	}

	mSLog->info("#### Check topology for unconnected components");
	for (auto pfe : mPowerflowEquipment) {
		auto c = pfe.second;

		if (mDomain == Domain::EMT) {
			if(SimPowerComp<Real>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Real>>(c)) {
				if (powercomp->terminalNumberConnected() < powercomp->terminalNumber())
					throw InvalidTopology();
			}
		}
		else {
			if(SimPowerComp<Complex>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(c)) {
				if (powercomp->terminalNumberConnected() < powercomp->terminalNumber())
					throw InvalidTopology();
			}
		}
	}
}

SystemTopology Reader::loadCIM(Real systemFrequency, const fs::path &filename, Domain domain, PhaseType phase) {
	mFrequency = systemFrequency;
	mOmega = 2 * PI*mFrequency;
	mDomain = domain;
	mPhase = phase;
	addFiles(filename);
	parseFiles();
	return systemTopology();
}

SystemTopology Reader::loadCIM(Real systemFrequency, const std::list<fs::path> &filenames, Domain domain, PhaseType phase) {
	mFrequency = systemFrequency;
	mOmega = 2 * PI*mFrequency;
	mDomain = domain;
	mPhase = phase;
	addFiles(filenames);
	parseFiles();
	return systemTopology();
}

void Reader::processSvVoltage(SvVoltage* volt) {
	IEC61970::Base::Topology::TopologicalNode* node = volt->TopologicalNode;
	if (!node) {
		mSLog->warn("SvVoltage references missing Topological Node, ignoring");
		return;
	}
	auto search = mPowerflowNodes.find(node->mRID);
	if (search == mPowerflowNodes.end()) {
		mSLog->warn("SvVoltage references Topological Node {}"
			" missing from mTopNodes, ignoring", node->mRID);
		return;
	}

	Real voltageAbs = Reader::unitValue(volt->v.value, UnitMultiplier::k);

	try{
		mSLog->info("    Angle={}", (float)volt->angle.value);
	}catch(ReadingUninitializedField* e ){
		volt->angle.value = 0;
		std::cerr<< "Uninitialized Angle for SVVoltage at " << volt->TopologicalNode->name << ".Setting default value of " << volt->angle.value << std::endl;
	}
	Real voltagePhase = volt->angle.value * PI / 180;
	mPowerflowNodes[node->mRID]->setInitialVoltage(std::polar<Real>(voltageAbs, voltagePhase));

	mSLog->info("Node {} MatrixNodeIndex {}: {} V, {} deg",
		mPowerflowNodes[node->mRID]->uid(),
		mPowerflowNodes[node->mRID]->matrixNodeIndex(),
		std::abs(mPowerflowNodes[node->mRID]->initialSingleVoltage()),
		std::arg(mPowerflowNodes[node->mRID]->initialSingleVoltage())*180/PI
	);
}

void Reader::processSvPowerFlow(SvPowerFlow* flow) {
	IEC61970::Base::Core::Terminal* term = flow->Terminal;

	mPowerflowTerminals[term->mRID]->setPower(
		Complex(Reader::unitValue(flow->p.value, UnitMultiplier::M),
		Reader::unitValue(flow->q.value, UnitMultiplier::M)));

	mSLog->warn("Terminal {}: {} W + j {} Var",
		term->mRID,
		mPowerflowTerminals[term->mRID]->singleActivePower(),
		mPowerflowTerminals[term->mRID]->singleReactivePower());
}

SystemTopology Reader::systemTopology() {
	SystemTopology system(mFrequency);

	for (auto comp : mPowerflowEquipment) {
		system.addComponent(comp.second);
		// TODO support Real
		if (SimPowerComp<Complex>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp.second)) {
			for (auto term : powercomp->topologicalTerminals()) {
				TopologicalNode::Ptr node=term->topologicalNodes();
			//TopologicalNode::Ptr node = powercomp->topologicalTerminals().back()->topologicalNodes();
			if (system.mComponentsAtNode.find(node) == system.mComponentsAtNode.end()) {
				TopologicalPowerComp::List complist;
				complist.push_back(powercomp);
				system.mComponentsAtNode.insert(std::make_pair(node, complist));
			}
			else {
				system.mComponentsAtNode[node].push_back(powercomp);
			}
			}
		}
	}

	system.mNodes.resize(mPowerflowNodes.size());

	for (auto node : mPowerflowNodes) {
		// The index of the node in the list should not matter anymore
		//system.mNodes[node.second->matrixNodeIndex()] = node.second;
		system.addNodeAt(node.second, node.second->matrixNodeIndex());
	}

	return system;
}

Matrix::Index Reader::mapTopologicalNode(String mrid) {
	auto search = mPowerflowNodes.find(mrid);
	if (search == mPowerflowNodes.end()) {
		return -1;
	}
	return search->second->matrixNodeIndex();
}

TopologicalPowerComp::Ptr Reader::mapEnergyConsumer(EnergyConsumer* consumer) {
	mSLog->info("    Found EnergyConsumer {}", consumer->name);
	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			return std::make_shared<EMT::Ph3::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
		}
		else
		{
		mSLog->info("    RXLoad for EMT not implemented yet");
		return std::make_shared<DP::Ph1::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
		}
	}
	else if (mDomain == Domain::SP) {
		auto load = std::make_shared<SP::Ph1::Load>(consumer->mRID, consumer->name, mComponentLogLevel);

		// TODO: Use EnergyConsumer.P and EnergyConsumer.Q if available, overwrite if existent SvPowerFlow data
		/*
		Real p = 0;
		Real q = 0;
		if (consumer->p.value){
			p = unitValue(consumer->p.value,UnitMultiplier::M);
		}
		if (consumer->q.value){
			q = unitValue(consumer->q.value,UnitMultiplier::M);
		}*/

		// P and Q values will be set according to SvPowerFlow data
		load->setParameters(0, 0, 0);
		load->modifyPowerFlowBusType(PowerflowBusType::PQ); // for powerflow solver set as PQ component as default
		return load;
	}
	else {
		Bool switchActive = false;
		return std::make_shared<DP::Ph1::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel, switchActive);
	}
}

TopologicalPowerComp::Ptr Reader::mapACLineSegment(ACLineSegment* line) {
	mSLog->info("    Found ACLineSegment {} r={} x={} bch={} gch={}", line->name,
		(float) line->r.value,
		(float) line->x.value,
		(float) line->bch.value,
		(float) line->gch.value);

	Real resistance = line->r.value;
	Real inductance = line->x.value / mOmega;

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	Real capacitance = mShuntCapacitorValue;
	Real conductance = mShuntConductanceValue;

	if(line->bch.value > 1e-9 && !mSetShuntCapacitor)
		capacitance = Real(line->bch.value / mOmega);

	if(line->gch.value > 1e-9 && !mSetShuntConductance)
		conductance = Real(line->gch.value);

	Real baseVoltage = 0;
    // first look for baseVolt object to set baseVoltage
    for (auto obj : mModel->Objects) {
        if (IEC61970::Base::Core::BaseVoltage* baseVolt = dynamic_cast<IEC61970::Base::Core::BaseVoltage*>(obj)) {
            for (auto comp : baseVolt->ConductingEquipment) {
                if (comp->name == line->name) {
                    baseVoltage = unitValue(baseVolt->nominalVoltage.value,UnitMultiplier::k);
                }
            }
        }
    }
    // as second option take baseVoltage of topologicalNode where line is connected to
    if(baseVoltage == 0){
        for (auto obj : mModel->Objects) {
			if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
                for (auto term : topNode->Terminal) {
					if (term->ConductingEquipment->name == line->name) {
                        baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value,UnitMultiplier::k);
					}
				}
			}
		}
    }

	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			Matrix res_3ph = singlePhaseParameterToThreePhase(resistance);
			Matrix ind_3ph = singlePhaseParameterToThreePhase(inductance);
			Matrix cap_3ph = singlePhaseParameterToThreePhase(capacitance);
			Matrix cond_3ph = singlePhaseParameterToThreePhase(conductance);

			auto cpsLine = std::make_shared<EMT::Ph3::PiLine>(line->mRID, line->name, mComponentLogLevel);
			cpsLine->setParameters(res_3ph, ind_3ph, cap_3ph, cond_3ph);
			return cpsLine;
		}
		else {
			mSLog->info("    PiLine for EMT not implemented yet");
			auto cpsLine = std::make_shared<DP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
			cpsLine->setParameters(resistance, inductance, capacitance, conductance);
			return cpsLine;
		}
	}
	else if (mDomain == Domain::SP) {
		auto cpsLine = std::make_shared<SP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
		cpsLine->setParameters(resistance, inductance, capacitance, conductance, mOmega);
		cpsLine->setBaseVoltage(baseVoltage);
		return cpsLine;
	}
	else {
		auto cpsLine = std::make_shared<DP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
		cpsLine->setParameters(resistance, inductance, capacitance, conductance);
		return cpsLine;
	}

}

TopologicalPowerComp::Ptr Reader::mapPowerTransformer(PowerTransformer* trans) {
	if (trans->PowerTransformerEnd.size() != 2) {
		mSLog->warn("PowerTransformer {} does not have exactly two windings, ignoring", trans->name);
		return nullptr;
	}
	mSLog->info("Found PowerTransformer {}", trans->name);

	// assign transformer ends
	PowerTransformerEnd* end1 = nullptr, *end2 = nullptr;
	for (auto end : trans->PowerTransformerEnd) {
		if (end->Terminal->sequenceNumber == 1) end1 = end;
		else if (end->Terminal->sequenceNumber == 2) end2 = end;
		else return nullptr;
	}

	// setting default values for non-set resistances and reactances
	mSLog->info("    PowerTransformerEnd_1 {}", end1->name);
    mSLog->info("    Srated={} Vrated={}", (float) end1->ratedS.value, (float) end1->ratedU.value);
	try{
		mSLog->info("       R={}", (float) end1->r.value);
	}catch(ReadingUninitializedField* e1){
		end1->r.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd1 setting default value of R={}", (float) end1->r.value);
	}
	try{
		mSLog->info("       X={}", (float) end1->x.value);
	}catch(ReadingUninitializedField* e1){
		end1->x.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd1 setting default value of X={}", (float) end1->x.value);
	}
    mSLog->info("    PowerTransformerEnd_2 {}", end2->name);
    mSLog->info("    Srated={} Vrated={}", (float) end2->ratedS.value, (float) end2->ratedU.value);
	try{
		mSLog->info("       R={}", (float) end2->r.value);
	}catch(ReadingUninitializedField* e1){
		end2->r.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd2 setting default value of R={}", (float) end2->r.value);
	}
	try{
		mSLog->info("       X={}", (float) end2->x.value);
	}catch(ReadingUninitializedField* e1){
		end2->x.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd2 setting default value of X={}", (float) end2->x.value);
	}

	if (end1->ratedS.value != end2->ratedS.value) {
		mSLog->warn("    PowerTransformerEnds of {} come with distinct rated power values. Using rated power of PowerTransformerEnd_1.", trans->name);
	}
	Real ratedPower = unitValue(end1->ratedS.value, UnitMultiplier::M);
	Real voltageNode1 = unitValue(end1->ratedU.value, UnitMultiplier::k);
	Real voltageNode2 = unitValue(end2->ratedU.value, UnitMultiplier::k);

    Real ratioAbsNominal = voltageNode1 / voltageNode2;
	Real ratioAbs = ratioAbsNominal;

	// use normalStep from RatioTapChanger
	if (end1->RatioTapChanger) {
		ratioAbs = voltageNode1 / voltageNode2 * (1 + (end1->RatioTapChanger->normalStep - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
	}

	// if corresponding SvTapStep available, use instead tap position from there
	if (end1->RatioTapChanger) {
		for (auto obj : mModel->Objects) {
			auto tapStep = dynamic_cast<IEC61970::Base::StateVariables::SvTapStep*>(obj);
			if (tapStep && tapStep->TapChanger == end1->RatioTapChanger) {
				ratioAbs = voltageNode1 / voltageNode2 * (1 + (tapStep->position - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
			}
		}
	}

	// TODO: To be extracted from cim class
	Real ratioPhase = 0;

    // Calculate resistance and inductance referred to high voltage side
	Real resistance = 0;
    Real inductance = 0;
	if (voltageNode1 >= voltageNode2 && abs(end1->x.value) > 1e-12) {
		inductance = end1->x.value / mOmega;
		resistance = end1->r.value;
	} else if (voltageNode1 >= voltageNode2 && abs(end2->x.value) > 1e-12) {
		inductance = end2->x.value / mOmega * std::pow(ratioAbsNominal, 2);
		resistance = end2->r.value * std::pow(ratioAbsNominal, 2);
	}
	else if (voltageNode2 > voltageNode1 && abs(end2->x.value) > 1e-12) {
		inductance = end2->x.value / mOmega;
		resistance = end2->r.value;
	}
	else if (voltageNode2 > voltageNode1 && abs(end1->x.value) > 1e-12) {
		inductance = end1->x.value / mOmega / std::pow(ratioAbsNominal, 2);
		resistance = end1->r.value / std::pow(ratioAbsNominal, 2);
	}

	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			Matrix resistance_3ph = singlePhaseParameterToThreePhase(resistance);
			Matrix inductance_3ph = singlePhaseParameterToThreePhase(inductance);
			Bool withResistiveLosses = resistance > 0;
			auto transformer = std::make_shared<EMT::Ph3::Transformer>(trans->mRID, trans->name, mComponentLogLevel, withResistiveLosses);
			transformer->setParameters(ratioAbs, ratioPhase, resistance_3ph, inductance_3ph);
			return transformer;
		}
		else
		{
			mSLog->info("    Transformer for EMT not implemented yet");
			return nullptr;
		}
	}
	else if (mDomain == Domain::SP) {
		auto transformer = std::make_shared<SP::Ph1::Transformer>(trans->mRID, trans->name, mComponentLogLevel);
		transformer->setParameters(voltageNode1, voltageNode2, ratedPower, ratioAbs, ratioPhase, resistance, inductance, mOmega);
		Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
		transformer->setBaseVoltage(baseVolt);
		return transformer;
	}
	else {
		Bool withResistiveLosses = true;
		Bool withSat = true;
		resistance = (resistance > 0) ? resistance : 1e-3;
		auto transformer = std::make_shared<DP::Ph1::Transformer>(trans->mRID, trans->name, mComponentLogLevel, withResistiveLosses, withSat);
		transformer->setParameters(ratioAbs, ratioPhase, resistance, inductance);

		Real NumTaps = 10;
		transformer->setOLTCParamters(NumTaps, voltageNode2*1.05);
		transformer->setOLTCTimeDelay(15);
		transformer->setOLTCDeadband(0.01);

		transformer->setParametersSaturationDefault(220000, 66000);
		//transformer->setMagnetizingInductance(350);
		//transformer->setSaturationCalculationMethod(true);
		return transformer;
	}
}

TopologicalPowerComp::Ptr Reader::mapSynchronousMachine(SynchronousMachine* machine) {
	mSLog->info("    Found  Synchronous machine {}", machine->name);

	if (mGeneratorType == GeneratorType::Transient) {
		Real directTransientReactance;
		Real inertiaCoefficient;
		Real ratedPower;
		Real ratedVoltage;

		for (auto obj : mModel->Objects) {
			// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
			if (IEC61970::Dynamics::StandardModels::SynchronousMachineDynamics::SynchronousMachineTimeConstantReactance* genDyn =
				dynamic_cast<IEC61970::Dynamics::StandardModels::SynchronousMachineDynamics::SynchronousMachineTimeConstantReactance*>(obj)) {
				if (genDyn->SynchronousMachine->mRID == machine->mRID) {
					directTransientReactance = genDyn->xDirectTrans.value;
					inertiaCoefficient = genDyn->inertia.value;

					ratedPower = unitValue(machine->ratedS.value, UnitMultiplier::M);

					ratedVoltage = unitValue(machine->ratedU.value, UnitMultiplier::k);
					auto gen = DP::Ph1::SynchronGeneratorTrStab::make(machine->mRID, machine->name, mComponentLogLevel);
					gen->setStandardParametersPU(ratedPower, ratedVoltage, mFrequency,
						directTransientReactance, inertiaCoefficient);
					return gen;
				}
			}
		}
	}

	if (mDomain == Domain::SP) {
		for (auto obj : mModel->Objects) {
			if (IEC61970::Base::Generation::Production::GeneratingUnit* genUnit
				= dynamic_cast<IEC61970::Base::Generation::Production::GeneratingUnit*>(obj))
			{

				for (auto syncGen : genUnit->RotatingMachine)
				{
                    if (syncGen->mRID == machine->mRID){

						// Check whether relevant input data are set, otherwise set default values
						Real setPointActivePower = 0;
						Real setPointVoltage = 0;
						Real maximumReactivePower = 1e12;
						try{
							setPointActivePower = unitValue(genUnit->initialP.value, UnitMultiplier::M);
							mSLog->info("    setPointActivePower={}", setPointActivePower);
						}catch(ReadingUninitializedField* e){
							std::cerr << "Uninitalized setPointActivePower for GeneratingUnit " << machine->name << ". Using default value of " << setPointActivePower << std::endl;
						}
						if (machine->RegulatingControl) {
							setPointVoltage = unitValue(machine->RegulatingControl->targetValue.value, UnitMultiplier::k);
							mSLog->info("    setPointVoltage={}", setPointVoltage);
						} else {
							std::cerr << "Uninitalized setPointVoltage for GeneratingUnit " <<  machine->name << ". Using default value of " << setPointVoltage << std::endl;
						}
						try{
							maximumReactivePower = unitValue(machine->maxQ.value, UnitMultiplier::M);
							mSLog->info("    maximumReactivePower={}", maximumReactivePower);
						}catch(ReadingUninitializedField* e){
							std::cerr << "Uninitalized maximumReactivePower for GeneratingUnit " <<  machine->name << ". Using default value of " << maximumReactivePower << std::endl;
						}

						auto gen = std::make_shared<SP::Ph1::SynchronGenerator>(machine->mRID, machine->name, mComponentLogLevel);
							gen->setParameters(unitValue(machine->ratedS.value, UnitMultiplier::M),
									unitValue(machine->ratedU.value, UnitMultiplier::k),
									setPointActivePower,
									setPointVoltage,
									maximumReactivePower,
									PowerflowBusType::PV);
							gen->setBaseVoltage(unitValue(machine->ratedU.value, UnitMultiplier::k));
						return gen;
					}
				}

			}

		}
		mSLog->info("no corresponding initial power for {}", machine->name);
		return std::make_shared<SP::Ph1::SynchronGenerator>(machine->mRID, machine->name, mComponentLogLevel);
	}
    else {
        return std::make_shared<DP::Ph1::SynchronGeneratorIdeal>(machine->mRID, machine->name, mComponentLogLevel);
    }
}

TopologicalPowerComp::Ptr Reader::mapExternalNetworkInjection(ExternalNetworkInjection* extnet) {
	mSLog->info("Found External Network Injection {}", extnet->name);
	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			return std::make_shared<EMT::Ph3::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
		}
		else {
			throw SystemError("Mapping of ExternalNetworkInjection for EMT::Ph1 not existent!");
			return nullptr;
		}
	} else if(mDomain == Domain::SP) {
		if (mPhase == PhaseType::Single) {
			auto cpsextnet = std::make_shared<SP::Ph1::externalGridInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
			cpsextnet->modifyPowerFlowBusType(PowerflowBusType::VD); // for powerflow solver set as VD component as default
			if(extnet->RegulatingControl){
				mSLog->info("       Voltage set-point={}", (float) extnet->RegulatingControl->targetValue);
				cpsextnet->setParameters(extnet->RegulatingControl->targetValue); // assumes that value is specified in CIM data in per unit
			} else
				mSLog->info("       No voltage set-point defined.");
			return cpsextnet;
		}
		else {
			throw SystemError("Mapping of ExternalNetworkInjection for SP::Ph3 not existent!");
			return nullptr;
		}
	} else {
		if (mPhase == PhaseType::Single) {
			//return std::make_shared<DP::Ph1::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
			Real baseVoltage = 0;
			// first look for baseVolt object to set baseVoltage
			for (auto obj : mModel->Objects) {
				if (IEC61970::Base::Core::BaseVoltage* baseVolt = dynamic_cast<IEC61970::Base::Core::BaseVoltage*>(obj)) {
					for (auto comp : baseVolt->ConductingEquipment) {
						if (comp->name == extnet->name) {
							baseVoltage = unitValue(baseVolt->nominalVoltage.value, UnitMultiplier::k);
						}
					}
				}
			}
			// as second option take baseVoltage of topologicalNode where ExternalNetworkInjection is connected to
			if (baseVoltage == 0) {
				for (auto obj : mModel->Objects) {
					if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
						for (auto term : topNode->Terminal) {
							if (term->ConductingEquipment->name == extnet->name) {
								baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value, UnitMultiplier::k);
							}
						}
					}
				}
			}

			// check if name contains "Slack" or "HSNetz", if so model as ideal voltage source
			if (extnet->name.find("Slack") != std::string::npos) {
				mSLog->info("NetworkInjection for DP single-phase modeled ideal Voltage Source (Slack-Node)");
				auto cpsext = std::make_shared<DP::Ph1::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
				return cpsext;
			}
			else if (extnet->name.find("HSNetz") != std::string::npos) {
				mSLog->info("NetworkInjection for DP single-phase modeled ideal Voltage Source (Slack-Node)");
				auto cpsext = std::make_shared<DP::Ph1::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
				return cpsext;
			}
			else
			{
				//return nullptr;
				mSLog->info("NetworkInjection for DP single-phase modeled as VSI in DQ-Frame");
				Bool has_trafo = true;
				Bool switchActive = true;
				auto ext_vsi = std::make_shared<DP::Ph1::AvVoltageSourceInverterDQ>(extnet->mRID, extnet->name, mComponentLogLevel, has_trafo, switchActive);

				// parameters for vsi
				Real omegeN = 2 * M_PI * mFrequency;
				Real Pref = unitValue(extnet->p.value, UnitMultiplier::M);
				//Real Qref = unitValue(extnet->q.value, UnitMultiplier::M);
				Real Sn = sqrt(pow(unitValue(extnet->maxP.value, UnitMultiplier::M), 2) + pow(unitValue(extnet->maxQ.value, UnitMultiplier::M),2));

				//Real Qref = unitValue(extnet->q.value, UnitMultiplier::M);
				Real Qref = 0;
				Real kp_pll = 0.25;
				Real ki_pll = 2;
				//Real Kp_powerCtrl = 0.001;
				//Real Ki_powerCtrl = 0.08;
				//Real Kp_currCtrl = 0.3;
				//Real Ki_currCtrl = 10;
				Real Kp_powerCtrl = 0.0005;
				Real Ki_powerCtrl = 0.01;
				Real Kp_currCtrl = 0.1;
				Real Ki_currCtrl = 2;
				Real Lf = 0.002;
				Real Cf = 7.89e-07;
				Real Rf = 0.01;
				Real Rc = 0.01;

				// parameters for connection transformer
				Real tr_nomVoltEnd1 = baseVoltage;
				Real tr_nomVoltEnd2 = 15e3;
				Real tr_ratedPower = 50e6;
				Real tr_ratioAbs = tr_nomVoltEnd1 / tr_nomVoltEnd2;
				Real tr_ratioPhase = 0;
				Real tr_resistance = 0.001;
				Real tr_inductance = 0.001;

				// set parameters for NetworkInjection that is modeled as VSI
				ext_vsi->setParameters(omegeN, tr_nomVoltEnd2, Pref, Qref, Sn);
				ext_vsi->setControllerParameters(kp_pll, ki_pll, Ki_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, omegeN);
				ext_vsi->setFilterParameters(Lf, Cf, Rf, Rc);
				ext_vsi->setTransformerParameters(tr_nomVoltEnd1, tr_nomVoltEnd2, tr_ratedPower, tr_ratioAbs, tr_ratioPhase, tr_resistance, tr_inductance, omegeN);
				ext_vsi->setInitialStateValues(0,0,Pref,Qref,0,0,0,0);

				// Q Control param
				Real Qmax = unitValue(extnet->maxQ.value, UnitMultiplier::M);
				Real Qmin = -Qmax;
				Real VRef = tr_nomVoltEnd1 * (1 + 0.05);
				Real Deadband = 0.01;
				Real SGain = 20;
				Real DGain = 3;
				ext_vsi->setQControlParameters(true, VRef, SGain, DGain, Deadband, Qmax, Qmin);

				return ext_vsi;
			}
		} else {
			throw SystemError("Mapping of ExternalNetworkInjection for DP::Ph3 not existent!");
			return nullptr;
		}
	}
}

TopologicalPowerComp::Ptr Reader::mapEquivalentShunt(EquivalentShunt* shunt){
	mSLog->info("Found EquivalentShunt {}", shunt->name);

	Real baseVoltage = 0;
    // first look for baseVolt object to set baseVoltage
    for (auto obj : mModel->Objects) {
        if (IEC61970::Base::Core::BaseVoltage* baseVolt = dynamic_cast<IEC61970::Base::Core::BaseVoltage*>(obj)) {
            for (auto comp : baseVolt->ConductingEquipment) {
                if (comp->name == shunt->name) {
                    baseVoltage = unitValue(baseVolt->nominalVoltage.value,UnitMultiplier::k);
                }
            }
        }
    }
    // as second option take baseVoltage of topologicalNode where shunt is connected to
    if(baseVoltage == 0){
        for (auto obj : mModel->Objects) {
			if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
                for (auto term : topNode->Terminal) {
					if (term->ConductingEquipment->name == shunt->name) {
                        baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value,UnitMultiplier::k);
					}
				}
			}
		}
    }

	auto cpsShunt = std::make_shared<SP::Ph1::Shunt>(shunt->mRID, shunt->name, mComponentLogLevel);
	cpsShunt->setParameters(shunt->g.value, shunt->b.value);
	cpsShunt->setBaseVoltage(baseVoltage);
	return cpsShunt;
}

TopologicalPowerComp::Ptr Reader::mapStaticVarCompensator(StaticVarCompensator* svc) {
	//mSLog->info("Found FACTS Element: {}", svc->name);

	Real baseVoltage = 0;
	// first look for baseVolt object to set baseVoltage
	for (auto obj : mModel->Objects) {
		if (IEC61970::Base::Core::BaseVoltage* baseVolt = dynamic_cast<IEC61970::Base::Core::BaseVoltage*>(obj)) {
			for (auto comp : baseVolt->ConductingEquipment) {
				if (comp->name == svc->name) {
					baseVoltage = unitValue(baseVolt->nominalVoltage.value, UnitMultiplier::k);
				}
			}
		}
	}
	// as second option take baseVoltage of topologicalNode where shunt is connected to
	if (baseVoltage == 0) {
		for (auto obj : mModel->Objects) {
			if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
				for (auto term : topNode->Terminal) {
					if (term->ConductingEquipment->name == svc->name) {
						baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value, UnitMultiplier::k);
					}
				}
			}
		}
	}

	if (mDomain == Domain::DP && mPhase == PhaseType::Single) {
		// evaluate maximum values for svc
		Real QCapMax = (baseVoltage * baseVoltage) / svc->capacitiveRating.value;
		Real QIndMax = (baseVoltage * baseVoltage) / svc->inductiveRating.value;
		// set maximum as reference value
		Real Qmax = Math::abs((QCapMax > QIndMax) ? QCapMax : QIndMax);

		if (svc->name.find("STATCOM") != std::string::npos)
		{
			// model as STATCOM
			//return nullptr;
			mSLog->info("Found FACTS Element: {} -> model as: STATCOM", svc->name);
			Bool has_trafo = true;
			Bool switchActive = false;
			auto statcom = std::make_shared<DP::Ph1::AvVoltageSourceInverterDQ>(svc->mRID, svc->name, mComponentLogLevel, has_trafo, switchActive);

			// parameters for vsi
			Real omegeN = 2 * M_PI * mFrequency;
			Real Pref = 0;
			Real Qref = unitValue(svc->q.value, UnitMultiplier::M);
			Real Sn = Qmax;
			
			Real ki_pll = 2;
			Real kp_pll = 0.25;
			//Real Kp_powerCtrl = 0.001;
			//Real Ki_powerCtrl = 0.08;
			//Real Kp_currCtrl = 0.3;
			//Real Ki_currCtrl = 10;
			Real Kp_powerCtrl = 0.0005;
			Real Ki_powerCtrl = 0.01;
			Real Kp_currCtrl = 0.1;
			Real Ki_currCtrl = 2;
			Real Lf = 0.002;
			Real Cf = 7.89e-06;
			Real Rf = 0.1;
			Real Rc = 0.1;

			// parameters for connection transformer
			Real tr_nomVoltEnd1 = baseVoltage;
			Real tr_nomVoltEnd2 = 15e3;
			Real tr_ratedPower  = 50e6;
			Real tr_ratioAbs = tr_nomVoltEnd1 / tr_nomVoltEnd2;
			Real tr_ratioPhase = 0;
			Real tr_resistance = 0.001;
			Real tr_inductance = 0.001;

			// set parameters for NetworkInjection that is modeled as VSI
			statcom->setParameters(omegeN, tr_nomVoltEnd2, Pref, Qref, Sn);
			statcom->setControllerParameters(kp_pll, ki_pll, Ki_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, omegeN);
			statcom->setFilterParameters(Lf, Cf, Rf, Rc);
			statcom->setTransformerParameters(tr_nomVoltEnd1, tr_nomVoltEnd2, tr_ratedPower, tr_ratioAbs, tr_ratioPhase, tr_resistance, tr_inductance, omegeN);
			statcom->setInitialStateValues(0, 0, Pref, Qref, 0, 0, 0, 0);

			// Q Control param
			Real VRef = unitValue(svc->voltageSetPoint.value, UnitMultiplier::k);
			Real Deadband = 0.01;
			Real SGain = 20;
			Real DGain = 3;
			statcom->setQControlParameters(true, VRef, SGain, DGain, Deadband, -QIndMax, -QCapMax);

			return statcom;

		}
		else
		{
			// model as SVC
			mSLog->info("Found FACTS Element: {} -> model as: SVC", svc->name);
			auto cpsSVC = std::make_shared<DP::Ph1::SVC>(svc->mRID, svc->name, mComponentLogLevel);
			//Real Sn = 5e6;

			// Bmax corresponds to inductive behaviour (voltage decrease)
			Real Bmax = -1 * (QIndMax / Qmax);
			// Bmin corresponds to capacitive behaviour (voltage increase)
			Real Bmin = -1 * (QCapMax / Qmax);

			//
			Real RefVoltage = unitValue(svc->voltageSetPoint.value, UnitMultiplier::k);

			cpsSVC->setParameters(Bmax, Bmin, Qmax, baseVoltage, RefVoltage);
			// vorher K = 40
			cpsSVC->setControllerParameters(0.2, 40);
			return cpsSVC;
		}
	}
}


TopologicalPowerComp::Ptr Reader::mapLinearShuntCompensator(LinearShuntCompensator* shunt) {
	mSLog->info("Found LinearShuntCompensator {}", shunt->name);

	Real baseVoltage = 0;
	// first look for baseVolt object to set baseVoltage
	for (auto obj : mModel->Objects) {
		if (IEC61970::Base::Core::BaseVoltage* baseVolt = dynamic_cast<IEC61970::Base::Core::BaseVoltage*>(obj)) {
			for (auto comp : baseVolt->ConductingEquipment) {
				if (comp->name == shunt->name) {
					baseVoltage = unitValue(baseVolt->nominalVoltage.value, UnitMultiplier::k);
				}
			}
		}
	}
	// as second option take baseVoltage of topologicalNode where shunt is connected to
	if (baseVoltage == 0) {
		for (auto obj : mModel->Objects) {
			if (IEC61970::Base::Topology::TopologicalNode* topNode = dynamic_cast<IEC61970::Base::Topology::TopologicalNode*>(obj)) {
				for (auto term : topNode->Terminal) {
					if (term->ConductingEquipment->name == shunt->name) {
						baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value, UnitMultiplier::k);
					}
				}
			}
		}
	}

	if (mDomain == Domain::DP && mPhase == PhaseType::Single) {
		// model as SVC
		auto cpsShunt = std::make_shared<DP::Ph1::SVC>(shunt->mRID, shunt->name, mComponentLogLevel);
		Real Sn = 0.5e6;
		Real Bmax = 1;
		Real Bmin = -1;
		cpsShunt->setParameters(Bmax, Bmin, Sn, baseVoltage);
		cpsShunt->setControllerParameters(0.2, 40);
		return cpsShunt;
	}
}


void Reader::readNodeVoltagesFromStaticSystemTopology(SystemTopology& sysStatic, SystemTopology& sysDynamic) {
	auto nodeDyn = sysDynamic.mNodes.begin();
	auto nodeSt = sysStatic.mNodes.begin();

	for (; nodeDyn != sysDynamic.mNodes.end() && nodeSt != sysStatic.mNodes.end(); ++nodeDyn, ++nodeSt) {
		if ((*nodeDyn)->name() == (*nodeSt)->name()) {
			mSLog->info("Updating SV info according to powerflow (1st loop):");
			mSLog->info("Before: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
			(*nodeDyn)->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeSt))->singleVoltage());
			mSLog->info("Now: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
		}
		else {
			for (auto node : sysStatic.mNodes) {
				if ((*nodeDyn)->name() == node->name())
				{
					mSLog->info("Updating SV info according to powerflow (2nd loop):");
					mSLog->info("Before: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
					(*nodeDyn)->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->singleVoltage());
					mSLog->info("Now: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
				}					
			}
		}
	}
	// in case the sysDynamic has more nodes than sysStatic
	if (nodeDyn != sysDynamic.mNodes.end()) {
		for (; nodeDyn != sysDynamic.mNodes.end(); ++nodeDyn) {
			for (auto node : sysStatic.mNodes) {
				if ((*nodeDyn)->name() == node->name())
				{
					mSLog->info("Updating SV info according to powerflow (3rd loop):");
					mSLog->info("Before: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
					(*nodeDyn)->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->singleVoltage());
					mSLog->info("Now: Voltage at node {} : {}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->name(), std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>((*nodeDyn))->initialSingleVoltage());
				}					
			}
		}
	}
}

void Reader::initDynamicSystemTopologyWithPowerflow(SystemTopology& systemPF, SystemTopology& system) {
	for (auto nodePF : systemPF.mNodes) {
		if (auto node = system.node<TopologicalNode>(nodePF->name())) {
			mSLog->info("Updating initial voltage of {} according to powerflow", node->name());
			mSLog->info("Former initial voltage: {}", node->initialSingleVoltage());
			node->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(nodePF)->singleVoltage());
			mSLog->info("Updated initial voltage: {}", node->initialSingleVoltage());
		}
	}
}


template<typename VarType>
void Reader::processTopologicalNode(IEC61970::Base::Topology::TopologicalNode* topNode) {
	// Add this node to global node list and assign simulation node incrementally.
	int matrixNodeIndex = Int(mPowerflowNodes.size());
	mPowerflowNodes[topNode->mRID] = SimNode<VarType>::make(topNode->mRID, topNode->name, matrixNodeIndex, mPhase);

	if (mPhase == PhaseType::ABC) {
		mSLog->info("TopologicalNode {} phase A as simulation node {} ", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::A));
		mSLog->info("TopologicalNode {} phase B as simulation node {}", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::B));
		mSLog->info("TopologicalNode {} phase C as simulation node {}", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::C));
	}
	else
		mSLog->info("TopologicalNode {} as simulation node {}", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex());

	for (auto term : topNode->Terminal) {
		// Insert Terminal if it does not exist in the map and add reference to node.
		// This could be optimized because the Terminal is searched twice.
		auto cpsTerm = SimTerminal<VarType>::make(term->mRID);
		mPowerflowTerminals.insert(std::make_pair(term->mRID, cpsTerm));
		cpsTerm->setNode(std::dynamic_pointer_cast<SimNode<VarType>>(mPowerflowNodes[topNode->mRID]));

		if (!term->sequenceNumber.initialized)
			term->sequenceNumber = 1;

		mSLog->info("    Terminal {}, sequenceNumber {}", term->mRID, (int) term->sequenceNumber);

		// Try to process Equipment connected to Terminal.
		IEC61970::Base::Core::ConductingEquipment *equipment = term->ConductingEquipment;
		if (!equipment) {
			mSLog->warn("Terminal {} has no Equipment, ignoring!", term->mRID);
		}
		else {
			// Insert Equipment if it does not exist in the map and add reference to Terminal.
			// This could be optimized because the Equipment is searched twice.
			if (mPowerflowEquipment.find(equipment->mRID) == mPowerflowEquipment.end()) {
				TopologicalPowerComp::Ptr comp = mapComponent(equipment);
				if (comp) {
					mPowerflowEquipment.insert(std::make_pair(equipment->mRID, comp));
				} else {
					mSLog->warn("Could not map equipment {}", equipment->mRID);
					continue;
				}
			}

			auto pfEquipment = mPowerflowEquipment.at(equipment->mRID);
			std::dynamic_pointer_cast<SimPowerComp<VarType>>(pfEquipment)->setTerminalAt(
				std::dynamic_pointer_cast<SimTerminal<VarType>>(mPowerflowTerminals[term->mRID]), term->sequenceNumber-1);

			mSLog->info("        Added Terminal {} to Equipment {}", term->mRID, equipment->mRID);
		}
	}
}

template void Reader::processTopologicalNode<Real>(IEC61970::Base::Topology::TopologicalNode* topNode);
template void Reader::processTopologicalNode<Complex>(IEC61970::Base::Topology::TopologicalNode* topNode);
