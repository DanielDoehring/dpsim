/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Definitions.h>
#include <cps/SystemTopology.h>
#include <cps/Components.h>

#pragma once

namespace CPS {
namespace CIM {
namespace Examples {

namespace SGIB {

    struct ScenarioConfig {
        Real systemFrequency = 50;
        Real systemNominalVoltage = 20e3;

        // Line parameters (R/X = 1)
        Real length = 5;
        Real lineResistance = 0.5 * length;
	    Real lineInductance = 0.5/314 * length;
        Real lineCapacitance = 50e-6/314 * length;

        // PV controller parameters
        Real scaling_P = 1;
        Real scaling_I = 0.1;

        Real KpPLL = 0.25*scaling_P;
        Real KiPLL = 2*scaling_I;
        Real KpPowerCtrl = 0.001*scaling_P;
        Real KiPowerCtrl = 0.08*scaling_I;
        Real KpCurrCtrl = 0.3*scaling_P;
        Real KiCurrCtrl = 10*scaling_I;
        Real OmegaCutoff = 2 * PI * systemFrequency;

        // Initial state values
        Real thetaPLLInit = 0; // only for debug
        Real phiPLLInit = 0; // only for debug
        Real phi_dInit = 0;
        Real phi_qInit = 0;
        Real gamma_dInit = 0;
        Real gamma_qInit = 0;

        // Nominal generated power values of PV
        Real pvNominalVoltage = 1500.;
        Real pvNominalActivePower = 100e3;
        Real pvNominalReactivePower = 50e3;

        // PV filter parameters
        Real Lf = 0.002;
        Real Cf = 789.3e-6;
        Real Rf = 0.1;
        Real Rc = 0.1;

        // PV connection transformer parameters
        Real transformerNominalPower = 5e6;
        Real transformerInductance = 0.928e-3;

        // Further parameters
        Real systemOmega = 2 * PI * systemFrequency;
    };
}

namespace CIGREMV {

    struct ScenarioConfig {
        Real systemFrequency = 50;
        Real systemNominalVoltage = 20e3;
        Real penetrationLevel = 1;
        Real totalLoad = 4319.1e3; // calculated total load in CIGRE MV left feeder (see CIM data)

        // parameters of one PV unit
        Real pvUnitNominalVoltage = 1500.;
        Real pvUnitNominalPower = 50e3;
        Real pvUnitPowerFactor = 1;
        
        // calculate PV units per plant to reach penetration level
        Int numberPVUnits = Int(totalLoad * penetrationLevel / pvUnitNominalPower);
        Int numberPVPlants = 9;
        Int numberPVUnitsPerPlant = numberPVUnits / numberPVPlants;
                
        // PV controller parameters
        Real scaling_P = 10.0;
        Real scaling_I = 1000.0;

        Real KpPLL = 0.25/scaling_P;
        Real KiPLL = 2/scaling_I;
        Real KpPowerCtrl = 0.001/scaling_P;
        Real KiPowerCtrl = 0.08/scaling_I;
        Real KpCurrCtrl = 0.3/scaling_P;
        Real KiCurrCtrl = 10/scaling_I;
        Real OmegaCutoff = 2 * PI * systemFrequency;

        // PV filter parameters
        Real Lf = 0.002;
        Real Cf = 789.3e-6;
        Real Rf = 0.1;
        Real Rc = 0.1;

        // PV connection transformer parameters
        Real transformerNominalPower = 5e6;
        Real transformerInductance = 0.928e-3;

        // Further parameters
        Real systemOmega = 2 * PI * systemFrequency;

        // Initial state values (use known values with scaled control params)
        Real thetaPLLInit = 314.168313-systemOmega;
        Real phiPLLInit = 8e-06;
        Real pInit = 450000.716605;
        Real qInit = -0.577218;
        Real phi_dInit = 3854.197405*scaling_I;
        Real phi_qInit = -0.003737*scaling_I;
        Real gamma_dInit = 128.892668*scaling_I;
        Real gamma_qInit = 23.068682*scaling_I;
    };

    void addInvertersToCIGREMV(SystemTopology& system, CIGREMV::ScenarioConfig scenario, Domain domain) {
        Real pvActivePower = scenario.pvUnitNominalPower*scenario.numberPVUnitsPerPlant;
        Real pvReactivePower = sqrt(std::pow(pvActivePower / scenario.pvUnitPowerFactor, 2) - std::pow(pvActivePower, 2));

        // add PVs to network topology
        for (Int n = 3; n <= 11; n++) {	
            // TODO: cast to BaseAverageVoltageSourceInverter and move set functions out of case distinction
            if (domain == Domain::SP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = SP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::DP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::EMT) {
                SimNode<Real>::Ptr connectionNode = system.node<CPS::SimNode<Real>>("N" + std::to_string(n));
                auto pv = EMT::Ph3::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Real>(pv, { connectionNode });
            }
        }
    }

    void logPVAttributes(DPsim::DataLogger::Ptr logger, CPS::TopologicalPowerComp::Ptr pv) {

        // power controller
        std::vector<String> inputNames = {  pv->name() + "_powerctrl_input_pref", pv->name() + "_powerctrl_input_qref",
                                            pv->name() + "_powerctrl_input_vcd", pv->name() + "_powerctrl_input_vcq",
                                            pv->name() + "_powerctrl_input_ircd", pv->name() + "_powerctrl_input_ircq"};
        logger->addAttribute(inputNames, pv->attribute("powerctrl_inputs"));
        std::vector<String> stateNames = {  pv->name() + "_powerctrl_state_p", pv->name() + "_powerctrl_state_q",
                                            pv->name() + "_powerctrl_state_phid", pv->name() + "_powerctrl_state_phiq",
                                            pv->name() + "_powerctrl_state_gammad", pv->name() + "_powerctrl_state_gammaq"};
        logger->addAttribute(stateNames, pv->attribute("powerctrl_states"));
        std::vector<String> outputNames = {  pv->name() + "_powerctrl_output_vsd", pv->name() + "_powerctrl_output_vsq"};
        logger->addAttribute(outputNames, pv->attribute("powerctrl_outputs"));

        // interface variables
        logger->addAttribute(pv->name() + "_v_intf", pv->attribute("v_intf"));
        logger->addAttribute(pv->name() + "_i_intf", pv->attribute("i_intf"));

        // additional variables
        logger->addAttribute(pv->name() + "_pll_output", pv->attribute("pll_output"));
        logger->addAttribute(pv->name() + "_vsref", pv->attribute("Vsref"));
        logger->addAttribute(pv->name() + "_vs", pv->attribute("Vs"));
    }	

}
        std::shared_ptr<DPsim::SwitchEvent> createEventAddPowerConsumption(String nodeName, Real eventTime, Real additionalActivePower, SystemTopology& system, Domain domain, DPsim::DataLogger::Ptr logger) {
        
        // TODO: use base classes ph1
        if (domain == CPS::Domain::DP) {
            auto loadSwitch = DP::Ph1::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(1e9, resistance);
            loadSwitch->open();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Complex>(loadSwitch, { CPS::SimNode<Complex>::GND, connectionNode});
            logger->addAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
        } else if (domain == CPS::Domain::SP) {
            auto loadSwitch = SP::Ph1::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(1e9, resistance);
            loadSwitch->open();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Complex>(loadSwitch, { CPS::SimNode<Complex>::GND, connectionNode});
            logger->addAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }

    std::shared_ptr<DPsim::SwitchEvent3Ph> createEventAddPowerConsumption3Ph(String nodeName, Real eventTime, Real additionalActivePower, SystemTopology& system, Domain domain, DPsim::DataLogger::Ptr logger) {
        
        // TODO: use base classes ph3
         if (domain == CPS::Domain::EMT) {
            auto loadSwitch = EMT::Ph3::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Real>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(Matrix::Identity(3,3)*1e9, Matrix::Identity(3,3)*resistance);
            loadSwitch->openSwitch();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Real>(loadSwitch, { CPS::SimNode<Real>::GND, system.node<CPS::SimNode<Real>>(nodeName) });
            logger->addAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent3Ph::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }
    
}
}
}
