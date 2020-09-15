/***************************************************************************
 * Performance Analysis and Optimisation Tools for AFDX network
 * Copyright (C) 2020  Aakash SONI
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "NetworkCalculus/networkcalculus.h"

NetworkCalculus::NetworkCalculus()
{
    configReady = false;
}

// initialize afdx network configuration and start E2E delay computation
void NetworkCalculus::init(const AFDX::AFDX_SOURCE &afdxConfigSource, const SCHEDULING &sPolicy, const MODE mode, const OPTIONS options) {

    qStdOut() << " mode : " << enumToString(mode) << "\n scheduling policy : " << enumToString(sPolicy) << "\n options : " << enumToString(options);

    if(afdxConfig.initConfig(afdxConfigSource) // load AFDX configuration from  afdxConfigSource (STATIC, FILE)
            && ncdata.initData(afdxConfig)){ //      load Network Calculus elements

        configReady = true;
        // select scheduler
        schedulingPolicy = sPolicy;
        // start computation
        simulate(mode, options);
    }
}

// start computation (called by init())
void NetworkCalculus::simulate(const MODE &mode, const OPTIONS &option)
{
    if(!configReady) { qStdOut() << "no afdx config loaded!"; return;}

    try { // error detection

        if(mode == MODE::WCTT) {
            // ***************** **************** ***************** //
            // **************** WCTT Analysis mode **************** //
            // ***************** **************** ***************** //


            // the E2E delay computation is divided into multiple steps
            //      these steps are same for each scheduling policy
            //      the only difference is in the computation within each step.
            //
            // Step 1 : select a flow (Vref)
            // Step 2 : select a path (Pi_Vref)
            // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
            //  Step 3a: compute overall arrival curve (alpha_o)
            //  Step 3b: compute service curve (beta_Vref)
            //  Step 3c: compute horizontal distance alpha_o and beta_Vref
            // Step 4 : compute e2e delay.

            // launch e2edelay computation based on the selected schedulingPolicy.
            if(schedulingPolicy == SCHEDULING::FIFO)
            {
                if(option == OPTIONS::NONE)
                {
                    NC_FIFO nc_fifo; nc_fifo.computeE2E(&ncdata);
                }
                else if(option == OPTIONS::OFFSETS)
                {
                    NC_FIFO_OFFSET nc_fifo_offset; nc_fifo_offset.computeE2E(&ncdata);
                }
                else if(option == OPTIONS::UNDER_BOUND)
                {
                    NC_UNDERBOUND nc_underbound(schedulingPolicy); nc_underbound.computeE2E(&ncdata);
                }
                else {
                    throw std::runtime_error("Invalid FIFO_OPTION. Select :: NONE (or) UNDER_BOUND");
                }
            }
            else if(schedulingPolicy == SCHEDULING::SPQ)
            {
                if(option == OPTIONS::NONE)
                {
                    NC_SPQ nc_spq; nc_spq.computeE2E(&ncdata);
                }
                else if(option == OPTIONS::UNDER_BOUND)
                {
                    NC_UNDERBOUND nc_underbound(schedulingPolicy); nc_underbound.computeE2E(&ncdata);
                }
                else {
                    throw std::runtime_error("Invalid SPQ_OPTION. Select :: NONE (or) UNDER_BOUND");
                }

            }
            else if(schedulingPolicy == SCHEDULING::DRR)
            {
                if(option == OPTIONS::CLASSIC)
                {
                    NC_DRR_Classic ncClassic; ncClassic.computeE2E(&ncdata);
                }
                else if(option == OPTIONS::OPTIMISED)
                {
                    NC_DRR_Optimised ncOptimised; ncOptimised.computeE2E(&ncdata);
                }
                else {
                    throw std::runtime_error("Invalid DRR_OPTION. Select :: CLASSIC (or) OPTIMISED");
                }
            }
            else if(schedulingPolicy == SCHEDULING::WRR)
            {
                if(option == OPTIONS::OPTIMISED)
                {
                    NC_WRR_Optimised ncOptimised; ncOptimised.computeE2E(&ncdata);
                }
                else {
                    throw std::runtime_error("Invalid WRR_OPTION. Select :: OPTIMISED");
                }
            }
        }
        else if (mode == MODE::QOS_TUNING)
        {
            // ***************** **************** ***************** //
            // *****************  Optimisation mode   ***************** //
            // ***************** **************** ***************** //
            if(schedulingPolicy == SCHEDULING::DRR)
            {
                if(option == OPTIONS::CLASSIC)
                {
                    NC_DRR_QuantumAssignment ncQAssignment; ncQAssignment.initComputation(&ncdata);
                }
                else if(option == OPTIONS::OPTIMISED)
                {
                    NC_DRR_QuantumAssignmentImproved ncQAssignmentImproved; ncQAssignmentImproved.initComputation(&ncdata);
                }
                else {
                    throw std::runtime_error("Invalid DRR tuning option. Select :: CLASSIC (or) OPTIMISED");
                }
            }
            else
            {
                throw std::runtime_error("Invalid scheduling policy. Select :: DRR");
            }
        }
        else {
            throw std::runtime_error("Invalid MODE. Select :: WCTT (or) QOS_TUNING");
        }

    } catch (const std::runtime_error& runtimeError) {  // error handle
        QString error; error.append(runtimeError.what());
        DISPLAY_OUTPUT = true;
        qStdOut() << error;
    }

}
