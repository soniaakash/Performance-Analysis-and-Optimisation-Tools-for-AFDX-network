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

#ifndef NETWORKCALCULUS_H
#define NETWORKCALCULUS_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/FIFO/nc_fifo.h"
#include "NetworkCalculus/FIFO/nc_fifo_offset.h"
#include "NetworkCalculus/SPQ/nc_spq.h"
#include "NetworkCalculus/DRR/nc_drr_classic.h"
#include "NetworkCalculus/DRR/nc_drr_optimised.h"
#include "NetworkCalculus/UnderBound/nc_underbound.h"
#include "NetworkCalculus/DRR/nc_drr_quantumassignment.h"
#include "NetworkCalculus/DRR/nc_drr_quantumassignmentimproved.h"
#include "NetworkCalculus/WRR/nc_wrr_optimised.h"

// this class is an implementation of the
// Network Calculus Theory (https://ieeexplore.ieee.org/document/669170)
// for worst-case delay analysis in real-time switched ethernet network.
// in particular, it is adapted for an aircraft data network called
// Avionic Full Duplex (AFDX) network (https://www.researchgate.net/publication/282108687_The_Evolution_of_Avionics_Networks_From_ARINC_429_to_AFDX).
//
// follwoing network features are currently implemented:
//     + scheduling Policy :
//         (1) FIFO
//              - (French) http://www.theses.fr/2004INPT041H
//              - https://www.researchgate.net/publication/4247669_Methods_for_bounding_end-to-end_delays_on_an_AFDX_network
//              - https://oatao.univ-toulouse.fr/2159/ (Using network calculus to optimize the AFDX network)
//              - https://ieeexplore.ieee.org/document/669170 (Application of network calculus to guaranteed service networks)
//              - https://hal.inria.fr/inria-00431674 (Dos and donts of service curve)
//              - https://ieeexplore.ieee.org/abstract/document/7993380 (pessimism analysis of network calculus)
//              - cite :: offsets
//         (2) SPQ
//              - https://hal.inria.fr/inria-00431674 (Dos and donts of service curve)
//              - https://ieeexplore.ieee.org/abstract/document/7993380 (pessimism analysis of network calculus)
//         (3) DRR
//              - https://ieeexplore.ieee.org/document/6376315 (DRR with network calculus)
//              - https://ieeexplore.ieee.org/document/1043123 (On the latency bound of DRR)
//              - https://ieeexplore.ieee.org/document/8603222 (Optimizing Network Calculus for DRR)
//              - https://ieeexplore.ieee.org/abstract/document/8756011 (Improved Delay Bound for a Service Curve Element)
//              - https://ieeexplore.ieee.org/abstract/document/8502523 (Integrating Offset in worst-case delay analysis for DRR)
//              - https://dl.acm.org/doi/10.1145/3356401.3356421 (Quantum assignment for QoS-aware AFDX network with deficit round robin)
//         (4) WRR
//              - https://dl.acm.org/doi/abs/10.1145/3273905.3273925 (WCTT analysis with WRR)
//
// following netowrk analysis features are implemented:
//     + worst-case delay analysis based on Network Calculus appraoch
//     + pessimism analysis of NC approach (FIFO and SPQ only)
//
// following network optimisation features are implemented:
//     + QoS tuning by credit optimisation (DRR only)
//
// following network analysis features are being implemented:
//     + exact worst-case delay analysis based on Model Checking appraoch

class NetworkCalculus
{
public:
    NetworkCalculus();

    // initialize afdx network configuration and start E2E delay computation
    //      load AFDX configuration from  afdxConfigSource (STATIC, FILE)
    //      load Network Calculus elements
    //      select scheduling policy sPolicy (FIFO, SPQ, DRR, ...)
    void init(AFDX::AFDX_SOURCE const & afdxConfigSource, SCHEDULING const &sPolicy, MODE const mode = MODE::WCTT, OPTIONS const options = OPTIONS::NONE);

    // start computation (called by init())
    void simulate(MODE const &mode, OPTIONS const &option);

private:
    SCHEDULING schedulingPolicy;
    AFDX afdxConfig;  // data extracted from afdx config source
    NCData ncdata;    // afdx config data (expanded) for NC computations
    bool configReady; // initialisation status flag
};

#endif // NETWORKCALCULUS_H
