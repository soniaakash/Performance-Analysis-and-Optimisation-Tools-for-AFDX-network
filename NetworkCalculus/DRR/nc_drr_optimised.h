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

#ifndef NC_DRR_OPTIMISED_H
#define NC_DRR_OPTIMISED_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/DRR/nc_drr.h"

// implementation of DRR scheduling
//
// the worst-case delay computation in this class
//      is based on the Network Calculus approach published in https://ieeexplore.ieee.org/document/8603222 (Optimizing Network Calculus for DRR)
//      with an assumption of equal quantum distribution at each switch output port.
// this approach takes into account the amount of traffic from each class flows
// and optimise the delay computation by
//      removing the over-estimated traffic considered by NC_DRR_Classic
//      removing the pessimism induced by latency-rate curve
class NC_DRR_Optimised : public NC_DRR
{
public:
    NC_DRR_Optimised();

    // compute worst-case end-to-end delay for all flows on all paths in the network
    void computeE2E(NCData *dataPtr);
    // compute worst-case delay for refFlow at the given node
    double computeNodeDelay(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the overall arrival curve for refFlow at the given node
    ConcaveCurve computeOverallArrivalCurve(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the jitter upper bound for refFlow at the given node
    double computeJitter(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the service curve for refFlow at the given node
    ConvexCurve computeServiceCurve(NCData::NCFlow const&refFlow, NODE_t const&node);
    // optimise the computed delay for refFlow at the given node
    double computeOptimisedNodeDelay(NCData::NCFlow const&refFlow, NODE_t const&node);

};

#endif // NC_DRR_OPTIMISED_H
