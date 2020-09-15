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

#ifndef NC_FIFO_H
#define NC_FIFO_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/nc_scheduling.h"
#include "NetworkCalculus/nccurves.h"

// implementation of FIFO scheduling
//
// the worst-case delay computation in this class
//      is based on the Network Calculus approach published in https://ieeexplore.ieee.org/document/669170
//      with added enhancements from https://ieeexplore-ieee-org.gorgone.univ-toulouse.fr/document/5524098 and https://hal.archives-ouvertes.fr/hal-02270458
// the pessimism involved in this approach can be upper bounded by coparing the results to those of NC_UNDERBOUND
class NC_FIFO : public NC_Scheduling
{
public:
    NC_FIFO();

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
};

#endif // NC_FIFO_H
