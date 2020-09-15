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
#ifndef NC_UNDERBOUND_H
#define NC_UNDERBOUND_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/nc_scheduling.h"
#include "NetworkCalculus/nccurves.h"

// implementation of an optimistic NC approach for FIFO scheduling and SPQ scheduling
//
// the worst-case delay under bound computation in this class
//      is based on the Network Calculus approach published in https://ieeexplore.ieee.org/abstract/document/7993380
// a worst-case delay under bound is the delay less than (yet close to) the exact worst-case delay.
// it is computed by introducing some "optimistic" assumptions in NC approach in the computation of arrival and service curves.
// the difference between
//      the delay upper bound computed by NC_FIFO or NC_SPQ
//      the delay under bound computed by NC_UNDERBOUND and
// gives the upper bound of pessimism in NC_FIFO/NC_SPQ approach.
class NC_UNDERBOUND : public NC_Scheduling
{
public:
    NC_UNDERBOUND(SCHEDULING const &sPolicy);

    // compute under bound on worst-case end-to-end delay for all flows on all paths in the network
    void computeE2E(NCData *dataPtr);
    // compute under bound on worst-case delay for refFlow at the given node
    double computeNodeDelay(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the optimistic overall arrival curve for refFlow at the given node
    ConcaveCurve computeOverallArrivalCurve(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the jitter upper bound for refFlow at the given node
    double computeJitter(NCData::NCFlow const&refFlow, NODE_t const&node);
    // compute the optimistic service curve for refFlow at the given node
    ConvexCurve computeServiceCurve(NCData::NCFlow const&refFlow, NODE_t const&node);

private:
    SCHEDULING policy; // scheduling policy
};

#endif // NC_UNDERBOUND_H
