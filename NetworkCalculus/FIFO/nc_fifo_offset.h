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
#ifndef NC_FIFO_OFFSET_H
#define NC_FIFO_OFFSET_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/nc_scheduling.h"
#include "NetworkCalculus/nccurves.h"

// implementation of FIFO scheduling
//     with flow scheduling (offset) at end systems
//
// the worst-case delay computation in this class
//      is based on the Network Calculus approach published in (cite:: ...)
class NC_FIFO_OFFSET : public NC_Scheduling
{
public:
    NC_FIFO_OFFSET();

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

    // compute max of aggregated arrival curves
    ConcaveCurve maxAggrCurve(const QList<ConcaveCurve> &aggrCurves);

    // compute the relative offsets with respect to the "benchmarkFlow" at the given "node"
    QMap<int, double> computeRelatveOffsets(NCData::NCFlow const &benchmarkFlow, NODE_t const &node);

private:    
    // compute the relative offsets with respect to the "benchmarkFlow" at the "sourceNode"
    QMap<int, double> computeRelatveOffsets_atSource(NCData::NCFlow const &benchmarkFlow, NODE_t const &sourceNode);

    // compute the definite offsets at the "node"
    QMap<int, double> computeDefiniteOffset(NODE_t const &node);

    double computeGranularity(QList<double> BAGs);

    struct ComputedOffsets {
        typedef QMap<int, double> OFFSET_MAP;
        QMap<int, OFFSET_MAP> fID_relativeOffsets; // <benchmarkFlow, relative offset map>
        OFFSET_MAP fID_definiteOffsets; // <flowID, definite offset>
    };
    QMap<NODE_t, ComputedOffsets> node_computedOffsets;

    struct ComputedAggregatedCurves {
        QMap<int, ConcaveCurve> fID_aggregatedCurve; // <benchmarkFlow, aggregated arrival curve>
    };
    QMap<NODE_t, ComputedAggregatedCurves> node_computedAggregatedCurves;


    // compute sum of aggregated arrival curves
    //     NOTE: addition of two aggregated arrival curves may result in a non-concave curve
    //     this function ensures concavity of the addition result
//    ConcaveCurve sum(QList<ConcaveCurve> const& aggregatedCurves) {

//        if (aggregatedCurves.empty()) throw std::runtime_error("empty list of aggregatedCurves. unable to make sum!");

//        // compute sum of arrival curves
//        //      NOTE: concavity validation is ignored since they may not exist in an aggregated arrival curve
//        RayList res = aggregatedCurves.begin()->getRayList();
//        for(int i = 1; i < aggregatedCurves.size(); ++i) {
//            res = addRayList(res, (aggregatedCurves.begin()+i)->getRayList()); // make sum
//            makeContinuous(res); // make sum
//        }

//        makeConcave(res);

//        ConcaveCurve  resConc; resConc.setRayList(res);
//        return resConc;
//    }

};

#endif // NC_FIFO_OFFSET_H
