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

#ifndef NC_DRR_QUANTUMASSIGNMENTIMPROVED_H
#define NC_DRR_QUANTUMASSIGNMENTIMPROVED_H

#include "NetworkCalculus/DRR/nc_drr_quantumassignment.h"

// implementation of DRR Quanum Assignment algorithm (improved by taking into account the optimised NC approach)
//      based on (cite::... INDIN_2020) (Efficient configuration of a QoS-aware AFDX network with Deficit Round Robin)
class NC_DRR_QuantumAssignmentImproved : protected NC_DRR
{
public:
    NC_DRR_QuantumAssignmentImproved();

    // initialise data and start computation
    void initComputation(NCData const *dataPtr);

    // compute the optimal quantum and its distribution
    QMap<int,int> computeOptimalQuantumAndItsDistribution(int const &initialQuantumValue, NCData const *dataPtr);

    // improve the given quantum distribution using NC_DRR_Optimised
    QMap<int,int> improveQuantumDistrbution(QMap<int,int> const &initialQunatumDistribution, NCData const *dataPtr);

    int reduceQuantumToMinimum(int const&classID, QMap<int,int> const &refDistribution, NCData const *dataPtr);

    QMap<int,int> maximiseCnService(QMap<int,int> const &refDistribution, NCData const *dataPtr);

    int increaseQuantumToMaximum(int const&classID, QMap<int,int> const &refDistribution, NCData const *dataPtr, QList<double> &res_deviationList);
    int increaseQuantumToMaximum(int const&classID, QMap<int,int> const &refDistribution, NCData const *dataPtr) { QList<double> optional; return increaseQuantumToMaximum(classID, refDistribution, dataPtr, optional);}

    bool incrementSmallestDifference(QMap<int,int> &refDistribution, NCData const *dataPtr);

    // implementation of a pure virtual function from parent NC_DRR
    void computeE2E(NCData *dataPtr) { Q_UNUSED(dataPtr) throw std::runtime_error("overloaded computeE2E() Not used in this class! :: NC_DRR_QuantumAssignmentImproved"); }

    // set critical class constraints
    void setDeadline(int classID, double deadline) { QList<int> classes = NC_DRR::getClasses(); if(classes.contains(classID)) {(classes.last()!=classID)?cID_deadline.insert(classID, deadline):throw std::runtime_error("cannot set a delay constraint on a non-critical class! :: NC_DRR_QuantumAssignmentImproved::setDeadline()");} else throw std::runtime_error("Invalid classID in :: NC_DRR_QuantumAssignmentImproved::setDeadline()"); }

private:
    QMap<int,double> cID_deadline; // <class ID, deadline in micro sec>

    // get critical class constraints
    double const& getDeadline(int const &cID) const { auto it = cID_deadline.find(cID); if(it != cID_deadline.end()) return it.value(); throw std::runtime_error("Invalid classID in :: NC_DRR_QuantumAssignmentImproved::getDeadline()"); }

    void arrangeClasses_basedOnDeviation(QList<int> &classes, QMap<int,int> const &refDistribution, NCData const *dataPtr, bool increasing = false);

    //      this is a re-implementation of NC_DRR_Optimised adapted for the quantum assignment algorithm.
    class NC_Optimised
    {
    public:
        // compute worst-case end-to-end delay for all flows on all paths in the network.
        static QMap<int,double> computeMaximumE2E(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData);

    private:
        // compute worst-case delay for refFlow at the given node
        static double computeNodeDelay(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the overall arrival curve for refFlow at the given node
        static ConcaveCurve computeOverallArrivalCurve(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the jitter upper bound for refFlow at the given node
        static double computeJitter(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the service curve for refFlow at the given node
        static ConvexCurve computeServiceCurve(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // optimise the computed delay for refFlow at the given node
        static double computeOptimisedNodeDelay(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);

    };
};

#endif // NC_DRR_QUANTUMASSIGNMENTIMPROVED_H
