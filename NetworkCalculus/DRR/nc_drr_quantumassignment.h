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

#ifndef NC_DRR_QUANTUMASSIGNMENT_H
#define NC_DRR_QUANTUMASSIGNMENT_H

#include "NetworkCalculus/DRR/nc_drr.h"

// implementation of DRR Quanum Assignment algorithm
//      based on https://dl.acm.org/doi/10.1145/3356401.3356421 (Quantum assignment for QoS-aware AFDX network with deficit round robin)
class NC_DRR_QuantumAssignment : public NC_DRR
{
    friend class NC_Classic;

public:
    NC_DRR_QuantumAssignment();

    // initialise data and start computation
    void initComputation(NCData const *dataPtr);

    // compute the optimal quantum and its distribution
    QMap<int,int> computeOptimalQuantumAndItsDistribution(int const&initialQuantumValue, NCData const *dataPtr);

    // compute the minimum quantum requirement for a given class to respect the given deadline
    int computeMinimumQ(int const&classID, double const&deadline, NCData const *dataPtr);

    // implementation of a pure virtual function from parent NC_DRR
    void computeE2E(NCData *dataPtr) { Q_UNUSED(dataPtr) throw std::runtime_error("overloaded computeE2E() Not used in this class! :: NC_DRR_QuantumAssignment"); }

    // set critical class constraints
    void setDeadline(int classID, double deadline) { QList<int> classes = NC_DRR::getClasses(); if(classes.contains(classID)) {(classes.last()!=classID)?cID_deadline.insert(classID, deadline):throw std::runtime_error("cannot set a delay constraint on a non-critical class! :: NC_DRR_QuantumAssignment::setDeadline()");} else throw std::runtime_error("Invalid classID in :: NC_DRR_QuantumAssignment::setDeadline()"); }

private:
    int Q_global; // global quantum
    QMap<int,double> cID_deadline; // <class ID, deadline in micro sec>

    // get critical class constraints
    double const& getDeadline(int const &cID) const { auto it = cID_deadline.find(cID); if(it != cID_deadline.end()) return it.value(); throw std::runtime_error("Invalid classID in :: NC_DRR_QuantumAssignment::getDeadline()"); }

    //      this is a re-implementation of NC_DRR_Classic adapted for the quantum assignment algorithm.
    class NC_Classic
    {
    public:
        // compute worst-case end-to-end delay for all flows of the given class on all paths in the network.
        static double computeMaximumE2E(NC_DRR_QuantumAssignment *parent, NCData *ncData, const int &classID);

    private:
        // compute worst-case delay for refFlow at the given node
        static double computeNodeDelay(NC_DRR_QuantumAssignment *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the overall arrival curve for refFlow at the given node
        static ConcaveCurve computeOverallArrivalCurve(NC_DRR_QuantumAssignment *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the jitter upper bound for refFlow at the given node
        static double computeJitter(NC_DRR_QuantumAssignment *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
        // compute the service curve for refFlow at the given node
        static ConvexCurve computeServiceCurve(NC_DRR_QuantumAssignment *parent, NCData *ncData, NCData::NCFlow const&refFlow, NODE_t const&node);
    };

};

#endif // NC_DRR_QUANTUMASSIGNMENT_H
