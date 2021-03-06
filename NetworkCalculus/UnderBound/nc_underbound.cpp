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
#include "NetworkCalculus/UnderBound/nc_underbound.h"

NC_UNDERBOUND::NC_UNDERBOUND(const SCHEDULING &sPolicy)
{
    ncData = nullptr;
    policy = sPolicy;
}

// compute under bound on worst-case end-to-end delay for all flows on all paths in the network
void NC_UNDERBOUND::computeE2E(NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_UNDERBOUND::computeE2E()");
    if(!((policy == SCHEDULING::FIFO)||(policy == SCHEDULING::SPQ))) throw std::runtime_error("Invalid scheduling policy in :: NC_UNDERBOUND::computeE2E()");

    // the E2E delay under bound computation is divided into multiple steps
    //      these steps are same for each scheduling policy
    //      the only difference is in the computation within each step.
    //
    // Step 1 : select a flow (Vref)
    // Step 2 : select a path (Pi_Vref)
    // Step 3 : compute under bound on worst-case delay at each node in the path (a.k.a. feed forward)
    //  Step 3a: compute optimistic overall arrival curve (alpha_o)
    //  Step 3b: compute optimistic service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref
    // Step 4 : compute e2e delay.


    qStdOut() << "start e2e computation!";

    // create local reference to NCData
    ncData = dataPtr;

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)
        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute under bound on worst-case delay at each node in the path
                double delay = computeNodeDelay(flow, node);

                e2eDelay += delay;                       // Step 4 : compute e2e delay.
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // store result
            flowID_pathNo_e2eDelays.insert(flow.getFlowID(), qMakePair(pNo,e2eDelay));
        }

    }
}

// Step 3 : compute under bound on worst-case delay at node
double NC_UNDERBOUND::computeNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{

    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_UNDERBOUND::computeNodeDelay()");
    if(!((policy == SCHEDULING::FIFO)||(policy == SCHEDULING::SPQ))) throw std::runtime_error("Invalid scheduling policy in :: NC_UNDERBOUND::computeE2E()");
    QList<NCData::NCNode>::iterator ncNode = ncData->selectNCNode(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode->getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    //  Step 3a: compute optimistic overall arrival curve (alpha_o)
    //      alpha_o computation is divided into multiple parts:
    //      (1) compute optimistic arival curve of each flow arriving at this output port.
    //          Note: jitter = 0 and BAG = +inf to introduce optimism.
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //  Step 3b: compute optimistic service curve (beta_Vref)
    //      NOTE: an optimistic service curve
    //                  in FIFO : is the same as the original service curve in NC_FIFO, which is not pessimistic.
    //                  in SPQ : is obtained by considering no reduced service due to flows of other priorities.
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    ConcaveCurve alpha_o = computeOverallArrivalCurve(refFlow, node); //  Step 3a: compute optimistic overall arrival curve (alpha_o)
    ConvexCurve beta = computeServiceCurve(refFlow, node);            //  Step 3b: compute optimistic service curve (beta_Vref)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode->addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute optimistic overall arrival curve (alpha_o)
ConcaveCurve NC_UNDERBOUND::computeOverallArrivalCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_UNDERBOUND::computeOverallArrivalCurve()");;
    if(!((policy == SCHEDULING::FIFO)||(policy == SCHEDULING::SPQ))) throw std::runtime_error("Invalid scheduling policy in :: NC_UNDERBOUND::computeE2E()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    ConcaveCurve alpha_o; // overall arrival curve
    // check if the overall arrival curve for the refFlow is already computed
    if(ncNode.getComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o)) {
        return alpha_o;
    }

    //  Step 3a: compute optimistic overall arrival curve (alpha_o)
    //      alpha_o computation is divided into multiple parts:
    //      (1) compute arival curve of each flow arriving at this output port.
    //          Note: jitter = 0 and BAG = +inf to introduce optimism.
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //          Note: AFDX end-systems implement only FIFO
    // ----------------------------------------------------------------------------


    //      (1) compute arival curve of each flow arriving at this output port.
    //          NOTE : to introduce optimism  (https://ieeexplore.ieee.org/abstract/document/7993380)
    //                      jitter = 0  and  BAG = infinite
    //              alpha = rate*(t + jitter) + burst
    //                  burst = (max frame size)
    //                  rate = (max frame size)/BAG = 0
    //          Note: AFDX end-systems implement only FIFO
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Px at this output port
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        if(ncNode.getNodeType() == NODE_TYPE::SW){ // in case of SPQ scheduling
            if(policy == SCHEDULING::SPQ)
                if(flow.getPriority() != refFlow.getPriority()) continue;
        }
        double burst = flow.getLMax(); // burst = (max frame size)
        double BAG = 1.0/0.0; // +inf
        double rate = trimDouble(flow.getLMax()/BAG); // rate = 0
        // double jitter = computeJitter(flow, node); // jitter = 0
        ConcaveCurve alpha(burst, rate);
        ncNode.addComputedArrivalCurve(node.second, fID, alpha);
    }

    //      (2) compute cumulative curve from all the arrival curves.
    //          at a switch output port
    //              based on serialisation effect: (https://hal.archives-ouvertes.fr/hal-02270458)
    //                  the flows sharing the same input port are serialised and cannot create a sum of burst at an output port.
    //              (a) the flows sharing the same input port are groupped together V_grp_input.
    //              (b) the burst due to V_grp_input is limited to maximum burst in a group. : maxBurst_V_group_input
    //              (c) the arrival rate is limited by the link rate : R
    //              (d) the arrival curve of V_grp_input is
    //                  alpha_grp_input = min(arrival_curve(rate = R, burst = maxBurst_V_group_input), sum(arrival curves of the flows in V_grp_input))
    //              (e) the overall arrival curve is
    //                  alpha_o = sum(alpha_grp_input1, alpha_grp_input2, ...)
    //          at an end-system (AFDX end-systems implement only FIFO)
    //                  alpha_o = sum of arrival curves of all the flows

    //          at an end-system (AFDX end-systems implement only FIFO))
    if(ncNode.getNodeType() == NODE_TYPE::ES) { // qStdOut() << "ES alpha_o " << NODE_t_toString(node);
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow at this output port
            ConcaveCurve alpha;
            if(ncNode.getComputedArrivalCurve(node.second, fID, alpha)) { // get arrival curve
                if (alpha_o.isBaseCurve()) alpha_o = alpha;
                else alpha_o += alpha; // alpha_o = sum of arrival curves of all the flows
            } else throw std::runtime_error("arrival curve not found :: computeOverallArrivalCurve()");
        }

        // store result
        ncNode.addComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o);
        return alpha_o;
    }

    //          at a switch output port
    if(ncNode.getNodeType() == NODE_TYPE::SW) { // qStdOut() << "SW alpha_o " << NODE_t_toString(node);
        //              (a) the flows sharing the same input port are groupped together V_grp_input.
        QMultiMap<NODE_t, int> inputNode_fID_map; // <inputNode, flowID>
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow at this output port
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            if(ncNode.getNodeType() == NODE_TYPE::SW){ // in case of SPQ scheduling
                if(policy == SCHEDULING::SPQ)
                    if(flow.getPriority() != refFlow.getPriority()) continue;
            }
            NODE_t inputNode = flow.getPreviousNode(node); // fetch input node
            inputNode_fID_map.insert(inputNode, fID); // group fIDs based on input node
        }

        // make group arrival curve
        QList<ConcaveCurve> groupArrivalCurves; // list of group arrival curves
        foreach (NODE_t const&inputNode, inputNode_fID_map.uniqueKeys()) {
            //          (b) the burst due to V_grp_input is limited to maximum burst in a group. : maxBurst_V_group_input
            double maxBurst = 0.0;

            ConcaveCurve alpha_sum; // sum(arrival curves of the flows in V_grp_input)
            foreach (int const&fID, inputNode_fID_map.values(inputNode)) {
                ConcaveCurve alpha;
                if(ncNode.getComputedArrivalCurve(node.second, fID, alpha)) { // get arrival curve
                    if(alpha_sum.isBaseCurve()) alpha_sum = alpha;
                    else alpha_sum += alpha; // make sum
                    double burst = alpha.getBurstValue();
                    if(maxBurst < burst) maxBurst = burst; // get maxBurst
                } else throw std::runtime_error("arrival curve not found :: computeOverallArrivalCurve()");
            }

            //          (c) the arrival rate is limited by the link rate : R
            double R = ncNode.getLinkRate(node.second);

            //          (d) the arrival curve of group is alpha_grp_input = min(alpha(maxBurst, R), alpha_sum)
            ConcaveCurve alpha(maxBurst, R);
            groupArrivalCurves.append(min(alpha,alpha_sum));
        }

        //              (e) the overall arrival curve is
        //                  alpha_o = sum(alpha_grp_input1, alpha_grp_input2, ...)
        foreach (ConcaveCurve const&alpha_grp, groupArrivalCurves) {
            if(alpha_o.isBaseCurve()) alpha_o = alpha_grp;
            else alpha_o += alpha_grp; // make sum
        }

        // store result
        ncNode.addComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o);
        return alpha_o;
    }

    return alpha_o;
}

// compute the jitter upper bound for refFlow at the given node
double NC_UNDERBOUND::computeJitter(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_UNDERBOUND::computeJitter()");;
    if(!((policy == SCHEDULING::FIFO)||(policy == SCHEDULING::SPQ))) throw std::runtime_error("Invalid scheduling policy in :: NC_UNDERBOUND::computeE2E()");
    Q_UNUSED(refFlow)
    Q_UNUSED(node)

    //  optimism is introduced in an arrival curve by assuming :
    //      jitter = 0

    return 0.0;
}

//  Step 3b: compute optimistic service curve (beta_Vref = beta_Px)
ConvexCurve NC_UNDERBOUND::computeServiceCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_UNDERBOUND::computeServiceCurve()");;
    if(!((policy == SCHEDULING::FIFO)||(policy == SCHEDULING::SPQ))) throw std::runtime_error("Invalid scheduling policy in :: NC_UNDERBOUND::computeE2E()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  Step 3b: compute optimistic service curve (beta_Vref)
    //      NOTE: an optimistic service curve
    //                  in FIFO : is the same as the original service curve in NC_FIFO, which is not pessimistic.
    //                  in SPQ : is obtained by considering no reduced service due to flows of higher priorities.
    // ----------------------------------------------------------------------------

    double lMax_Pz = 0.0;
    if(policy == SCHEDULING::SPQ){
        if(ncNode.getNodeType() == NODE_TYPE::SW) {
            //              (1) identify largest frame size (lMax_Pz) among all the lower priority flows (Pz < Px) at this node.
            foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Pz (<Px) at this output port
                NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
                if(!(flow.getPriority() < refFlow.getPriority())) continue;
                if(lMax_Pz < flow.getLMax()) lMax_Pz = flow.getLMax(); // identify largest frame
            }
        }
        //              (2) compute the leftover service for Px flows
        //                      beta_Px = beta - lMax_Pz
        //                              = R[t -sl - (lMax_Pz/R)]+
    }

    double R = ncNode.getLinkRate(node.second); // fetch link rate
    double sl = ncNode.getSwitchingLatnecy(); // fetch switching latency
    ConvexCurve beta((sl+trimDouble(lMax_Pz/R)),R); // compute leftover service for Px flows
    return beta;

}
