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

#include "nc_drr_classic.h"

NC_DRR_Classic::NC_DRR_Classic()
{
    ncData = nullptr;
}

// compute worst-case end-to-end delay for all flows on all paths in the network
void NC_DRR_Classic::computeE2E(NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_Classic::computeE2E()");

    // the E2E delay computation is divided into multiple steps
    //      these steps are same for each scheduling policy
    //      the only difference is in the computation within each step.
    //
    // Step 1 : select a flow (Vref)
    // Step 2 : select a path (Pi_Vref)
    // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
    //        Note: in DRR scheduling, computation is "per class basis"
    //  Step 3a: compute overall arrival curve (alpha_o)
    //  Step 3b: compute service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref
    // Step 4 : compute e2e delay.


    qStdOut() << "start e2e computation!";

    // create local reference to NCData
    ncData = dataPtr;

    // intialise DRR
    if(!initDRRConfig(ncData)) return;

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)
        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute worst-case delay at each node in the path
                double delay = computeNodeDelay(flow, node);
                qStdOut() << flow.getFlowID() << " NODE " << NODE_t_toString(node) << " Delay " << delay << " microSec";

                e2eDelay += delay;                       // Step 4 : compute e2e delay.
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // store result
            flowID_pathNo_e2eDelays.insert(flow.getFlowID(), qMakePair(pNo,e2eDelay));
        }

    }

}

// Step 3 : compute worst-case delay at node
double NC_DRR_Classic::computeNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_Classic::computeNodeDelay()");
    if(!configReady) throw std::runtime_error("DRR not configured! :: NC_DRR_Classic::computeNodeDelay()");
    QList<NCData::NCNode>::iterator ncNode = ncData->selectNCNode(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode->getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
    //      alpha_o computation is divided into multiple parts:
    //          Note: the overall arrival curve for Vref corresponds to the curve (alpha_Cx) of its corresponding class (Cx)
    //      (1) compute arival curve of each flow in Cx arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require delay at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //  Step 3b: compute service curve (beta_Vref = beta_Cx)
    //       beta_Cx computation is divided into multiple parts:
    //          Note: the service of Vref corresponds to the service (beta_Cx) provided to its corresponding class (Cx)
    //      (1) compute residual service rate rho_Cx
    //      (2) compute scheduler latency theta_Cx
    //      (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
    //  Step 3c: compute horizontal distance alpha_o and beta_Cx.

    ConcaveCurve alpha_o = computeOverallArrivalCurve(refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = computeServiceCurve(refFlow, node);            //  Step 3b: compute service curve (beta_Cx)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode->addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
ConcaveCurve NC_DRR_Classic::computeOverallArrivalCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_Classic::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("DRR not configured! :: NC_DRR_Classic::computeOverallArrivalCurve()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    ConcaveCurve alpha_o; // overall arrival curve
    // check if the overall arrival curve for the refFlow is already computed
    if(ncNode.getComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o)) {
        return alpha_o;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
    //      alpha_o computation is divided into multiple parts:
    //          Note: the overall arrival curve for Vref corresponds to the curve (alpha_Cx) of its corresponding class (Cx)
    //      (1) compute arival curve of each flow in Cx arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //          Note: AFDX end-systems implement only FIFO
    // ----------------------------------------------------------------------------


    //      (1) compute arival curve of each flow in Cx arriving at this output port.
    //                  jitter integration increases the burst (https://ieeexplore-ieee-org.gorgone.univ-toulouse.fr/document/5524098)
    //              alpha = rate*(t + jitter) + burst
    //                  burst = (max frame size)
    //                  rate = (max frame size)/BAG
    //                  increased burst = (rate * jitter) + burst
    //          Note: AFDX end-systems implement only FIFO
    const int Cx = getClassID(refFlow.getFlowID());
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Cx at this output port
        if(ncNode.getNodeType() == NODE_TYPE::SW){
            if(getClassID(fID) != Cx) continue;
        }
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        double burst = flow.getLMax(); // burst = (max frame size)
        double BAG = flow.getBAG();
        double rate = trimDouble(flow.getLMax()/BAG); // rate = (max frame size)/BAG
        double jitter = computeJitter(flow, node);
        burst = trimDouble(rate*jitter) + burst; // increased burst = (rate * jitter) + burst
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

    //          at a switch output port (DRR scheduling)
    if(ncNode.getNodeType() == NODE_TYPE::SW) { // qStdOut() << "SW alpha_o " << NODE_t_toString(node);
        //              (a) the flows sharing the same input port are groupped together V_grp_input.
        QMultiMap<NODE_t, int> inputNode_fID_map; // <inputNode, flowID>
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Cx at this output port
            if(getClassID(fID) != Cx) continue;
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
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
double NC_DRR_Classic::computeJitter(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_Classic::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("DRR not configured! :: NC_DRR_Classic::computeOverallArrivalCurve()");

    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //
    //   jitter for a flow V_ref at a node h is computed as: (https://ieeexplore.ieee.org/document/5524098)
    //      (a) compute the worst-case delay till the previous node in the path
    //             D_max_till_(h-1) = sum(D_(endSystem), D_(switch_1), ... D_(switch_(h-1)))
    //      (b) compute the minimum delay till the previous node in the path
    //             D_min_till_(h-1) = sum(D_min_(endSystem), D_min_(switch_1), ... D_min_(switch_(h-1)))
    //                  minimum delay for any flow at any node is equal to its transmission time + switching latnecy
    //      (c) compute jitter
    //              jitter = D_max_till_(h-1) - D_min_till_(h-1)
    //  ---------------------------------------------------------------------------------------------

    //      (a) compute the worst-case delay till the previous node in the path
    double sum_Dmax = 0.0;
    //      (b) compute the minimum delay till the previous node in the path
    double sum_Dmin = 0.0;

    NODE_t prevNode = refFlow.getPreviousNode(node); // fetch previous node
    while(!NODE_generic::isVirtalES(prevNode)) // virtual node == end of path
    {
        sum_Dmax += computeNodeDelay(refFlow, prevNode); // make sum

        // minimum delay for any flow at any node is equal to its transmission time (tr) + switching latnecy (sl)
        NCData::NCNode &prev_ncNode = ncData->selectNCNode_ref(prevNode.first);
        double sl = prev_ncNode.getSwitchingLatnecy();
        double R = prev_ncNode.getLinkRate(prevNode.second);
        double tr = trimDouble(refFlow.getLMax()/R); // max transmission time = (max frame length) / link rate
        sum_Dmin += (tr + sl); // make sum

        prevNode = refFlow.getPreviousNode(prevNode); // fetch previous node
    }

    //      (c) compute jitter = D_max_till_(h-1) - D_min_till_(h-1)
    double jitter = sum_Dmax - sum_Dmin;
    return jitter;
}

//  Step 3b: compute service curve (beta_Vref = beta_Cx)
ConvexCurve NC_DRR_Classic::computeServiceCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_Classic::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("DRR not configured! :: NC_DRR_Classic::computeOverallArrivalCurve()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  Step 3b: compute service curve (beta_Vref = beta_Cx)
    //          at a switch output port
    //               beta_Cx computation is divided into multiple parts:
    //                  Note: the service of Vref corresponds to the service (beta_Cx) provided to its corresponding class (Cx)
    //              (1) compute residual service rate rho_Cx
    //              (2) compute scheduler latency theta_Cx
    //              (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
    //
    //          at an end-system (AFDX end-systems implement only FIFO)
    //              beta_Vref computation is straight forward in FIFO.
    //              beta = R[t - sl]+
    // ----------------------------------------------------------------------------

    //          at an end-system
    if(ncNode.getNodeType() == NODE_TYPE::ES) {
        //              beta_Vref computation is straight forward in FIFO.
        //              beta = R[t - sl]+
        double R = ncNode.getLinkRate(node.second);
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta(sl,R);
        return beta;
    }

    //          at a switch output port
    if(ncNode.getNodeType() == NODE_TYPE::SW) {
        //      (1) compute residual service rate rho_Cx
        //              rho_Cx = (quantum_Cx / sum of quanta of all the classes) linkRate
        const int Cx = getClassID(refFlow.getFlowID());
        double Q_Cx = cID_quantum_map.value(Cx);
        double Q_sum = 0.0;
        foreach (double const&Q_Ci, cID_quantum_map.values()) {
            Q_sum += Q_Ci; // make sum
        }
        double R = ncNode.getLinkRate(node.second);
        double rho_Cx = trimDouble(trimDouble(Q_Cx/Q_sum) * R);

        //      (2) compute scheduler latency
        //          scheduler latency is divided into two parts: (https://ieeexplore.ieee.org/document/8603222)
        //          (a) X_Cx = sum(Q_Ci + maximum deficit of Ci) / R       (Caution :  Ci != Cx)
        //                  maximum deficit of Ci = (max frame size in bytes) - 1 (in bytes) = (max frame size in bits) - 8 (in bits)
        //          (b) Y_Cx = ((Q_Cx - max deficit of Cx) + sum(Q_Ci))/R - (Q_Cx - max deficit of Cx)/rho_Cx
        //            theta_Cx = X_Cx + Y_Cx
        double X_Cx = 0.0;
        foreach (int const&Ci, cID_quantum_map.keys()) {
            if(Ci == Cx) continue;
            double Q_Ci = cID_quantum_map.value(Ci); // in bits
            double lMax_Ci = cID_lMax_map.value(Ci); // in bits
            X_Cx += trimDouble((Q_Ci + lMax_Ci - 8)/R);
        }

        double Y_Cx = 0.0;
        double lMax_Cx = cID_lMax_map.value(Cx); // in bits
        Y_Cx += trimDouble((Q_Cx - (lMax_Cx - 8) + (Q_sum - Q_Cx))/R);
        Y_Cx -= trimDouble((Q_Cx - (lMax_Cx - 8))/rho_Cx);

        //            theta_Cx = X_Cx + Y_Cx
        double theta_Cx = X_Cx + Y_Cx;

        //      (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta((sl+theta_Cx),rho_Cx);
        return beta;
    }

    throw std::runtime_error("Unable to compute service curve. Invalid node type. :: NC_DRR_Classic::computeServiceCurve()");
}
