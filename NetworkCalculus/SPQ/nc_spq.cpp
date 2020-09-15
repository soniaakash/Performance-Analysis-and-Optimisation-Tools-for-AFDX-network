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
#include "NetworkCalculus/SPQ/nc_spq.h"

NC_SPQ::NC_SPQ()
{
    ncData = nullptr;
}

// compute worst-case end-to-end delay for all flows on all paths in the network
void NC_SPQ::computeE2E(NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_SPQ::computeE2E()");

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


    qStdOut() << "start e2e computation!";

    // create local reference to NCData
    ncData = dataPtr;

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)
        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute worst-case delay at each node in the path
                double delay = computeNodeDelay(flow, node);

                e2eDelay += delay;                       // Step 4 : compute e2e delay.
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // store result
            flowID_pathNo_e2eDelays.insert(flow.getFlowID(), qMakePair(pNo,e2eDelay));
        }

    }
}

// Step 3 : compute worst-case delay at node
double NC_SPQ::computeNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{

    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_SPQ::computeNodeDelay()");
    QList<NCData::NCNode>::iterator ncNode = ncData->selectNCNode(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode->getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Px)
    //      alpha_o computation is divided into multiple parts:
    //          Note: the overall arrival curve for Vref corresponds to the curve (alpha_Px) of its corresponding priority (Px)
    //      (1) compute arival curve of each flow in Px arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require delay at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //  Step 3b: compute service curve (beta_Vref = beta_Px)
    //      beta_Px computation is divided into multiple parts: https://hal.inria.fr/inria-00431674
    //                  Note: the service of Vref corresponds to the service (beta_Px) provided to its corresponding priority (Px)
    //                        it is the leftover service after serving all the higher priority flows (> Px)
    //                        and possibly at most one non-preemptive lower priority flow (< Px).
    //              (1) compute sum of arrival curves sum_Py(alpha) of higher priority flows (Py > Px) at this node.
    //              (2) identify largest frame size (lMax_Pz) among all the lower priority flows (Pz < Px) at this node.
    //              (3) compute the leftover service for Px flows
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    ConcaveCurve alpha_o = computeOverallArrivalCurve(refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = computeServiceCurve(refFlow, node);            //  Step 3b: compute service curve (beta_Vref)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode->addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o)
ConcaveCurve NC_SPQ::computeOverallArrivalCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_SPQ::computeOverallArrivalCurve()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    ConcaveCurve alpha_o; // overall arrival curve
    // check if the overall arrival curve for the refFlow is already computed
    if(ncNode.getComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o)) {
        return alpha_o;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Px)
    //      alpha_o computation is divided into multiple parts:
    //          Note: the overall arrival curve for Vref corresponds to the curve (alpha_Px) of its corresponding priority (Px)
    //      (1) compute arival curve of each flow in Px arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require delay at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //          Note: AFDX end-systems implement only FIFO
    // ----------------------------------------------------------------------------


    //      (1) compute arival curve of each flow in Px arriving at this output port.
    //                  jitter integration increases the burst (https://ieeexplore-ieee-org.gorgone.univ-toulouse.fr/document/5524098)
    //              alpha = rate*(t + jitter) + burst
    //                  burst = (max frame size)
    //                  rate = (max frame size)/BAG
    //                  increased burst = (rate * jitter) + burst
    //          Note: AFDX end-systems implement only FIFO
    const int Px = refFlow.getPriority();
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Px at this output port
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        if(ncNode.getNodeType() == NODE_TYPE::SW){
            if(flow.getPriority() != Px) continue;
        }
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

    //          at a switch output port (SPQ scheduling)
    if(ncNode.getNodeType() == NODE_TYPE::SW) { // qStdOut() << "SW alpha_o " << NODE_t_toString(node);
        //              (a) the flows sharing the same input port are groupped together V_grp_input.
        QMultiMap<NODE_t, int> inputNode_fID_map; // <inputNode, flowID>
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Px at this output port
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            if(flow.getPriority() != Px) continue;
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
double NC_SPQ::computeJitter(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_SPQ::computeJitter()");;
    // NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

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

//  Step 3b: compute service curve (beta_Vref = beta_Px)
ConvexCurve NC_SPQ::computeServiceCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_SPQ::computeServiceCurve()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  Step 3b: compute service curve (beta_Vref = beta_Px)
    //          at a switch output port
    //               beta_Px computation is divided into multiple parts: https://hal.inria.fr/inria-00431674
    //                  Note: the service of Vref corresponds to the service (beta_Px) provided to its corresponding priority (Px)
    //                        it is the leftover service after serving all the higher priority flows (> Px)
    //                        and possibly at most one non-preemptive lower priority flow (< Px).
    //              (1) compute sum of arrival curves sum_Py(alpha) of higher priority flows (Py > Px) at this node.
    //              (2) identify largest frame size (lMax_Pz) among all the lower priority flows (Pz < Px) at this node.
    //              (3) compute the leftover service for Px flows
    //                      beta_Px = beta - sum_Py(alpha) - lMax_Pz
    //                              = R'[t - sl - theta]+        where R' = (R - sum_Py_rate) and theta = (sum_Py_burst + lMax_Pz)/(R')
    //
    //  Note : this left-over service computation is very pessimistic, since
    //          it does not consider serialisation of high priority flows,
    //          and it con be improved by finding a way to compute (beta - alpha_highPriority) for alpha_highPriority with integrated serialisation.
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
        //              (1) compute sum of arrival curves sum_Py(alpha) of higher priority flows (Py > Px) at this node.
        ConcaveCurve sum_Py_alpha;
        const int Px = refFlow.getPriority();
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Py (>Px) at this output port
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            if(!(flow.getPriority() > Px)) continue;
            double burst = flow.getLMax(); // burst = (max frame size)
            double BAG = flow.getBAG();
            double rate = trimDouble(flow.getLMax()/BAG); // rate = (max frame size)/BAG
            double jitter = computeJitter(flow, node);
            burst = trimDouble(rate*jitter) + burst; // increased burst = (rate * jitter) + burst
            ConcaveCurve alpha(burst, rate);
            ncNode.addComputedArrivalCurve(node.second, fID, alpha);

            if (sum_Py_alpha.isBaseCurve()) sum_Py_alpha = alpha;
            else sum_Py_alpha += alpha; // make sum
        }

        //              (2) identify largest frame size (lMax_Pz) among all the lower priority flows (Pz < Px) at this node.
        double lMax_Pz = 0.0;
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Pz (<Px) at this output port
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            if(!(flow.getPriority() < Px)) continue;
            if(lMax_Pz < flow.getLMax()) lMax_Pz = flow.getLMax(); // identify largest frame
        }

        //              (3) compute the leftover service for Px flows
        //                      beta_Px = beta - sum_Py_alpha) - lMax_Pz
        //                              = R'[t - sl - theta]+
        //              where beta = R[t]+ and R' = (R - sum_Py_rate) and theta = (sum_Py_burst + lMax_Pz)/(R')
        double R = ncNode.getLinkRate(node.second); // fetch link rate
        double sl = ncNode.getSwitchingLatnecy(); // fetch switching latency
        double sum_Py_rate = 0.0;
        double sum_Py_burst = 0.0;
        if(!sum_Py_alpha.isBaseCurve()) {
            sum_Py_burst = sum_Py_alpha.getBurstValue(); // get sum of high priority flow bursts
            sum_Py_rate = sum_Py_alpha.getFinalRateValue(); // get sum of high priority flow arrival rates
        }
        double theta = trimDouble((sum_Py_burst + lMax_Pz)/trimDouble(R - sum_Py_rate)); // compute increased latency
        ConvexCurve beta((sl+theta),trimDouble(R - sum_Py_rate)); // compute leftover service for Px flows
        return beta;
    }

    throw std::runtime_error("Unable to compute service curve. Invalid node type. :: NC_SPQ::computeServiceCurve()");
}
