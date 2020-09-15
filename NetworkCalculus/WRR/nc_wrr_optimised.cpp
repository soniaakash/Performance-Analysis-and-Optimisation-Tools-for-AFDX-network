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
#include "nc_wrr_optimised.h"

NC_WRR_Optimised::NC_WRR_Optimised()
{
    ncData = nullptr;
}

// compute worst-case end-to-end delay for all flows on all paths in the network
void NC_WRR_Optimised::computeE2E(NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeE2E()");

    // the E2E delay computation is divided into multiple steps
    //      these steps are same for each scheduling policy
    //      the only difference is in the computation within each step.
    //
    // Step 1 : select a flow (Vref)
    // Step 2 : select a path (Pi_Vref)
    // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
    //        Note: in WRR scheduling, computation is "per class basis"
    //  Step 3a: compute overall arrival curve (alpha_o)
    //  Step 3b: compute service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref
    //  Step 3d: optimise the computed worst-case delay at the given node
    // Step 4 : compute e2e delay.


    qStdOut() << "start e2e computation!";

    // create local reference to NCData
    ncData = dataPtr;

    // intialise WRR
    if(!initWRRConfig(ncData)) return;

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)
        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute worst-case delay at each node in the path
                double delay = computeNodeDelay(flow, node);
                // qStdOut() << flow.getFlowID() << " NODE " << NODE_t_toString(node) << " Delay " << delay << " microSec";

                e2eDelay += delay;                       // Step 4 : compute e2e delay.
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // store result
            flowID_pathNo_e2eDelays.insert(flow.getFlowID(), qMakePair(pNo,e2eDelay));
        }
    }
}

// Step 3 : compute worst-case delay at node
double NC_WRR_Optimised::computeNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeNodeDelay()");
    if(!configReady) throw std::runtime_error("WRR not configured! :: NC_WRR_Optimised::computeNodeDelay()");
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
    //  Step 3d: optimise the computed worst-case delay at the given node
    //          it includes computation of over-estimated load from competing classes (Ci != Cx)
    //          Note: over-estimated load computation require overall arrival curve of Ci (which require recursive computations)
    //

    ConcaveCurve alpha_o = computeOverallArrivalCurve(refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = computeServiceCurve(refFlow, node);            //  Step 3b: compute service curve (beta_Cx)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.
    if(!std::isfinite(delay)) {qStdOut() << " inf delay in C" << getClassID(refFlow.getFlowID()) << "\n"; exit(0);}
    // store result
    ncNode->addComputedDelay(node.second, refFlow.getFlowID(), delay);

    delay = computeOptimisedNodeDelay(refFlow, node);

    // store optimised result
    ncNode->addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
ConcaveCurve NC_WRR_Optimised::computeOverallArrivalCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("WRR not configured! :: NC_WRR_Optimised::computeOverallArrivalCurve()");
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

    //          at a switch output port (WRR scheduling)
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
double NC_WRR_Optimised::computeJitter(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("WRR not configured! :: NC_WRR_Optimised::computeOverallArrivalCurve()");

    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //  (cite :: ...)
    //   jitter for a flow V_ref at a node h is computed as:
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
ConvexCurve NC_WRR_Optimised::computeServiceCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("WRR not configured! :: NC_WRR_Optimised::computeOverallArrivalCurve()");
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
        //              rho_Cx = ((weight_Cx * l_min_Cx) / ((weight_Cx * l_min_Cx) + sum of (weight * l_max) of competing classes)) linkRate
        const int Cx = getClassID(refFlow.getFlowID());

        //
        QMap<int, double> lMax_Ci_atThisNode;
        foreach (int const& Ci, cID_weight_map.uniqueKeys()) {
            if(Ci == Cx) continue;

            double lMax_Ci = -1;
            QList<int> const &fList = ncNode.getFlowList(node.second);
            foreach (int const&fID, fList) {
                if(getClassID(fID) != Ci) continue;
                NCData::NCFlow const& f = ncData->selectNCFlow_ref(fID);
                if(lMax_Ci < f.getLMax()) { lMax_Ci = f.getLMax();}
            }
            if(lMax_Ci < 0) continue;
            lMax_Ci_atThisNode.insert(Ci,lMax_Ci);
        }
        //

        double W_Cx = cID_weight_map.value(Cx);
        // double lMin_Cx = cID_lMin_map.value(Cx);
        double lMin_Cx = 1.0/0.0; //cID_lMin_map.value(Cx);
        { // lMin at node
            QList<int> const &fList = ncNode.getFlowList(node.second);
            foreach (int const&fID, fList) {
                if(getClassID(fID) != Cx) continue;
                NCData::NCFlow const& f = ncData->selectNCFlow_ref(fID);
                if(lMin_Cx > f.getLMin()) { lMin_Cx = f.getLMin();}
            }
            if(!std::isfinite(lMin_Cx)) throw std::runtime_error("something went wrong in Cx :: serviceCurve");
        }
        double R = ncNode.getLinkRate(node.second);
        double rho_Cx = trimDouble(W_Cx * lMin_Cx * R);
        {
            double divisor = trimDouble(W_Cx * lMin_Cx);
            foreach (int const& Ci, cID_weight_map.uniqueKeys()) {
                if(Ci == Cx) continue;
                if(!lMax_Ci_atThisNode.contains(Ci)) continue; // no Ci flow at this node
                divisor += trimDouble(cID_weight_map.value(Ci) * lMax_Ci_atThisNode.value(Ci)); // cID_lMax_map.value(Ci));
            }
            rho_Cx = trimDouble(rho_Cx/divisor);
        }

        //      (2) compute scheduler latency
        //          scheduler latency is computed as: (cite:...)
        //          (a) theta_Cx = sum(W_Ci * maximum frame length of Ci) / R       (Caution :  Ci != Cx)
        double theta_Cx = 0.0;
        foreach (int const&Ci, cID_weight_map.keys()) {
            if(Ci == Cx) continue;
            if(!lMax_Ci_atThisNode.contains(Ci)) continue; // no Ci flow at this node
            double W_Ci = cID_weight_map.value(Ci); // in number of frames
            double lMax_Ci = lMax_Ci_atThisNode.value(Ci); // cID_lMax_map.value(Ci); // in bits
            theta_Cx += trimDouble((W_Ci * lMax_Ci)/R);
        }

        //      (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta((sl+theta_Cx),rho_Cx);
        return beta;
    }

    throw std::runtime_error("Unable to compute service curve. Invalid node type. :: NC_WRR_Optimised::computeServiceCurve()");
}

//  Step 3d: optimise the computed worst-case delay at the given node
double NC_WRR_Optimised::computeOptimisedNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_WRR_Optimised::computeOverallArrivalCurve()");
    if(!configReady) throw std::runtime_error("WRR not configured! :: NC_WRR_Optimised::computeOverallArrivalCurve()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  for a flow (Vref) having a service curve beta_Cx and an overall arrival curve alpha_Cx
    //  whose pessimistic delay computed by classical approach is D_Vref
    //  the optimised delay computation is divided into multiple parts:    (cite::...)
    //          Note: over-estimated load computation require overall arrival curve of Ci (which require recursive computations)
    //       (1) compute service load SL_Ci of competing class Ci
    //              it is the amount of Ci traffic considered by beta_Cx in the duration [0, D_servicepoint]
    //       (2) compute actual load L_Ci of competing class Ci
    //              it is the upper-bound on the amount of traffic that can arrive in Ci in the duration [0, D_servicepoint]
    //       (3) compute over-estimated load of competing class Ci
    //              it is the difference between the service load SL_Ci and the actual load L_Ci
    //       (4) compute optimised delay delay_opt
    //              it is obtained by substracting the transmission time of the over-estimated load of each cometing class
    //              from the delay D_Vref
    //
    //   D_servicepoint is the horizontal distance between x-axis and the point on service curve
    //   where worst-case delay D_Vref is computed.
    //          it is >= worst-case delay
    //
    //  a supplementary optimisation is applied to compute tighter delay bound (https://ieeexplore.ieee.org/abstract/document/8756011)
    //     it based on the consideration that the transmission rate for Cx flows
    //     is equal to the link rate (R) but not the residual service rate (rho_Cx)
    //       (5) introduce supplementary optimisation
    //             the classical NC approach assumes rate-latency curve
    //             this optimisation removes the horizontal difference between
    //             the staircase curve and the rate-latency curve in the last service round
    //
    //  Note: the optimisation is applicable only to the output ports implementing WRR scheduling
    // ----------------------------------------------------------------------------

    //          at an end-system
    if(ncNode.getNodeType() == NODE_TYPE::ES) { // (AFDX end-systems implement only FIFO)
        return computeNodeDelay(refFlow, node);
    }

    //          at a switch output port
    if(ncNode.getNodeType() == NODE_TYPE::SW) {

        const int Cx = getClassID(refFlow.getFlowID()); // fetch Vref class ID

        //
        QMap<int, double> lMax_Ci_atThisNode;
        foreach (int const& Ci, cID_weight_map.uniqueKeys()) {
            if(Ci == Cx) continue;
            double lMax_Ci = -1;
            QList<int> const &fList = ncNode.getFlowList(node.second);
            foreach (int const&fID, fList) {
                if(getClassID(fID) != Ci) continue;
                NCData::NCFlow const& f = ncData->selectNCFlow_ref(fID);
                if(lMax_Ci < f.getLMax()) { lMax_Ci = f.getLMax();}
            }
            if(lMax_Ci < 0) continue;
            lMax_Ci_atThisNode.insert(Ci,lMax_Ci);
        }
        //

        // compute service point
        ConcaveCurve alpha_Cx = computeOverallArrivalCurve(refFlow, node);
        ConvexCurve beta_Cx = computeServiceCurve(refFlow, node);
        double D_servicepoint = maxHorizontalDistance_X0(alpha_Cx, beta_Cx, refFlow.getFlowID(), node);

        // prerequisite
        double R = ncNode.getLinkRate(node.second);     // fetch link rate
        double W_Cx = cID_weight_map.value(Cx);         // in number of frames
        // double lMin_Cx = cID_lMin_map.value(Cx);        // in bits
        double lMin_Cx = 1.0/0.0; //parent->cID_lMin_map.value(Cx);
        { // lMin at node
            QList<int> const &fList = ncNode.getFlowList(node.second);
            foreach (int const&fID, fList) {
                if(getClassID(fID) != Cx) continue;
                NCData::NCFlow const& f = ncData->selectNCFlow_ref(fID);
                if(lMin_Cx > f.getLMin()) { lMin_Cx = f.getLMin();}
            }
            if(!std::isfinite(lMin_Cx)) throw std::runtime_error("something went wrong in Cx :: computeOptimisedNodeDelay");
        }
        //

        double theta_Cx = 0.0; // compute theta_Cx
        foreach (int const&Cj, cID_weight_map.keys()) {
            if(Cj == Cx) continue;
            if(!lMax_Ci_atThisNode.contains(Cj)) continue;
            double W_Cj = cID_weight_map.value(Cj); // in number of frames
            double lMax_Cj = lMax_Ci_atThisNode.value(Cj); // cID_lMax_map.value(Cj); // in bits
            theta_Cx += trimDouble((W_Cj * lMax_Cj)/R);
        }

//        const int Cn = getClasses().last(); // fetch non-critical class ID

        double overestimatedLoad = 0.0;
        foreach (int const&Ci, cID_weight_map.keys()) { // for each competing class Ci
            if(Ci == Cx) continue;

//            if(Ci == Cn) continue; //   NOTE : the non-critical class Cn is assumed to have "infinite traffic". this assumption is implemented by considering (over-estimated load from Cn) = 0

            if(!lMax_Ci_atThisNode.contains(Ci)) continue; // no Ci flow at this node

            //       (1) compute service load SL_Ci of competing class Ci
            //              it is the amount of Ci traffic considered by beta_Cx in the duration [0, D_servicepoint]
            //          SL_Ci = 0                                                      , D_servicepoint < theta_Cx
            //                = ( 1 + floor(Z) ) (W_Ci * lMax_Ci) , theta_Cx <= D_servicepoint
            //
            //      Z = R * (D_servicepoint - theta_Cx) / ((W_Cx * lMin_Cx) + sum(W_Ci * lMax_Ci))


            // compute SL_Ci
            double SL_Ci = 0.0;
            if(D_servicepoint < theta_Cx) {
                SL_Ci = 0.0;
            } else { // D_servicepoint >= theta_Cx
                // compute Z = R * (D_servicepoint - theta_Cx) / ((W_Cx * lMin_Cx) + sum(W_Ci * lMax_Ci))
                double Z = trimDouble(R * trimDouble(D_servicepoint - theta_Cx));
                {
                    double divisor = trimDouble(W_Cx * lMin_Cx);
                    foreach (int const& Cj, cID_weight_map.uniqueKeys()) {
                        if(Cj == Cx) continue;
                        if(!lMax_Ci_atThisNode.contains(Cj)) continue; // no Cj flow at this node
                        divisor += trimDouble(cID_weight_map.value(Cj) * lMax_Ci_atThisNode.value(Cj)); // cID_lMax_map.value(Cj));
                    }
                    Z = trimDouble(Z/divisor);
                }

                double W_Ci = cID_weight_map.value(Ci);         // in number of frames
                double lMax_Ci = lMax_Ci_atThisNode.value(Ci);  // cID_lMax_map.value(Ci);        // in bits
                SL_Ci = trimDouble((1 + floor(Z))*(trimDouble(W_Ci * lMax_Ci))); // ( 1 + floor(Z) ) (W_Ci * lMax_Ci)
            }

            //       (2) compute actual load L_Ci of competing class Ci
            //              it is the upper-bound on the amount of traffic that can arrive in Ci in the duration [0, D_servicepoint]
            //              it can be obtained from overall arrival curve of any flow in Ci as
            //                  L_Ci = alpha_Ci(D_servicepoint)
            double L_Ci = 0.0;
            int fID_Ci = -1;
            foreach(const int &fID, ncNode.getFlowList(node.second)) { // fetch flowID of any Ci flow at the given node
                if(getClassID(fID) == Ci) {fID_Ci = fID; break; }
            }
            if(fID_Ci == -1) { L_Ci = 0.0; }
            else {
                NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID_Ci); // fetch flow with selected fID
                ConcaveCurve alpha_Ci = computeOverallArrivalCurve(flow, node); // fetch overall arrival curve

                // compute L_Ci = alpha_Ci(D_servicepoint)
                RayList rl_Ci = alpha_Ci.getRayList();
                RayList::iterator it = rl_Ci.begin(); RayList::iterator itEnd = rl_Ci.end();
                for(; it != itEnd; ++it) { // look for the line corresponding to D_servicepoint
                    if(it->p.x > D_servicepoint) break;
                } --it;
                L_Ci = trimDouble(it->k * (D_servicepoint - it->p.x)) + it->p.y; // fetch y-value at x = D_servicepoint
            }

            //       (3) compute over-estimated load of competing class Ci
            //              it is the difference between the service load SL_Ci and the actual load L_Ci
            double overestimatedLoad_Ci = trimDouble(SL_Ci - L_Ci);

            if(overestimatedLoad_Ci < 0) { // service load is less than the actual traffic : no overestimated traffic.
                overestimatedLoad_Ci = 0;
            }
            overestimatedLoad += overestimatedLoad_Ci;
        }

        //       (4) compute optimised delay delay_opt
        //              it is obtained by substracting the transmission time of the over-estimated load of each cometing class
        //              from the delay D_Vref
        double D_Vref = computeNodeDelay(refFlow, node);
        double delay_opt = trimDouble(D_Vref - trimDouble(overestimatedLoad/R));
return delay_opt;
        //       (5) introduce supplementary optimisation (https://ieeexplore.ieee.org/abstract/document/8756011)
        //             the NC approach assumes rate-latency curve with service rate rho_Cx
        //             this optimisation considers staircase curve with transmission rate of Cx flows equal to the link rate R
        //             it comes down to removal of horizontal distance between
        //                  the staircase curve (rate R) and the rate-latency curve (rate rho_Cx) in the last service round
        //             from the delay D_Vref.
        //           (a) indentify the last service round of Cx in the duration [0, D_servicepoint]
        //           (b) remove the horizontal distance between staircase curve and the rate-latency curve in the last service round.
        //                  horizontal distance = (Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R
        //           (c) compute optimised delay
        //              Dopt = D_Vref - (((b-a)/rho) - ((b-a)/R))
        //                  b = total bytes served in Cx
        //                  a = total bytes served in Cx until the begining of last round
        //                  b = rho_Cx (D_servicepoint - theta_Cx)
        // ----------------------------------------------------------------------------

        //           (a) indentify the last service round of Cx in the duration [0, D_servicepoint]
        int rounds = 1;
        double rho_Cx = beta_Cx.getInitialRateValue(); // fetch residual service rate of Cx

        double bitsServed_Cx = trimDouble((D_servicepoint - theta_Cx)*rho_Cx); // compute the amount of Cx data transmitted in the duration [0, D_servicepoint] :: y1 = 0, y2 = (x2-x1)*slope
        bitsServed_Cx = round(bitsServed_Cx); // To prevent bits in decimal

        double bitsServed_beforeLastRound_Cx = 0; // compute the amount of Cx data transmitted before the begining of the last service round.
        if(bitsServed_Cx > (trimDouble(W_Cx * lMin_Cx))) {
            int n = ceil(trimDouble((bitsServed_Cx - (W_Cx * lMin_Cx))/(W_Cx * lMin_Cx)));
            bitsServed_beforeLastRound_Cx = trimDouble((W_Cx * lMin_Cx)) + trimDouble((n-1)*(W_Cx * lMin_Cx));
            rounds += n;
        }

        double bitsServed_lastRound_Cx = trimDouble(bitsServed_Cx - bitsServed_beforeLastRound_Cx); // compute the amount of Cx data transmitted in the last round

        //           (b) remove the horizontal difference between staircase curve and the rate-latency curve in the last service round.
        //                  horizontal distance = (Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R
        double hDistance = 0.0;
        hDistance = trimDouble(bitsServed_lastRound_Cx * trimDouble((R - rho_Cx)/(R*rho_Cx)));

        //           (c) compute optimised delay
        delay_opt -= hDistance;
        return delay_opt;
    }

    throw std::runtime_error("Unable to compute optimised delay. Invalid node type. :: NC_WRR_Optimised::computeOptimisedNodeDelay()");
}
