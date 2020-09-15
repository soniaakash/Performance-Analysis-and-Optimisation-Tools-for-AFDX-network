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

#include "nc_drr_quantumassignment.h"

NC_DRR_QuantumAssignment::NC_DRR_QuantumAssignment()
{
    ncData = nullptr;
    Q_global = 0;
}

// initialise data and start computation
void NC_DRR_QuantumAssignment::initComputation(const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::computeE2E()");

    // **** define computation environment ****

    if(!NC_DRR::initDRRConfig(dataPtr)) return; // init DRR scheduler

    int initialQ = 10237.3 * 8; // set initial quantum (in bits)

    // set critical class constraints

    // comment the next line to define constraints for the sample network configuration initialised in AFDX::loadConfig_hardcoded()
    qStdOut() << "DEFINE YOUR OWN CRITICAL CLASS CONSTRAINTS :: NC_DRR_QuantumAssignment::initComputation()"; return;

    // ****** sample ******
    //  it belongs to the network example in AFDX::loadConfig_hardcoded()
    setDeadline(1, 1000);
    setDeadline(2, 1000);
    // ****** end of sample ******

    // **** end of define computation environment ****

    qStdOut() << "start QoS tuning!";

    // compute
    computeOptimalQuantumAndItsDistribution(initialQ, dataPtr);

}

// compute the optimal quantum and its distribution
QMap<int, int> NC_DRR_QuantumAssignment::computeOptimalQuantumAndItsDistribution(const int &initialQuantumValue, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::computeOptimalQuantumAndItsDistribution()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::computeOptimalQuantumAndItsDistribution()");

    // optimal quantum distribution belongs to the smallest valid Q_global value : (https://dl.acm.org/doi/10.1145/3356401.3356421)
    //      Q_global is valid if it leads to a valid quantum distribution among each class.
    // the smallest valid Q_global is found at the convergence of Q_global values between valid ones and in valid ones.
    // if the algorithm is unable to converge, the valid Q_global closest to the optimal one is considered as the valid answer.
    //
    // the optimisation is divided into multiple steps
    //     step I: initialise global quantum Q = valInit (in bits) and critical class constraints
    //     step II: compute minimum portion Qx of Q needed by each criticalc class Cx to satisfy the constraint Deadline_Cx
    //     step III: check if Q distribution is valid. i.e. when each criticalc class Cx gets its minimum portion the residual quantum Qresd for non-criticam class Cn is > 0
    //     step IV: check if Q is valid and improvable.
    // ***************************************************************************

    //     step I: initialise global quantum Q = valInit (in bits)
    Q_global = initialQuantumValue; // bits
    double deviation = 0.0001; // acceptable deviation from optimal quantum

    QMap<int, QMap<int,int>> listOfValid_distributions; // result: <Q_global, distribution of Q_global> list of valid global quantum values and their distributions

    QList<int> classes = NC_DRR::getClasses(); // fetch list of class IDs (in ascending order). NOTE : highest class ID is assumed to be non-critical class (Cn)
    if(classes.empty()) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::computeQuantumDistrbution()");
    int Cn = classes.takeLast(); // Cn = non-critical class

    int iterationCount = 0; // for output display only

    // start computation
    while(true) { ++iterationCount;

        //     step II: compute minimum portion Qx of Q_global needed by each criticalc class Cx to satisfy the constraint Deadline_Cx
        QMap<int,int> Cx_minimumQuantum; // <critical class iD, minimum quantum>
        int Qresd = Q_global;

        qStdOut() << "------- ------- Iteration " << iterationCount << " with Q = " <<  Q_global << " bits ("<<trimDouble(Q_global/8.0)<<" bytes)" << " ------- ------- ";
        foreach (const int &cID, classes) { //  for each critical class
            int Q_Cx = computeMinimumQ(cID, getDeadline(cID), dataPtr); // compute minimum portion Qx of Q_global needed by Cx
            Cx_minimumQuantum.insert(cID, Q_Cx); // store result
            Qresd -= Q_Cx; // update residual quantum for non-critical class Cn

            qStdOut() << "C" << cID << " gets " << Q_Cx << " out of " << Q_global << " bits. (in bytes " << Q_Cx/8.0 << " [" << trimDouble((Q_Cx*100.0)/Q_global) << "%]"<< " out of " << Q_global/8.0 << ")";
        }
        qStdOut() << "C" << Cn << " residue " << Qresd << " out of " << Q_global << " bits. (in bytes " << Qresd/8.0 << " out of " << Q_global/8.0 << ")";
        qStdOut() << " ------- -------   -------   ------- ------- ";

        //     step III: check if Q distribution is valid. i.e. when each criticalc class Cx gets its minimum portion the residual quantum Qresd for non-critical class Cn is > 0
        if(Qresd < 0) { // distribution is invalid and the algorithm cannot converge further
            if(!listOfValid_distributions.empty()) { // return the result closest to the optimal one (if any)
                qDebug() << "unable to converge. returning quantum distribution closest to the optimal one!";

                // display results
                qStdOut() << "quantum distribution \"near optimal\" " << listOfValid_distributions.firstKey() << " bits ("<<trimDouble(listOfValid_distributions.firstKey()/8.0)<<" bytes) ";
                foreach (int const &cID, classes) {
                    qStdOut() << "C" << cID << " : " << listOfValid_distributions.first().value(cID) << " bits (" << trimDouble(listOfValid_distributions.first().value(cID)/8.0) << " bytes)";
                }
                qStdOut() << "C" << Cn << " : " << listOfValid_distributions.first().value(Cn) << " bits ("<< trimDouble(listOfValid_distributions.first().value(Cn)/8.0) << " bytes)";

                return listOfValid_distributions.first(); // return the distribtuion corresponding to the smallest of all the computed Q_global
            }
            else throw std::runtime_error("insufficient global quantm. unable to find an optimal distribution. try different Q_global. :: NC_DRR_QuantumAssignment::computeQuantumDistrbution();");
        }

        //     step IV: check if Q is valid and improvable.
        //          Q is invalid if at least one class gets quantum less than its maximum frame length
        //          Q can be improved until at least one class gets quantum equal to its maximum frame length
        double improvementFactor = 1.0/0.0; // smallest quantum value devitation from maximum frame length.

        foreach (int const &cID, classes) { // for each critical class
            double Q_Cx = Cx_minimumQuantum.value(cID); // fetch computed minimum quantum
            double lMax_Cx = cID_lMax_map.value(cID); // fetch maximum frame ength
            double fraction = trimDouble(Q_Cx/lMax_Cx); // compute Qx devitation from lMax_Cx
            if(improvementFactor>fraction) improvementFactor = fraction; // update smallest deviation value
        }
        { // for non-critical class
            double Q_Cn = Qresd;
            double lMax_Cn = cID_lMax_map.value(Cn);
            double fraction = trimDouble(Q_Cn/lMax_Cn); // compute Qn devitation from lMax_Cn
            if(improvementFactor>fraction) improvementFactor = fraction; // update smallest deviation value
        }

        // improve Q
        //          Q can be improved if atleast one of the class gets quantum larger than its maximum frame length
        if(improvementFactor < 1.0) { // Q_global is invalid yet improvable
            Q_global = floor(static_cast<double>(Q_global)/improvementFactor); // update global quantum value
        }
        else if (improvementFactor > (1.0 + deviation)){ // Q_global is valid and improvable

            // store results
            QMap<int,int> result_cID_quantum = Cx_minimumQuantum; // critical classes
            result_cID_quantum.insert(Cn, Qresd); // non-critical class
            listOfValid_distributions.insert(Q_global, result_cID_quantum);

            Q_global = floor(static_cast<double>(Q_global)/improvementFactor); // update global quantum value
        }
        else // Q_global is not improvable.
        {
            // display results
            qStdOut() << "quantum distribution " << Q_global << " bits ("<<trimDouble(Q_global/8.0)<<" bytes) ";
            foreach (int const &cID, classes) {
                qStdOut() << "C" << cID << " : " << Cx_minimumQuantum.value(cID) << " bits (" << trimDouble(Cx_minimumQuantum.value(cID)/8.0) << " bytes)";
            }
            qStdOut() << "C" << Cn << " : " << Qresd << " bits ("<< trimDouble(Qresd/8.0) << " bytes)";

            // store results
            QMap<int,int> result_cID_quantum = Cx_minimumQuantum; // critical classes
            result_cID_quantum.insert(Cn, Qresd); // non-critical class

            // return optimal quantum distribution
            return result_cID_quantum;
        }
    }
}

// step II: compute minimum portion Qx of Q_global needed by each criticalc class Cx to satisfy the constraint Deadline_Cx
int NC_DRR_QuantumAssignment::computeMinimumQ(const int &classID, const double &deadline, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::computeMinimumQ()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::computeMinimumQ()");

    // the minimum Q_Cx out of Q_global required to respect Cx deadline is computed in multiple steps:
    //     (1) compute maximum dealy D_max_Cx in Cx with the given Q_Cx
    //     (2) adjust Q_Cx
    //          if D_max_Cx is
    //              > Deadline_Cx then increase Q_Cx
    //              < Deadline_Cx then decrease Q_Cx
    //              = Deadline_Cx then stop
    //          if Q_Cx is converged and
    //              D_max_Cx > Deadline_Cx : Q_global is insufficient (either too big or too small) to repect the constraint
    //              D_max_Cx <<< Deadline_Cx : traffic in Cx is too low to reach the deadline. (result can be very far from the optimal value)


    int res_Q_Cx = Q_global; // result : minimum quantum required by Cx to respect deadline
    double res_deviation = -1; // result : D_max_Cx deviation (in %) with respect to deadline

    // set quantum search space
    int Q_Cx = Q_global;
    int up = Q_Cx;
    int down = cID_lMax_map.value(classID);


    while(true)
    {
        cID_quantum_map.insert(classID,Q_Cx); // update DRR scheduler

        //     (1) compute maximum dealy D_max_Cx in Cx with the given Q_Cx
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        double maxE2E = NC_Classic::computeMaximumE2E(this, &ncData_temp, classID); // compute maximum delay in Cx
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // check deviation with respect to deadline
        //      if deviation of maxDelay from deadline is higher than 10 %.
        //      the result can be very far from the optimal value
        double deviation = trimDouble((deadline - maxE2E)*100.0/deadline);


        //     (2) adjust Q_Cx
        if(maxE2E > deadline) {
            // Q_Cx must increase
            down = Q_Cx; // update search space
        }
        else if(maxE2E < deadline) {
            // Q_Cx must decrease
            up = Q_Cx; // update search space

            // acceptable value
            res_Q_Cx = Q_Cx;
            res_deviation = deviation;
        }
        else // maxE2E == deadline
        {
            // acceptable value
            res_Q_Cx = Q_Cx;
            res_deviation = deviation;
            break;
        }

        int prevQ_Cx = Q_Cx;
        Q_Cx = floor((up+down)/2); // compute next quantum value within the search space. caution : computation in bits !
        if(prevQ_Cx == Q_Cx) break; // Q_Cx converged
    }

    // check validity of the result
    if(res_deviation > 10) { // result valid but too far from optimal
        qStdOut() << "traffic in C" << classID << " is too low. " << res_deviation << " % deviation from deadline. quantum distribution may be very far from optimal";
    }
    else if(res_deviation < 0) { // result invalid
        throw std::runtime_error("global quantum is insufficient (either too big or too small) :: NC_DRR_QuantumAssignment::computeQuantumDistrbution();");
    }
    else // result valid
        qStdOut() << "C" << classID << " : deviation " << res_deviation << " %";

    return res_Q_Cx;
}

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************     NC_Classic   ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //
//
// NC_Classic is the re-implementation of NC_DRR_Classic adapted for the quantum assignment algorithm.

// compute worst-case end-to-end delay for all flows of the given class on all paths in the network.
double NC_DRR_QuantumAssignment::NC_Classic::computeMaximumE2E(NC_DRR_QuantumAssignment *parent, NCData *ncData, const int &classID)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::NC_Classic::computeMaximumE2E()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::NC_Classic::computeMaximumE2E()");

    // the E2E delay computation is divided into multiple steps
    //
    // Step 1 : select a flow (Vref)
    // Step 2 : select a path (Pi_Vref)
    // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
    //        Note: in DRR scheduling, computation is "per class basis"
    //  Step 3a: compute overall arrival curve (alpha_o)
    //  Step 3b: compute service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref
    // Step 4 : compute e2e delay.

    double maxE2Edelay = 0.0; // result : maximum end-to-end delay in Cx

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)

        if(parent->getClassID(flow.getFlowID()) != classID) continue; // skip classes other than Cx

        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute worst-case delay at each node in the path
                double delay = NC_Classic::computeNodeDelay(parent, ncData, flow, node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

                e2eDelay += delay;                       // Step 4 : compute e2e delay.

                qStdOut() << flow.getFlowID() << " NODE " << NODE_t_toString(node) << " delay " << delay << " microSec";
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // update result
            if(maxE2Edelay < e2eDelay) maxE2Edelay = e2eDelay;
        }
    }

    return maxE2Edelay;
}

// Step 3 : compute worst-case delay for refFLow at the given node
double NC_DRR_QuantumAssignment::NC_Classic::computeNodeDelay(NC_DRR_QuantumAssignment *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::NC_Classic::computeNodeDelay()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::NC_Classic::computeNodeDelay()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode.getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
    //        Note: in DRR scheduling, computation is "per class basis"
    //  Step 3a: compute overall arrival curve (alpha_o)
    //  Step 3b: compute service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref

    ConcaveCurve alpha_o = NC_Classic::computeOverallArrivalCurve(parent, ncData, refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = NC_Classic::computeServiceCurve(parent, ncData, refFlow, node);            //  Step 3b: compute service curve (beta_Cx)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode.addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o)
ConcaveCurve NC_DRR_QuantumAssignment::NC_Classic::computeOverallArrivalCurve(NC_DRR_QuantumAssignment *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::NC_Classic::computeOverallArrivalCurve()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::NC_Classic::computeOverallArrivalCurve()");
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
    const int Cx = parent->getClassID(refFlow.getFlowID());
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Cx at this output port
        if(ncNode.getNodeType() == NODE_TYPE::SW){
            if(parent->getClassID(fID) != Cx) continue;
        }
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        double burst = flow.getLMax(); // burst = (max frame size)
        double BAG = flow.getBAG();
        double rate = trimDouble(flow.getLMax()/BAG); // rate = (max frame size)/BAG
        double jitter = computeJitter(parent, ncData, flow, node);
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
            } else throw std::runtime_error("arrival curve not found :: NC_DRR_QuantumAssignment::NC_Classic::computeOverallArrivalCurve()");
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
            if(parent->getClassID(fID) != Cx) continue;
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
                } else throw std::runtime_error("arrival curve not found :: NC_DRR_QuantumAssignment::NC_Classic::computeOverallArrivalCurve()");
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
double NC_DRR_QuantumAssignment::NC_Classic::computeJitter(NC_DRR_QuantumAssignment *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::NC_Classic::computeJitter()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::NC_Classic::computeJitter()");

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
        sum_Dmax += computeNodeDelay(parent, ncData, refFlow, prevNode); // make sum

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

//  Step 3b: compute service curve (beta_Vref)
ConvexCurve NC_DRR_QuantumAssignment::NC_Classic::computeServiceCurve(NC_DRR_QuantumAssignment *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignment::NC_Classic::computeServiceCurve()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignment::NC_Classic::computeServiceCurve()");
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
    if(ncNode.getNodeType() == NODE_TYPE::SW) {// qStdOut() << "new service curve " << NODE_t_toString(node) << " Q " << Q_global << " Qx " << cID_quantum_map.value(getClassID(refFlow.getFlowID()));;
        //      (1) compute residual service rate rho_Cx
        //              rho_Cx = (quantum_Cx / sum of quanta of all the classes) linkRate
        //  **                 = (quantum_Cx / Q_global) linkRate
        const int Cx = parent->getClassID(refFlow.getFlowID());
        double Q_Cx = parent->cID_quantum_map.value(Cx);
        double Q_sum = parent->Q_global;
        double R = ncNode.getLinkRate(node.second);
        double rho_Cx = trimDouble(trimDouble(Q_Cx/Q_sum) * R);

        //      (2) compute scheduler latency
        //          scheduler latency is divided into two parts: (https://ieeexplore.ieee.org/document/8603222)
        //          (a) X_Cx = sum(Q_Ci + maximum deficit of Ci) / R       (Caution :  Ci != Cx)
        // **                = sum(Q_Ci)/R + sum(maximum deficit of Ci)/R
        // **                = (Q_global - Q_Cx)/R + sum(maximum deficit of Ci)/R
        //                  maximum deficit of Ci = (max frame size in bytes) - 1 (in bytes) = (max frame size in bits) - 8 (in bits)
        //          (b) Y_Cx = ((Q_Cx - max deficit of Cx) + sum(Q_Ci))/R - (Q_Cx - max deficit of Cx)/rho_Cx
        // **                = ((Q_Cx - max deficit of Cx) + (Q_global - Q_Cx))/R - (Q_Cx - max deficit of Cx)/rho_Cx
        //            theta_Cx = X_Cx + Y_Cx
        double X_Cx = 0.0;
        X_Cx += trimDouble((Q_sum - Q_Cx)/R);
        foreach (int const&Ci, parent->cID_quantum_map.keys()) {
            if(Ci == Cx) continue;
            double lMax_Ci = parent->cID_lMax_map.value(Ci); // in bits
            X_Cx += trimDouble((lMax_Ci - 8)/R);
        }

        double Y_Cx = 0.0;
        double lMax_Cx = parent->cID_lMax_map.value(Cx); // in bits
        Y_Cx += trimDouble((Q_Cx - (lMax_Cx - 8))/R);
        Y_Cx += trimDouble((Q_sum - Q_Cx)/R);
        Y_Cx -= trimDouble((Q_Cx - (lMax_Cx - 8))/rho_Cx);

        //            theta_Cx = X_Cx + Y_Cx
        double theta_Cx = X_Cx + Y_Cx;

        //      (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta((sl+theta_Cx),rho_Cx);
        return beta;
    }

    throw std::runtime_error("Unable to compute service curve. Invalid node type. :: NC_DRR_QuantumAssignment::NC_Classic::computeServiceCurve()");
}
