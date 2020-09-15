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
#include "NetworkCalculus/FIFO/nc_fifo_offset.h"

NC_FIFO_OFFSET::NC_FIFO_OFFSET()
{
    ncData = nullptr;
}

// compute worst-case end-to-end delay for all flows on all paths in the network
void NC_FIFO_OFFSET::computeE2E(NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeE2E()");

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
                qStdOut() << flow.getFlowID() << " node " << NODE_t_toString(node) << " delay " << delay;

                e2eDelay += delay;                       // Step 4 : compute e2e delay.
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // store result
            flowID_pathNo_e2eDelays.insert(flow.getFlowID(), qMakePair(pNo,e2eDelay));
        }

    }
}

// Step 3 : compute worst-case delay at node
double NC_FIFO_OFFSET::computeNodeDelay(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeNodeDelay()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode.getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    //  Step 3a: compute overall arrival curve (alpha_o)
    //      alpha_o computation is divided into multiple parts:
    //      (1) compute arival curve of each flow arriving at this output port.
    //          it also includes "jitter" integration.
    //          Note: jitter computation require delay at previous nodes (which require recursive computations)
    //      (2) compute aggregated arival curves at this output port.
    //          it includes "offset" integration.
    //      (3) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //  Step 3b: compute service curve (beta_Vref)
    //       beta_Vref computation is straight forward in FIFO. beta = R[t - sl]+
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    ConcaveCurve alpha_o = computeOverallArrivalCurve(refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = computeServiceCurve(refFlow, node);            //  Step 3b: compute service curve (beta_Vref)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode.addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o)
ConcaveCurve NC_FIFO_OFFSET::computeOverallArrivalCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeOverallArrivalCurve()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    ConcaveCurve alpha_o; // overall arrival curve
    // check if the overall arrival curve for the refFlow is already computed
    if(ncNode.getComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o)) {
        return alpha_o;
    }


    //  Step 3a: compute overall arrival curve (alpha_o)
    //      alpha_o computation is divided into multiple parts:
    //      (1) compute arival curve of each flow arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve for flows arriving from shared input node.
    //      (2a) sort flows based on input nodes
    //      (2b) compute aggregated arival curves for selected input.
    //          it includes "offset" integration.
    //      (2c) compute cumulative curve from all the aggregated arival curves.
    //          it includes "serialisation" integration.
    //      (3) alpha_o is the sum of all the cumulative curves from different inputs.
    // ----------------------------------------------------------------------------

    //      (1) compute arival curve of each flow arriving at this output port.
    //                  jitter integration increases the burst (https://ieeexplore-ieee-org.gorgone.univ-toulouse.fr/document/5524098)
    //              alpha = rate*(t + jitter) + burst
    //                  burst = (max frame size)
    //                  rate = (max frame size)/BAG
    //                  increased burst = (rate * jitter) + burst
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow at this output port
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        double burst = flow.getLMax(); // burst = (max frame size)
        double BAG = flow.getBAG();
        double rate = trimDouble(flow.getLMax()/BAG); // rate = (max frame size)/BAG
        double jitter = computeJitter(flow, node);
        burst = trimDouble(rate*jitter) + burst; // increased burst = (rate * jitter) + burst
        ConcaveCurve alpha(burst, rate);
        ncNode.addComputedArrivalCurve(node.second, fID, alpha);
    }

    //      (2) compute cumulative curve for flows arriving from shared input node.
    QMap<NODE_t,ConcaveCurve> inputNode_alpha_map; // <inputNode, cumulative arrival curve>
    {
        //      (2a) sort flows based on input nodes
        QMultiMap<NODE_t, int> inputNode_fID_map; // <inputNode, list of flowID>
        foreach (int const&fID, ncNode.getFlowList(node.second)) {
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            NODE_t inputNode = flow.getPreviousNode(node); // fetch input node
            inputNode_fID_map.insert(inputNode, fID); // group fIDs based on input node
        }

        foreach (NODE_t const &inputNode, inputNode_fID_map.uniqueKeys()) { // from each input
            QList<int> fID_fromSelectedInputNode = inputNode_fID_map.values(inputNode); // list of flows from the selected input node

            //      (2b) compute aggregated arival curves for selected input.
            //              based on the flow scheduling at end-systems (cite :: ... )
            //                  the flows emitted by the same end-system are temproally separated and cannot create a sum of burst at an output port (h).
            //              (i) the flows emitted by the same end-system (es_x) are represented by a unique subset (SS_es_x).
            //              (ii) the flows temporally separated at their source end-system (es_x) are aggregated into a single flow with respect to a benchmark flow (v_benchmark).
            //                      - each flow at es_x can be v_benchmark. this gives as many aggregated flows as there are flows arriving from es_x at h.
            //                      - each aggregated flow is represented by an aggregated arrival curve (alpha_aggr_v_benchmark).
            //              (iii) maximum of all the alpha_aggr_v_benchmark is the upper bound representing the aggregated arrival curve (alpha_SS_es_x) of the subset (SS_es_x).

            //              (i) the flows emitted by the same end-system are represented by a unique subset
            QMultiMap<NODE_t, int> sourceNode_fID_map; // <sourceNode, flowID> :: sourceNode (es_x) == subset (SS_es_x)
            {
                foreach (int const&fID, fID_fromSelectedInputNode) { // get the sourceNode of each flow arriving from the selected input node
                    NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
                    NODE_t sourceNode = node; // starting from present output port
                    NODE_t prevNode = flow.getPreviousNode(node); // fetch previous node

                    while(!NODE_generic::isVirtalES(prevNode)) // until the beginning of the flow path. virtual node == end of path.
                    {
                        sourceNode = prevNode;
                        prevNode = flow.getPreviousNode(prevNode); // fetch previous node
                    }
                    sourceNode_fID_map.insert(sourceNode, fID);
                }
            }

            QMultiMap<NODE_t, ConcaveCurve> sourceNode_alpha_map; // <sourceNode, aggregated arrival curve of the subset>

            //              (ii) the flows temporally separated at their source end-system are aggregated into a single flow with respect to a benchmark flow (v_benchmark).
            //                      - each flow at es_x can be v_benchmark. this gives as many aggregated flows as there are flows arriving from es_x at h.
            //                      - each aggregated flow is represented by an aggregated arrival curve (alpha_aggr_v_benchmark).
            //                  an aggregated arrival curve is the sum of arrival curves of all the flows shifted right by their respective "relative offset" values
            foreach (NODE_t const &sourceNode, sourceNode_fID_map.uniqueKeys()) { // for each subset (belonging to a source node)
                QList<ConcaveCurve> alpha_aggr_list; // list of aggregated arrival curves in this subset
                foreach (int const &fID_benchmark, sourceNode_fID_map.values(sourceNode)) { // consider each flow as a benchmark once

                    // check if the aggregated arrival curve with respect to the selected bechmark is already computed at this output port
                    {
                        QMap<NODE_t, ComputedAggregatedCurves>::iterator it = node_computedAggregatedCurves.find(node);
                        if(it != node_computedAggregatedCurves.end()) { // at the selected node
                            ConcaveCurve alpha_aggr = it->fID_aggregatedCurve.value(fID_benchmark, ConcaveCurve()); // fetch aggregated arrival curve of the benchmarkFlow
                            if(!alpha_aggr.isBaseCurve()) { alpha_aggr_list.append(alpha_aggr); continue; }
                        }
                    }

                    NCData::NCFlow const &flow_benchmark = ncData->selectNCFlow_ref(fID_benchmark); // fetch flow with selected fID
                    QMap<int, double> fID_offset_map = computeRelatveOffsets(flow_benchmark, node); // <flowID, relativeOffset> :: compute relative offsets with respect to fID_benchmark

                    //  an aggregated arrival curve is the sum of arrival curves of all the flows shifted right by their respective "relative offset" values
                    ConcaveCurve alpha_aggr;
                    {
                        QList<ConcaveCurve> alpha_withOffset; // list of arroval curves shifted right by the relative offset values
                        foreach (int const& fID, sourceNode_fID_map.values(sourceNode)) {
                            ConcaveCurve alpha;
                            if(ncNode.getComputedArrivalCurve(node.second, fID, alpha)) { // get arrival curve
                                alpha.setOffset(fID_offset_map.value(fID)); // introduce offset in arrival curve
                                alpha_withOffset.append(alpha);
                            } else throw std::runtime_error("arrival curve not found :: NC_FIFO_OFFSET::computeOverallArrivalCurve()");
                        }
                        alpha_aggr.makeAggregatedArrivalCurve(alpha_withOffset); // NOTE : this curve is not concave
                    }

                    // store aggregated arrival curve
                    {
                        QMap<NODE_t, ComputedAggregatedCurves>::iterator it = node_computedAggregatedCurves.find(node);
                        if(it == node_computedAggregatedCurves.end()) it = node_computedAggregatedCurves.insert(node, ComputedAggregatedCurves());
                        it->fID_aggregatedCurve.insert(fID_benchmark, alpha_aggr);
                    }

                    alpha_aggr_list.append(alpha_aggr);
                }

                if(sourceNode_fID_map.values(sourceNode).contains(refFlow.getFlowID())) { // refFlow is the benchmark
                    alpha_aggr_list.clear();
                    QMap<NODE_t, ComputedAggregatedCurves>::iterator it = node_computedAggregatedCurves.find(node);
                    alpha_aggr_list.append(it->fID_aggregatedCurve.value(refFlow.getFlowID()));
                }

                //          (iii) maximum of all the alpha_aggr_v_benchmark is the upper bound representing the aggregated arrival curve (alpha_SS_es_x) of the subset (SS_es_x).
                ConcaveCurve alpha_aggr_subset = maxAggrCurve(alpha_aggr_list);
                sourceNode_alpha_map.insert(sourceNode, alpha_aggr_subset); // store aggregated arrival curve
            }

            //      (2c) compute cumulative curve (alpha_grp_input) from all the aggregated arival curves.
            //              based on serialisation effect: (https://hal.archives-ouvertes.fr/hal-02270458)
            //                  the flows sharing the same input port are serialised and cannot create a sum of burst at an output port.
            //              (a) all the subsets of flows sharing the selected input port are represented by a group SS_grp_input.
            //              (b) the burst due to SS_grp_input is limited to maximum burst among the subsets. : maxBurst_SS_group_input
            //              (c) the arrival rate is limited by the link rate : R
            //              (d) the arrival curve of SS_grp_input is
            //                  alpha_grp_input = min(arrival_curve(rate = R, burst = maxBurst_SS_group_input), sum(arrival curves of the subsets in SS_grp_input))

            //          (a) sourceNode_alpha_map == SS_grp_input
            //          (b) the burst due to V_grp_input is limited to maximum burst in a group. : maxBurst_SS_group_input
            double maxBurst = 0.0;
            ConcaveCurve alpha_sum; // sum(arrival curves of the flows in SS_grp_input)

            foreach (NODE_t const &sourceNode, sourceNode_alpha_map.uniqueKeys()) { // for each subset (belongs to a source node)
                ConcaveCurve const &alpha = sourceNode_alpha_map.value(sourceNode); // get arrival curve
                if(alpha_sum.isBaseCurve()) alpha_sum = alpha;
                else alpha_sum += alpha; // make sum

                double burst = alpha.getBurstValue(); // get burst value of selected subset
                if(maxBurst < burst) maxBurst = burst; // get maxBurst
            }

            //          (c) the arrival rate is limited by the link rate : R
            double R = ncNode.getLinkRate(node.second);

            //          (d) the arrival curve of group is alpha_grp_input = min(alpha(maxBurst, R), alpha_sum)
            ConcaveCurve alpha(maxBurst, R);
            inputNode_alpha_map.insert(inputNode, min(alpha,alpha_sum));
        }
    }

    //      (3) alpha_o is the sum of all the cumulative curves from different inputs.
    //                  alpha_o = sum(alpha_grp_input1, alpha_grp_input2, ...)
    foreach (ConcaveCurve const&alpha_input, inputNode_alpha_map.values()) {
        if(alpha_o.isBaseCurve()) alpha_o = alpha_input;
        else alpha_o += alpha_input; // make sum
    }

    // store result
    ncNode.addComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o);
    return alpha_o;
}

// compute the jitter upper bound for refFlow at the given node
double NC_FIFO_OFFSET::computeJitter(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeJitter()");;
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

//  Step 3b: compute service curve (beta_Vref)
ConvexCurve NC_FIFO_OFFSET::computeServiceCurve(const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeServiceCurve()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);
    // QList<NCData::NCNode>::iterator ncNode = ncData->selectNCNode(node.first);
    Q_UNUSED(refFlow) // to depricate warning

    //  Step 3b: compute service curve (beta_Vref)
    //       beta_Vref computation is straight forward in FIFO.
    //       beta = R[t - sl]+

    double R = ncNode.getLinkRate(node.second);
    double sl = ncNode.getSwitchingLatnecy();
    ConvexCurve beta(sl,R);
    return beta;
}

ConcaveCurve NC_FIFO_OFFSET::maxAggrCurve(const QList<ConcaveCurve> &aggrCurves)
{
    ConcaveCurve resConc;

    if(aggrCurves.empty()) throw std::runtime_error("empty list of arrival curves. unable to compute max!");

    if(aggrCurves.size() == 1) return aggrCurves.first();

    RayList res = aggrCurves.at(0).getRayList();

    // compute piecewise linear maximum of the given list of curves
    for(int i = 1; i < aggrCurves.size(); ++i) {
        RayList r1 = res;
        RayList const &r2 = aggrCurves.at(i).getRayList();
        maxRayList(r1, r2, res);
    }

    makeConcave(res);

    // makeContinuous(res); // suggestion

    resConc.setRayList(res);
    return resConc;
}

QMap<int, double> NC_FIFO_OFFSET::computeRelatveOffsets(const NCData::NCFlow &benchmarkFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeRelatveOffsets()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //   relative offsets with respect to a benchmark flow (v_b) at a node h is computed as: (cite::...)
    //      (a) compute the worst-case delay for v_b till the previous node in the path.
    //             D_max_till_(h-1) = sum(D_(endSystem), D_(switch_1), ... D_(switch_(h-1)))
    //      (b) compute the minimum delay for other flows v_i till the previous node in the path
    //             D_min_till_(h-1) = sum(D_min_(endSystem), D_min_(switch_1), ... D_min_(switch_(h-1)))
    //                  minimum delay for any flow at any node is equal to its transmission time + switching latnecy
    //      (c) compute the relative offsets (O_r_b_i_es) with respect to v_b at the source end-system of v_b.
    //      (d) compute relative offsets at h
    //             O_r_b_i_h = O_r_b_i_es - ( D_max_till_(h-1) - D_min_till_(h-1) )
    //  ---------------------------------------------------------------------------------------------

    QMap<int, double> O_r_b_i_h; // result : <flowID, relative offset with respect to benchmarkFlow>

    // check if the relative offsets with respect to benchmarkFlow are already computed.
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(node);
        if(it != node_computedOffsets.end()) { // at the selected node
            O_r_b_i_h = it->fID_relativeOffsets.value(benchmarkFlow.getFlowID(), QMap<int, double>()); // fetch relative offsets for benchmarkFlow
            if(!O_r_b_i_h.empty()) return O_r_b_i_h;
        }
    }

    O_r_b_i_h.insert(benchmarkFlow.getFlowID(), 0.0); // benchmarkFlow offset relative to itself is 0

    //      (a) compute the worst-case delay for v_b till the previous node in the path.
    double sum_Dmax = 0.0;
    NODE_t sourceNode = node;
    {
        NODE_t prevNode = benchmarkFlow.getPreviousNode(node); // fetch previous node
        while(!NODE_generic::isVirtalES(prevNode)) // virtual node == end of path
        {
            sum_Dmax += computeNodeDelay(benchmarkFlow, prevNode); // make sum
            sourceNode = prevNode; // get source node

            prevNode = benchmarkFlow.getPreviousNode(prevNode); // fetch previous node
        }
    }

    //      (c) compute the relative offsets (O_r_b_i_es) with respect to v_b at the source end-system of v_b.
    QMap<int, double> O_r_b_i_es = computeRelatveOffsets_atSource(benchmarkFlow, sourceNode);

    // identify the flows sharing the path with v_b from sourceNode till h
    QList<int> vi_list; // list of flows sharing the path with v_b till h
    {
        NCData::NCNode &ncNode_sourceNode = ncData->selectNCNode_ref(sourceNode.first);
        QList<int> const &flowsAtSourceNode = ncNode_sourceNode.getFlowList(sourceNode.second); // fetch list of flows at source node
        QList<int> const &flowsAtThisNode = ncNode.getFlowList(node.second); // fetch list of flows at this node (h)
        vi_list = flowsAtThisNode.toSet().intersect(flowsAtSourceNode.toSet()).toList(); // fetch common flows at sourceNode and at h.
        vi_list.removeOne(benchmarkFlow.getFlowID()); // remove v_b from the list
    }

    //      (d) compute relative offsets at h
    foreach (int const &vi, vi_list) { // for each flow vi
        NCData::NCFlow const &flow_vi = ncData->selectNCFlow_ref(vi); // fetch flow with selected fID

        //      (b) compute the minimum delay for v_i till the previous node in the path
        double sum_Dmin = 0.0;
        NODE_t prevNode = flow_vi.getPreviousNode(node); // fetch previous node
        while(!NODE_generic::isVirtalES(prevNode)) // virtual node == end of path
        {
            // minimum delay for any flow at any node is equal to its transmission time (tr) + switching latnecy (sl)
            NCData::NCNode &prev_ncNode = ncData->selectNCNode_ref(prevNode.first);
            double sl = prev_ncNode.getSwitchingLatnecy();
            double R = prev_ncNode.getLinkRate(prevNode.second);
            double tr = trimDouble(flow_vi.getLMax()/R); // max transmission time = (max frame length) / link rate
            sum_Dmin += (tr + sl); // make sum

            prevNode = flow_vi.getPreviousNode(prevNode); // fetch previous node
        }

        //      (d) compute relative offsets at h
        //             O_r_b_i_h = O_r_b_i_es - ( D_max_till_(h-1) - D_min_till_(h-1) )
        double offset = O_r_b_i_es.value(vi) - (sum_Dmax - sum_Dmin);

        O_r_b_i_h.insert(vi, offset); // store result
    }

    // store result
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(node);
        if(it == node_computedOffsets.end()) it = node_computedOffsets.insert(node, ComputedOffsets());
        it->fID_relativeOffsets.insert(benchmarkFlow.getFlowID(), O_r_b_i_h);
    }

    return O_r_b_i_h;
    // throw std::runtime_error("Compute relative offset here :: computeRelatveOffsets()"); // integrate offset in the arrival curve
}

QMap<int, double> NC_FIFO_OFFSET::computeRelatveOffsets_atSource(const NCData::NCFlow &benchmarkFlow, const NODE_t &sourceNode)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeRelatveOffsets_atSource()");;
    // NCData::NCNode &ncNode = ncData->selectNCNode_ref(sourceNode.first);

    //   relative offsets with respect to a benchmark flow (v_b) at source end-system (es) is computed as:
    //      (a) compute definite offset (O_d_i) for each flow v_i at the end-system
    //      (b) compute relative offset at the end-system
    //              O_r_b_i_es = (O_d_i + x BAG_vi) - O_d_b,  when O_d_b > O_d_i, where x = floor(O_d_b/BAG_vi) + 1
    //                         = O_d_i - (O_d_b + y BAG_vb),  when O_d_i > O_d_b, where y = floor(O_d_i/BAG_vb) - 1
    //              O_r_b_b_es = 0,                                O_d_i == O_d_b
    //  ---------------------------------------------------------------------------------------------

    if(!NODE_generic::isVirtalES(benchmarkFlow.getPreviousNode(sourceNode))) throw std::runtime_error(NODE_t_toString(sourceNode).toStdString() + "cannot be a sourceNode for the given benchmark flow! :: NC_FIFO_OFFSET::computeRelatveOffsets_atSource()");

    QMap<int, double> O_r_b_i_es; // result : <flowID, relative offset with respect to benchmarkFlow>

    // check if the relative offsets with respect to benchmarkFlow are already computed.
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(sourceNode);
        if(it != node_computedOffsets.end()) { // at the selected source node
            O_r_b_i_es = it->fID_relativeOffsets.value(benchmarkFlow.getFlowID(), QMap<int, double>()); // fetch relative offsets for benchmarkFlow
            if(!O_r_b_i_es.empty()) return O_r_b_i_es;
        }
    }

    //      (a) compute definite offset (O_d_i) for each flow v_i at the end-system
    QMap<int, double> O_d_i_list = computeDefiniteOffset(sourceNode);

    //      (b) compute relative offset at the end-system
    double O_d_b = O_d_i_list.value(benchmarkFlow.getFlowID()); // fetch definite offset of the benchmarkFlow
    foreach (int const& vi, O_d_i_list.uniqueKeys()) {
        if(vi == benchmarkFlow.getFlowID()) {O_r_b_i_es.insert(vi, 0.0); continue; } // benchmark flow has 0 offset

        double O_d_i = O_d_i_list.value(vi); // fetch definite offset of vi
        if(O_d_b > O_d_i) // O_r_b_i_es = (O_d_i + x BAG_vi) - O_d_b,  when O_d_b > O_d_i, where x = floor(O_d_b/BAG_vi) + 1
        {
            NCData::NCFlow const &flow_vi = ncData->selectNCFlow_ref(vi); // fetch flow with selected fID
            double BAG_vi = flow_vi.getBAG();
            double x = floor(trimDouble(O_d_b/BAG_vi)) + 1;
            O_r_b_i_es.insert(vi, trimDouble(O_d_i + trimDouble(x * BAG_vi) - O_d_b));
        }
        else if (O_d_b < O_d_i) // O_r_b_i_es = O_d_i - (O_d_b + y BAG_vb),  when O_d_i > O_d_b, where y = floor(O_d_i/BAG_vb) - 1
        {
            double BAG_vb = benchmarkFlow.getBAG();
            double y = floor(trimDouble(O_d_i/BAG_vb)) - 1;
            if(y < 0)
                O_r_b_i_es.insert(vi, O_d_i);
            else
                O_r_b_i_es.insert(vi, trimDouble(O_d_i -  (O_d_b + trimDouble(y * BAG_vb))));
        }
        else
        {
            throw std::runtime_error("Invalid difinite offset O_d_b == O_d_i :: NC_FIFO_OFFSET::computeRelatveOffsets_atSource()");;
        }
    }

    // store result
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(sourceNode);
        if(it == node_computedOffsets.end()) it = node_computedOffsets.insert(sourceNode, ComputedOffsets());
        it->fID_relativeOffsets.insert(benchmarkFlow.getFlowID(), O_r_b_i_es);
    }

    return O_r_b_i_es;
}

QMap<int, double> NC_FIFO_OFFSET::computeDefiniteOffset(const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_FIFO_OFFSET::computeRelatveOffsets_atSource()");;
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //   defintie offsets at source node are computed as: (cite::...)
    //      (a) arrange flows in ascending order of BAG values
    //      (b) compute granularity G
    //      (c) generate initial frame release sequence seq[time] = frame using the first flow (v_i).
    //          - set time reference map limits [0, BAGmax), with steps = G
    //          - set deffinite offset of v_i to O_d_i = 0
    //          - fill the frame release sequence seq[O_d_i + n BAGi] = v_first, where n belongs to the natural numbers including zero
    //      (d) generate frame release sequence seq[time] = frame for the remaining flows v_j
    //          - look for the longest least loaded interval in [0, BAGj), with steps = G
    //              get start Bj and end Ej of least loaded interval
    //          - set definite offset O_d_j = Bj + (Ej - Bj)/2 - ( [(Ej - Bj)/2] % G )
    //          - fill the frame release sequence seq[O_d_j + n BAGj] = v_j, where n belongs to Natural numbers including zero
    //
    //  * a srcNode can be either an ES or a GATEWAY switch


    QMap<int, double> O_d_i; // result : <flowID, definite offset>

    // check if the definite offsets are already computed.
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(node);
        if(it != node_computedOffsets.end()) { // at the selected source node
            O_d_i = it->fID_definiteOffsets; // fetch definite offsets
            if(!O_d_i.empty()) return O_d_i;
        }
    }

    //      (a) arrange flows in ascending order of BAG values
    QMultiMap<double,int> BAG_fID_map; // <BAG, flowID> :: BAG and flows map in assending order of BAG values
    {
        QList<int> const &flows = ncNode.getFlowList(node.second); // fetch list of flows at source node
        foreach (int const &fID, flows) {
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            BAG_fID_map.insert(flow.getBAG(), fID);
        }
    }
    QList<int> flows = BAG_fID_map.values(); //      (a) list of flows in ascending order of BAG values

    //      (b) compute granularity G
    double G = computeGranularity(BAG_fID_map.keys());
    double seq_timeUpperLimit = BAG_fID_map.lastKey(); // set time reference map limits [0, BAGmax)

    //      (c) generate initial frame release sequence seq[time] = frame using the first flow (v_i).
    QMap<double, int> seq; // <frame release time, flowID> :: frame release sequence
    {
        double offset = 0; // set deffinite offset of v_i to = 0

        // fill the frame release sequence seq[O_d_i + n BAGi] = v_first, where n belongs to the natural numbers including zero
        double BAG = BAG_fID_map.firstKey();
        int fID = BAG_fID_map.take(BAG);
        double time = 0.0;
        for(int n = 0; (time = (offset + (n * BAG))) < seq_timeUpperLimit; ++n){ seq.insert(time, fID); }

        O_d_i.insert(fID, offset); // store result
    }

    //      (d) generate frame release sequence seq[time] = frame for the remaining flows v_j
    //          - look for the longest least loaded interval in [0, BAGj), with steps = G
    //          - set definite offset O_d_j = Bj + (Ej - Bj)/2 - ( [(Ej - Bj)/2] % G )
    //          - fill the frame release sequence seq[O_d_j + n BAGj] = v_j, where n belongs to Natural numbers including zero
    while(!BAG_fID_map.empty()) {
        // fetch next fID and its BAG value
        double BAG = BAG_fID_map.firstKey();
        int fID = BAG_fID_map.take(BAG);

        //          - look for the longest least loaded interval in [0, BAGj), with steps = G
        double begin = 0; double end = -1;
        for(int possibleReleaseTime = 0; possibleReleaseTime < BAG; possibleReleaseTime += G) {

            // skip the time slot already taken by other flows
            if(seq.contains(possibleReleaseTime)) { continue; }

            // mark the available iterval
            double begin_temp = possibleReleaseTime; double end_temp = possibleReleaseTime + G;
            {
                // check if end_temp can be farther away
                auto it = seq.find(possibleReleaseTime - G); // fetch last occupied slot
                if((it != seq.end()) && (++it != seq.end())) { // fetch next occupied slot
                    end_temp = it.key(); // mark the end of available interval
                }
            }

            if( (end - begin) < (end_temp - begin_temp)) { end = end_temp, begin = begin_temp; } // update largest avaiable interval
        }

        //          - set definite offset O_d_j = Bj + (Ej - Bj)/2 - ( [(Ej - Bj)/2] % G )
        double offset = trimDouble(begin + floor((end - begin)/2) - (static_cast<int>(floor((end - begin)/2)) % static_cast<int>(G)));

        //          - fill the frame release sequence seq[O_d_j + n BAGj] = v_j, where n belongs to Natural numbers including zero
        double time = 0.0;
        for(int n = 0; (time = (offset + (n * BAG))) < seq_timeUpperLimit; ++n){ seq.insert(time, fID); }

        O_d_i.insert(fID, offset); // store result
    }


    // store result
    {
        QMap<NODE_t, ComputedOffsets>::iterator it = node_computedOffsets.find(node);
        if(it == node_computedOffsets.end()) it = node_computedOffsets.insert(node, ComputedOffsets());
        it->fID_definiteOffsets = O_d_i;
    }

    return O_d_i;
}

//
double NC_FIFO_OFFSET::computeGranularity(QList<double> BAGs)
{
    if(BAGs.empty())  throw std::runtime_error("invalid BAG list : computeGranularity()");

    //   granularity is computed as: (cite::...)
    //      (a) sort the BAG values in decreasing order
    //      (b) replace each pair of repeating BAG by a single BAG of their half value
    //      (c) repeate (a) and (b) until a list of unique BAG values is obtained
    //      (d) compute granularity
    //

    //      (a) sort the BAG values in decreasing order
    std::sort(BAGs.rbegin(), BAGs.rend());

    for(int i = 0; i < BAGs.size(); ++i) {
        double const BAG = BAGs.at(i);

        //      (b) replace each pair of repeating BAG by a single BAG of their half value
        if(BAGs.count(BAG) > 1) {
            BAGs.removeOne(BAG); BAGs.removeOne(BAG); // remove BAG pair
            BAGs.append(trimDouble(BAG/2)); // insert single BAG with half a value

            //      (c) repeate (a) and (b) until a list of unique BAG values is obtained
            std::sort(BAGs.rbegin(), BAGs.rend());
            --i; continue;
        }
    }

    //      (d) compute granularity
    double G = BAGs.last();
    if(BAGs.size() > 1) { G = trimDouble(G/2); }
    return G;
}
