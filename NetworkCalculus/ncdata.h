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

#ifndef NCDATA_H
#define NCDATA_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/nccurves.h"


// NC DATA structure
//   NCNode is a network node : ES or SW
//   NCPort is an output port of a NCNode
//   NCFLow is an AFDX flow
//
// Info:
//   This class has same network information (and much more) as class AFDX.
//   NCNode can have one or more NCPort
//   NCPort has a list of flows sharing this port
//   Computation results (NC Curves, delays ... ) are stored in NCPort for quick reference. (Improved computation speed)
class NCData
{
    // classes that can directly access NCData members
    friend class NC_FIFO; // FIFO schduling policy
    friend class NC_FIFO_OFFSET; // FIFO schduling policy with offsets
    friend class NC_SPQ; // SPQ schduling policy
    friend class NC_DRR; // DRR schduling policy (base class)
    friend class NC_DRR_Classic; // DRR schduling policy (classical approach)
    friend class NC_DRR_Optimised; // DRR schduling policy (optimised approach)
    friend class NC_DRR_QuantumAssignment; // DRR schduling policy (QoS tuning based on classical approach)
    friend class NC_DRR_QuantumAssignmentImproved; // DRR schduling policy (QoS tuning based on optimised approach)
    friend class NC_WRR; // WRR schduling policy (base class)
    friend class NC_WRR_Optimised; // WRR schduling policy (optimised approach)
    friend class NC_WRR_Optimised_New; // WRR schduling policy (optimised approach)
    friend class NC_UNDERBOUND; // Underbound on FIFO and SPQ scheduling

public:
    NCData();

    // network nodes defined for NC computaions
    typedef class NetworkCalculusNode {
    public:
        NetworkCalculusNode(QString name, NODE_TYPE type) {_name = name; nType = type;}
        NetworkCalculusNode(NODE_generic const& nodeGeneric) { _name = nodeGeneric.getNodeName(); nType = nodeGeneric.getNodeType(); QMap<int,double> const &ports = nodeGeneric.getPorts(); foreach (int pID, ports.keys()) { addOutputPort(pID,ports.value(pID)); } }

        // output port defined for NC computaions
        typedef class NetworkCalculusPort{
            // this is an encapsulation of an output port in NCNode
            // all members in this class are public and directly accessed by NCNode
        public:
            NetworkCalculusPort();
            NetworkCalculusPort(int portID, double rate) { id = portID; linkRate = rate; }
            int id; // portID
            double linkRate; // (in bits/microSecond)
            QList<int> bufferFlows; // list of flows at this output port

            // ******** results of NC ********
            // Common to all scheduling
            QMap<int, double> fID_delay_map; // <flowID, delay in microSec> result : computed delay at output port
            QMap<int, ConcaveCurve> fID_arrivalCurve_map; // <flowID, arrival curve> result : computed arrival curve at output port
            QMap<int, ConcaveCurve> fID_overallArrivalCurve_map; // <flowID, overall arrival curve> result : computed overall arrival curve at output port
            // ******** end : results of NC ********
        }NCPort;

        // add new output port (ncdata initialisation)
        void addOutputPort(int const &pID, double const &linkRate) { outputPorts.insert(pID,NCPort(pID,linkRate)); }
        // add list of flows traversing output port (ncdata initialisation)
        void associateFlowList(int const &pID, QList<int> const &flowList) { QMap<int, NCPort>::iterator it = selectPort(pID); foreach (int fID, flowList) { if(!it->bufferFlows.contains(fID)) it->bufferFlows.append(fID); }}

        // get node name
        QString const& getName() const {return _name;}
        // get node type
        NODE_TYPE const& getNodeType() const {return nType;}
        // get list of flows traversing output port
        QList<int> const &getFlowList(int const &pID) { return selectPort(pID)->bufferFlows; }
        // get link rate
        double const& getLinkRate(int const &pID) { return selectPort(pID)->linkRate; }
        // get switching latency. sl = const for a switch and sl = 0 for end-system
        double getSwitchingLatnecy() { return (nType==SW)?SWITCHING_LATENCY:0; }

        // functions for nc result management
        void addComputedDelay(int const& pID, int const& fID, double const& delay) { selectPort(pID)->fID_delay_map.insert(fID, delay); }
        bool getComputedDelay(int const& pID, int const& fID, double &delay) { double d = selectPort(pID)->fID_delay_map.value(fID, -1); if(d > 0) {delay = d; return true;} return false; } // if result not found : return false and set delay = 0
        void addComputedArrivalCurve(int const& pID, int const& fID, ConcaveCurve const& alpha) { selectPort(pID)->fID_arrivalCurve_map.insert(fID, alpha); }
        bool getComputedArrivalCurve(int const& pID, int const& fID, ConcaveCurve &alpha) { auto it = selectPort(pID)->fID_arrivalCurve_map.find(fID); if(it != selectPort(pID)->fID_arrivalCurve_map.end()) { alpha = it.value(); return true; } return false; } // if result not found : return false and alpha is not changed.
        void addComputedOverallArrivalCurve(int const& pID, int const& fID, ConcaveCurve const& alpha_o) { selectPort(pID)->fID_overallArrivalCurve_map.insert(fID, alpha_o); }
        bool getComputedOverallArrivalCurve(int const& pID, int const& fID, ConcaveCurve &alpha_o) { auto it = selectPort(pID)->fID_overallArrivalCurve_map.find(fID); if(it != selectPort(pID)->fID_overallArrivalCurve_map.end()) { alpha_o = it.value(); return true; } return false; } // if result not found : return false and alpha_o is not changed.

    private:
        QString _name; // node name
        NODE_TYPE nType; // node type
        QMap<int, NCPort> outputPorts; // list of output ports at this node. <portID, NCPort>
        const double SWITCHING_LATENCY = 0; // switching latency (in microSecond)

        // function for internal reference in class
        QMap<int, NCPort>::iterator selectPort(int portID) {auto it = outputPorts.find(portID); if(it == outputPorts.end()) { throw std::runtime_error("Invalid portID in :: NCNode::selectPort()");} return it;}
    }NCNode;

    // AFDX flows defined for NC computaions
    typedef struct AFDXFLOW NCFlow;

    // initialise ncdata
    bool initData(AFDX const &afdxConfig);

    // get config source name
    QString const& getConfigName() const { return configName; }

    // functions for quick (flow and node) refernece while computing delay
    QList<NCNode>::iterator selectNCNode(QString nodeName) {QList<NCNode>::iterator it = nodes.begin(); while(it!=nodes.end()){if(it->getName()==nodeName)break;++it;} if(it==nodes.end()){ throw std::runtime_error("Invalid nodeName in :: NCNode::selectNCNode()");} return it;}
    QList<NCFlow>::iterator selectNCFlow(int flowID) {QList<NCFlow>::iterator it = flows.begin(); while(it!=flows.end()){if(it->getFlowID()==flowID)break;++it;} if(it==flows.end()){ throw std::runtime_error("Invalid flowID in :: NCNode::selectNCFlow()");} return it;}
    NCNode& selectNCNode_ref(QString const &nodeName) {int count = nodes.size(); for(int index = 0; index < count; ++index) { if(nodes.at(index).getName()==nodeName) return nodes[index]; } throw std::runtime_error("Invalid nodeName in :: NCNode::selectNCNode_ref()"); }
    NCNode const& selectNCNode_ref(QString const &nodeName) const {int count = nodes.size(); for(int index = 0; index < count; ++index) { if(nodes.at(index).getName()==nodeName) return nodes.at(index); } throw std::runtime_error("Invalid nodeName in :: NCNode::selectNCNode_constRef()"); }
    NCFlow& selectNCFlow_ref(int const &flowID) {int count = flows.size(); for(int index = 0; index < count; ++index) { if(flows.at(index).getFlowID()==flowID) return flows[index]; } throw std::runtime_error("Invalid flowID in :: NCNode::selectNCFlow_ref()"); }
    NCFlow const& selectNCFlow_ref(int const &flowID) const {int count = flows.size(); for(int index =0; index < count; ++index) { if(flows.at(index).getFlowID()==flowID) return flows.at(index); } throw std::runtime_error("Invalid flowID in :: NCNode::selectNCFlow_constRef()"); }
private:
    QList<NCNode> nodes; // list of network nodes // initialised from the data parsed in class AFDX
    QList<NCFlow> flows; // list of afdx flows    // initialised from the data parsed in class AFDX

    QString configName; // name of config source
};


#endif // NCDATA_H
