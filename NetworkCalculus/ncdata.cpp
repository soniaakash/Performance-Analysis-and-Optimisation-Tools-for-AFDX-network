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

#include "ncdata.h"

NCData::NCData()
{

}

// initialise ncdata
bool NCData::initData(const AFDX &afdxConfig)
{
    // load afdx flows
    flows = afdxConfig.getFLows();

    // load network nodes
    nodes.clear();
    foreach (NODE_generic const& nodeGeneric, afdxConfig.getNodes()) {
        nodes << NCNode(nodeGeneric);
    }

    // extract information needed for NC computations : list of flows per output port
    QList<NCFlow>::const_iterator flow_it = flows.begin();
    while(flow_it != flows.end()) { // for each flow
        QList<NODE_t> const& allNodes_ofSelectedFLow = flow_it->getAllNodes();
        foreach (NODE_t n, allNodes_ofSelectedFLow) {
            selectNCNode(n.first)->associateFlowList(n.second,QList<int>{flow_it->getFlowID()}); // update output port
        }
        ++flow_it;
    }

    qStdOut() << "nc init data success";

    configName = afdxConfig.getConfigName();
    return true;
}
