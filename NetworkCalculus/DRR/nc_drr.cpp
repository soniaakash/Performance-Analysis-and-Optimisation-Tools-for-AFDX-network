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

#include "NetworkCalculus/DRR/nc_drr.h"

NC_DRR::NC_DRR()
{
    configReady = false;
}

// initialise DRR scheduler
//      define flow classes
//      fix quantum (in bits) value assigned to each flow class
bool NC_DRR::initDRRConfig(const NCData *ncData)
{
    // comment the next line to use the DRR configuration for the sample network configuration initialised in AFDX::loadConfig_hardcoded()
    qStdOut() << "DEFINE YOUR OWN DRR CONFIGURATION"; return false;


    // ****** sample ******
    //  it belongs to the network example in AFDX::loadConfig_hardcoded()
    // define classes
     fID_cID_map.insert(1001,1); cID_quantum_map.insert(1, 800); cID_lMax_map.insert(1, ncData->selectNCFlow_ref(1001).getLMax());
     fID_cID_map.insert(1002,2); cID_quantum_map.insert(2, 800); cID_lMax_map.insert(2, ncData->selectNCFlow_ref(1002).getLMax());
     fID_cID_map.insert(1003,3); cID_quantum_map.insert(3, 800); cID_lMax_map.insert(3, ncData->selectNCFlow_ref(1003).getLMax());
    // ****** End of sample ******

    qStdOut() << "DRR configuration success!";
    configReady = true;
    return configReady;
}

// get the classID of the given flow
const int &NC_DRR::getClassID(const int &flowID)
{
    auto it = fID_cID_map.find(flowID); if(it == fID_cID_map.end()) throw std::runtime_error("Invalid flowID in :: NC_DRR::getClassID()");
    return it.value();
}

// get list of classes
QList<int> NC_DRR::getClasses()
{
    return make_nonRepeatingSortedList(fID_cID_map.values());
}

