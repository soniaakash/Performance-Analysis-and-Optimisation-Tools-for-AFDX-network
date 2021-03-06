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

#ifndef NC_DRR_H
#define NC_DRR_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"
#include "NetworkCalculus/nc_scheduling.h"

// this is a generic class for DRR scheduling
//      it defines the common data variables and member functions for different NC approaches
//
// it is used to initialise DRR scheduler
//      define flow classes
//      fix quantum (in bits) value assigned to each flow class
//
class NC_DRR : public NC_Scheduling
{
public:
    NC_DRR();

    // compute worst-case end-to-end delay for all flows on all paths in the network
    virtual void computeE2E(NCData *dataPtr) = 0;

    // initialise DRR scheduler
    bool initDRRConfig(const NCData *ncData);

    // get the classID of the given flow
    int const& getClassID(int const &flowID);

    // get list of classes
    QList<int> getClasses();

private:
    QMap<int,int> fID_cID_map; // <flowID, classID> : mapping of flows per class
protected:
    QMap<int,int> cID_quantum_map; // <classID, quantum> : mapping of quantum (in bits) per class
    QMap<int,int> cID_lMax_map; // <classID, max frame size> : mapping of max frame size (in bits) per class
    bool configReady; // initialisation status flag
};

#endif // NC_DRR_H
