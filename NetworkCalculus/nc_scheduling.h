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

#ifndef NC_SCHEDULING_H
#define NC_SCHEDULING_H

#include "AFDX/afdx.h"
#include "NetworkCalculus/ncdata.h"

// this is a generic NC scheduling class
//      it defines the data variables for different schedulers
class NC_Scheduling
{
public:
    NC_Scheduling();

protected:
    NCData * ncData; // afdx config data (expanded) for NC computations

    QMultiMap<int, QPair<int,double> > flowID_pathNo_e2eDelays;  // results <flowID, <pathNo, e2eDelay>> : list of computed e2e delays per flow per path
};

#endif // NC_SCHEDULING_H
