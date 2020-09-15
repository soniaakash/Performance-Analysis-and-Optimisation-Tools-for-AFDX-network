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

#ifndef AFDX_H
#define AFDX_H

#include "SharedData/global_typedef.h"

// a generic class to store AFDX config data :: Flows, ES, Switches, link rate, flow paths, ...
// this class is a parser of AFDX network configuration.
// the AFDX network configuration can be provided by either of the two input AFDX_SOURCE
//      STATIC source
//          - loadConfig_hardcoded()
//              * a sample network configuration is provided
//      input FILE source
//          - loadConfig_fromFile()
//              * the implementation is highly dependent on the input file format
//              * a sample file format is provided in afdx_config.txt
//              * along with a sample code to parse this file in loadConfig_fromFile()
class AFDX
{
public:
    AFDX();

    // the AFDX network configuration can be provided one of the following sources
    enum AFDX_SOURCE {
        FILE,
        STATIC
    };

    // load AFDX network config from input file
    bool loadConfig_fromFile();
    // load AFDX network config from the code
    bool loadConfig_hardcoded();

    // check input source and load the AFDX network config
    bool initConfig(AFDX_SOURCE srcType) {
        switch (srcType) {
        case AFDX::FILE:
            return loadConfig_fromFile();
        case AFDX::STATIC:
            return loadConfig_hardcoded();
        default:
            qStdOut() << "invalid AFDX network source. unbale to init connfig! :: AFDX::initConfig()";
            return false;
        }
    }

    // get list of flows
    QList<FLOW> const &getFLows() const {return flows;}
    // get list of network nodes
    QList<NODE_generic> const &getNodes() const {return nodes;}
    // get config source name
    QString const &getConfigName() const {return configName;}


private:
    QList<FLOW> flows; // list of flows
    QList<NODE_generic> nodes; // list of nodes

    // make compatible output port of a network node (for initialisation purpose)
    NODE_t makePort(QString nodeName, int portID, int linkRate, NODE_TYPE nType);

    QString configName; // name of config source
};

#endif // AFDX_H
