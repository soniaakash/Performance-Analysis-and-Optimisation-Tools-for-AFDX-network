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

#include "AFDX/afdx.h"
#include <QDebug>

#include <QFile>
#include <QFileInfo>

AFDX::AFDX()
{

}

// load AFDX network config from input file
bool AFDX::loadConfig_fromFile() {

    // comment the next line to use the parser of the default file format
     qStdOut() << "DEFINE YOUR OWN CONFIG FILE PARSER :: AFDX::loadConfig_fromFile()"; return false;

    // default config file format
    //      * different entities in each line are separated by a black space
    // **** begin file format ****
    // <no_of_end_systems>
    // <name_of_first_end_system> <no_of_output_ports>
    //      <port_id> <link_rate>
    //      <port_id> <link_rate>
    //      ...
    // <name_of_second_end_system> <no_of_output_ports>
    //      <port_id> <link_rate>
    //      <port_id> <link_rate>
    //      ...
    // <no_of_switches>
    // <name_of_first_switch> <no_of_output_ports>
    //      <port_id> <link_rate>
    //      <port_id> <link_rate>
    //      ...
    // <name_of_second_switch> <no_of_output_ports>
    //      <port_id> <link_rate>
    //      <port_id> <link_rate>
    //      ...
    // <no_of_flows>
    // <first_flow_id> <maximum_frame_legth> <minimum_frame_length> <bandwidth_allocation_gap> <priority> <no_of_paths>
    // <first_node_in_path_1> <second_node_in_path_1> ...  <last_node_in_path_1>
    // <first_node_in_path_2> <second_node_in_path_2> ...  <last_node_in_path_2>
    // ...
    // <second_flow_id> <maximum_frame_legth> <minimum_frame_length> <bandwidth_allocation_gap> <priority> <no_of_paths>
    // <first_node_in_path_1> <second_node_in_path_1> ...  <last_node_in_path_1>
    // <first_node_in_path_2> <second_node_in_path_2> ...  <last_node_in_path_2>
    // ...
    // **** end file format ****


    // default AFDX network config file parser
    QString fileName = "full_path_to_your_config_file/full_name_of_your_config_file"; // "C:/Users/myusername/Performance_Analysis_Tools/config/configFileName.mod";

    qStdOut() << "Read Config Begin : " << fileName;

    QFile inputFile(fileName);
    if (inputFile.open(QIODevice::ReadOnly))
    {        
        // Input File Handler
        QTextStream in(&inputFile);

        QString line; // data string of fetched line
        if(!in.atEnd()) line = in.readLine(); // read line
        else return false;

        QStringList lists = line.split(" "); // break the contents of the line separated by a blank space and store them as a list of strings

        // End Systems
        int totalES = QString(lists.at(0)).toInt(); // first line of the config file
        QList<NODE_t> allES; // list of all end-systems

        // loop to parse all the end-systems
        for(int i = 0; i < totalES; ++i) { // ES
            if(!in.atEnd()) line = in.readLine();
            else break;
            lists = line.split(" ");
            QString esName = lists.at(0); // ES name
            int nbPorts = QString(lists.at(1)).toInt(); // number of ports
            // loop to parse all the output ports of the end-system
            for(int i = 0; i < nbPorts; ++i) { // ES ports
                if(!in.atEnd()) line = in.readLine();
                else break;
                lists = line.split(" ");
                for(int j = 0; j < lists.size(); ++j) { // port ID and link rate
                    if(lists.at(j) == "") continue;
                    int portID = QString(lists.at(j + 0)).toInt();
                    int linkRate = QString(lists.at(j + 1)).toInt();
                    allES << makePort(esName, portID, linkRate, NODE_TYPE::ES); // store collected data as a new output port
                    break;
                }
            }
        }

        if(!in.atEnd()) line = in.readLine();
        else return false;

        lists = line.split(" ");

        // Switches
        int totalSW = QString(lists.at(0)).toInt();
        QList<NODE_t> allSW; // list of all end-systems

        // loop to parse all the switches
        for(int i = 0; i < totalSW; ++i) { // SW
            if(!in.atEnd()) line = in.readLine();
            else break;
            lists = line.split(" ");
            QString swName = lists.at(0);
            int nbPorts = QString(lists.at(1)).toInt();
            // loop to parse all the output ports of the switch
            for(int i = 0; i < nbPorts; ++i) { // SW ports
                if(!in.atEnd()) line = in.readLine();
                else break;
                lists = line.split(" ");
                for(int j = 0; j < lists.size(); ++j) {
                    if(lists.at(j) == "") continue;
                    int portID = QString(lists.at(j + 0)).toInt();
                    int linkRate = QString(lists.at(j + 1)).toInt();
                    allSW << makePort(swName, portID, linkRate, NODE_TYPE::SW); // store collected data as a new output port
                    break;
                }
            }
        }

        if(!in.atEnd()) line = in.readLine();
        else return false;

        lists = line.split(" ");

        // Flows
        int totalFlows = QString(lists.at(0)).toInt();
        int pathCount = 0;

        // loop to parse all the flows and their paths
        for(int i = 0; i < totalFlows; ++i) { // Flow
            if(!in.atEnd()) line = in.readLine();
            else break;
            lists = line.split(" ");
            // flow details
            int flowID = QString(lists.at(0)).toInt();
            int lMax = QString(lists.at(1)).toInt();
            int lMin = QString(lists.at(2)).toInt();
            int BAG = QString(lists.at(3)).toInt();
            int priority = QString(lists.at(4)).toInt();
            int nbPaths = QString(lists.at(5)).toInt();

            flows << FLOW(flowID, lMax, lMin, BAG, priority);  // store collected data as a new flow

            // loop to parse all the paths of a flow
            for(int i = 0; i < nbPaths; ++i) { // Flow paths
                if(!in.atEnd()) line = in.readLine();
                else break;
                lists = line.split(" ");
                PATH fPath;
                for(int j = 0; j < lists.size(); ++j) {
                    if(lists.at(j) == "") continue;
                    QString nodeName = lists.at(j); ++j;
                    int portID = QString(lists.at(j)).toInt();
                    NODE_t node(nodeName,portID);         // temporarily store the collected data as a compatible format of an output node
                    fPath << ((allES.contains(node) || allSW.contains(node))?node:throw std::runtime_error("invalid node in flow path")); // check if the node was created (ES ot switch) in previous loops. store it as a node in the flow path
                }
                flows.last().addPath(fPath); // store collected data as a new flow path
                ++pathCount;
            }

        }

        qStdOut() << "load config file success!";
        qStdOut() << "flows " << flows.size() << " ES " << allES.size() << " SW " << totalSW << " Paths " << pathCount;
        inputFile.close();

        configName = QFileInfo(fileName).fileName();
        return true;
    }
    // end of default AFDX network config file parser


    qStdOut() << "unable to open config file!";
    return false;
}

// load AFDX network config from the code
bool AFDX::loadConfig_hardcoded() {

    // a sample of hardcoded network configuration
    // it has 3 flows with flow IDs 1001, 1002 and 1003
    // each flow has only one path in the network
    // there ae 3 end-systems ES1, ES2 and ES3
    // and 2 switches SW1 and SW2
    // all the links have link rate of 100 Mbits/sec (i.e. 100 bits/micro sec)
    //
    // advice :: to prevent any error:
    //      first initialise flows
    //      initialise nodes (ES and switches) by calling makePort()
    //      add flow paths

    // define nodes and flow paths
    PATH f1Path; f1Path << makePort("ES1",1, 100, NODE_TYPE::ES) << makePort("SW1",1, 100,NODE_TYPE::SW) << makePort("SW2",1, 100,NODE_TYPE::SW);
    PATH f2Path; f2Path << makePort("ES1",1, 100, NODE_TYPE::ES) << makePort("SW1",1, 100,NODE_TYPE::SW) << makePort("SW2",1, 100,NODE_TYPE::SW);
    PATH f3Path; f3Path << makePort("ES2",1, 100, NODE_TYPE::ES) << makePort("SW1",1, 100,NODE_TYPE::SW) << makePort("SW2",1, 100,NODE_TYPE::SW);


    // define flows                             // add paths to flows
    flows << FLOW(1001, 100*8, 100*8, 32000);   flows.last().addPath(f1Path);
    flows << FLOW(1002, 100*8, 100*8, 32000);   flows.last().addPath(f2Path);
    flows << FLOW(1003, 100*8, 100*8, 32000);   flows.last().addPath(f3Path);

    qStdOut() << "load static connfig success!";

    configName = "static";

    return true;
}

// make compatible output port of a network node (for initialisation purpose)
NODE_t AFDX::makePort(QString nodeName, int portID, int linkRate, NODE_TYPE nType) {
    // add output to existing node
    auto it = nodes.begin();
    while(it != nodes.end()){ if(it->getNodeName() == nodeName) {it->addPorts(portID,linkRate); return qMakePair(nodeName,portID);} ++it;}

    // otherwise, make new node
    nodes << NODE_generic(nodeName, nType); nodes.last().addPorts(portID, linkRate); return qMakePair(nodeName,portID);
}
