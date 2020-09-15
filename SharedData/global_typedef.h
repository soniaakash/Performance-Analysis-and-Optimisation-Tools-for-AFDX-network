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

#ifndef GLOBAL_TYPEDEF_H
#define GLOBAL_TYPEDEF_H

#include <QDebug>
#include <QMap>
#include <QStringList>
#include <QTextStream>
#include <QMetaEnum>

// this file is included by all the classes in the project
// it includes definition of common
//      macros
//      data types
//          structures
//          enums (used as configuration options)
//      mathematical operations
//      constant
//      debug function

// to print output on console
extern bool DISPLAY_OUTPUT_DEFAULT;
extern bool DISPLAY_OUTPUT;
extern bool DISPLAY_WAITCURSOR;
QTextStream& qStdOut(bool DISPLAY_INLINE = false);
QTextStream& qStdOutInLine();
QTextStream& qStdIn();

// to prevent floating point precision problems
const int DECIMAL_POINTS = 5;
const int PRECISION = std::pow(10,DECIMAL_POINTS);
double trimDouble(double value, int uptoDecimal = DECIMAL_POINTS);
bool approxEquality(double const &value, double const &comparedToValue, const int uptoDecimal = DECIMAL_POINTS);

// ***************** typedef : Global ***************** //
// ***************** **************** ***************** //

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************        AFDX      ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //

typedef QPair<QString,int> NODE_t; // an output port <name,id>
typedef QList<NODE_t> PATH; // a flow path :: list of output ports :: EndSystem -> Switches

// for debug only
QString NODE_t_toString(NODE_t const&n);
QString PATH_toString(PATH const&path);
template <typename T>
QString MAP_toString(QMultiMap<int,T> const &m) {
    QString str; auto it = m.cbegin();
    while(it != m.cend()) {
        str.append(QString( "(" + QString().setNum(it.key()) + " : " + QString().setNum(it.value(), 'f', 5) + ") " ));
        ++it;
    }
    return str;
}
template <typename T>
QString MAP_toString(QMap<int,T> const &m) {
    QMultiMap<int, T> mm = m;
    return MAP_toString(mm);
}
template <typename T>
QString LIST_toString(QList<T> const &l) {
    QString str; auto it = l.cbegin();
    while(it != l.cend()) {
        str.append(QString().setNum(*it));
        if(it+1 != l.cend()) str.append(", ");
        ++it;
    }
    return str;
}

enum NODE_TYPE{ // type of AFDX node
    ES, // an end system
    VirtualES, // It is not a part of neither AFDX nor Network calculus. virtual node is explicitly added in this project at the beginning of each flow path. It is to allow compatibility with the flow paths begining at a "Gateway". transmission delay at virtual node = 0.
    SW, // a switch
};

// a network node in AFDX netork.
//      this struct is initiated by class AFDX.
//      it serves as a generic network node model which is re-implemented/extended in different analysis approaches
typedef struct AFDXNODE {
    AFDXNODE(QString name, NODE_TYPE nType) { _name = name; nodeType = nType; }
private:
    QString _name; // node name
    NODE_TYPE nodeType; // type of AFDX node : ES and SWITCH
    QMap<int,double> ports; // <portID, linkRate> :: list of output ports and output link rate (in bits/microSecond)

public:
    QString const &getNodeName() const {return _name;}
    NODE_TYPE const &getNodeType() const {return nodeType;}
    QMap<int,double> const &getPorts() const {return ports; }
    void addPorts(int portID, double linkRate) { ports.insert(portID,linkRate);}
    static NODE_t makeVirtualNode(); // virtual node is explicitly added in this project at the beginning of each flow path. It is to allow compatibility with the flow paths begining at a "Gateway" for . transmission delay at virtual node = 0
    static bool isVirtalES(NODE_t const&node); // virtual node is explicitly added in this project at the beginning of each flow path. It is to allow compatibility with the flow paths begining at a "Gateway" for . transmission delay at virtual node = 0
} NODE_generic; // ES and Switch


// a flow in AFDX netork.
//      this struct is initiated by class AFDX.
//      it serves as a generic flow model which is re-implemented/extended in different analysis approaches
typedef struct AFDXFLOW {
    AFDXFLOW(int id, int max, int min, int bag, int p = 0) { fID = id; lMax = max; lMin = min; BAG = bag; priority = p; }
private:
    int fID; // flowID
    int lMax; // max frame size (bits)
    int lMin; // min frame size (bits)
    int BAG; // bandwidth allocation gap (or) inter-frame time  (microSec)
    int priority; // 0 (lowest) ... + integers
    QMap<int, PATH> multipath; // list of flow paths <pathID, path>
    QList<NODE_t> allNodes; // list of all the traversed nodes

public:
    int const& getFlowID() const {return fID;}
    int const& getLMax() const {return lMax;}
    int const& getLMin() const {return lMin;}
    int const& getBAG() const {return BAG;}
    int const& getPriority() const {return priority;}
    int getPathCount() const {return multipath.size();}
    PATH const &getPath(int pNo) const {auto it = multipath.find(pNo); if(it == multipath.end()) throw std::runtime_error("PATH OUT OF REFERENCE! :: AFDXFLOW::getPath()"); return it.value();}
    int getPathNumber(PATH const&path) const { return multipath.key(path, -1); }
    NODE_t getPreviousNode(NODE_t const&node) const { auto it = multipath.begin(); while(it != multipath.end()) { int index = it->indexOf(node); if(index != -1) {if(index > 0) { return it->at(index-1); } return AFDXNODE::makeVirtualNode(); } ++it;} throw std::runtime_error("Invalid node selection in :: AFDXFLOW::getPreviousNode()");}
    QList<NODE_t> const &getAllNodes() const {return allNodes;}
    void addPath(PATH &p) { multipath.insert((getPathCount()),p); foreach(NODE_t n, p){if(!allNodes.contains(n))allNodes.append(n);}}
} FLOW;


// ***************** **************** ***************** //
// ***************** **************** ***************** //
// ***************** scheduling policy ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //
enum class SCHEDULING {
    FIFO,
    SPQ,
    DRR,
    WRR,
};

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************  config options  ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //
enum class MODE {
    WCTT,
    QOS_TUNING,
};

enum class OPTIONS{
    // DEFAULT
    NONE,

    // FIFO_OPTION and/or SPQ_OPTION
    UNDER_BOUND,
    OFFSETS,

    // DRR_OPTIONS
    CLASSIC,
    OPTIMISED,
    QUANTUM_ASSIGNEMENT,
    QUANTUM_ASSIGNEMENT_IMPROVED,

};

QString enumToString(SCHEDULING value);
QString enumToString(MODE value);
QString enumToString(OPTIONS value);


// ***************** **************** ***************** //
// ***************** **************** ***************** //
// ***************** shared functions ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //
template <typename T>
QList<T> make_nonRepeatingSortedList(QList<T> const&l)
{
    QList<T> resultList;
    foreach (T const &item, l) {
        if(!resultList.contains(item)) resultList.append(item);
    }
    std::sort(resultList.begin(), resultList.end());
    return resultList;
}


#endif // GLOBAL_TYPEDEF_H
