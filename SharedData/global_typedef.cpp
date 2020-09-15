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

#include "SharedData/global_typedef.h"

// to prevent floating point precision problems
double trimDouble(double value, int uptoDecimal) { double precision = std::pow(10,uptoDecimal); value *= precision; value = floor(value); return QString().setNum(value/precision, 'f', 5).toDouble(); /*value/precision;*/}

// compare approximate equality of two values
bool approxEquality(double const &value, double const &comparedToValue, int const uptoDecimal) {
    double epsilon = trimDouble(10.0/std::pow(10,uptoDecimal)); // qStdOut() << QString().setNum(value , 'f', DECIMAL_POINTS) << " compared to " << QString().setNum(comparedToValue - epsilon, 'f', DECIMAL_POINTS) << " and " << QString().setNum(comparedToValue + epsilon, 'f', DECIMAL_POINTS);
    return (((comparedToValue-epsilon) <= value) && (value <= (comparedToValue+epsilon)));
}

// for debug only
QString NODE_t_toString(const NODE_t &n) { return QString(n.first + ":" + QString().setNum(n.second)); }
QString PATH_toString(const PATH &path) { QString str; foreach (NODE_t const&n, path) { str.append(NODE_t_toString(n) + " "); } return str; }

// virtual node is explicitly added in this project at the beginning of each flow path. It is to allow compatibility with the flow paths begining at a "Gateway" for . transmission delay at virtual node = 0
bool AFDXNODE::isVirtalES(NODE_t const&node)
{
    return (node.first == "VirtualES" && node.second == 99)?true:false;
}

// virtual node is explicitly added in this project at the beginning of each flow path. It is to allow compatibility with the flow paths begining at a "Gateway" for . transmission delay at virtual node = 0
NODE_t AFDXNODE::makeVirtualNode()
{
    return qMakePair(QString("VirtualES"),99);
}

#include <QTime>
// to print output on console
bool DISPLAY_OUTPUT = true;
bool DISPLAY_OUTPUT_DEFAULT = DISPLAY_OUTPUT;
bool DISPLAY_WAITCURSOR = true;
static QTextStream r(stdout);
QTextStream &qStdOut(bool DISPLAY_INLINE)
{
    r.flush();
    //#define qStdOut() (QTextStream(stdout) << '\n' )
    if(DISPLAY_OUTPUT){
        if(DISPLAY_INLINE) r << " ";
        else r << '\n';
        return r;
    }

    if(DISPLAY_WAITCURSOR){ // show the waiting cursor
        static int prevtStamp = 0;
        static int tStamp = 0;
        tStamp = QTime().currentTime().second();
        if((tStamp - prevtStamp) > 5){
            r << " .";
            prevtStamp = tStamp;
        } else if(tStamp < prevtStamp) {
            prevtStamp = tStamp;
        }
    }

    static QString str; str.clear();
    static QTextStream noOutput(&str);
    return noOutput;
}


QTextStream &qStdOutInLine()
{
    return qStdOut(true);
}

QTextStream &qStdIn()
{
    //#define qStdIn() (QTextStream(stdin))
    static QTextStream rIn(stdin);
    return rIn;
}


QString enumToString(SCHEDULING value)
{
    QString str;
    switch (value) {
    case SCHEDULING::FIFO:str.append("FIFO");break;
    case SCHEDULING::SPQ:str.append("SPQ");break;
    case SCHEDULING::DRR:str.append("DRR");break;
    case SCHEDULING::WRR:str.append("WRR");break;
    default: str.append("unknown!");break;
    }
    return str;
}

QString enumToString(MODE value)
{
    QString str;
    switch (value) {
    case MODE::WCTT:str.append("WCTT");break;
    case MODE::QOS_TUNING:str.append("QOS_TUNING");break;
    default: str.append("unknown!");break;
    }
    return str;
}

QString enumToString(OPTIONS value)
{
    QString str;
    switch (value) {
    case OPTIONS::NONE:str.append("NONE");break;
    case OPTIONS::OFFSETS:str.append("OFFSETS");break;
    case OPTIONS::UNDER_BOUND:str.append("UNDER_BOUND");break;
    case OPTIONS::CLASSIC:str.append("CLASSIC");break;
    case OPTIONS::OPTIMISED:str.append("OPTIMISED");break;
    case OPTIONS::QUANTUM_ASSIGNEMENT:str.append("QUANTUM_ASSIGNEMENT");break;
    case OPTIONS::QUANTUM_ASSIGNEMENT_IMPROVED:str.append("QUANTUM_ASSIGNEMENT_IMPROVED");break;
    default: str.append("unknown!");break;
    }
    return str;
}

//QList<int> makeSortedNonrepeatingList(const QList<int> &l)
//{
//    QMap<int,int> resultList;
//    foreach (int const &item, l) {
//        resultList.insert(item,0);
//    }
//    return resultList.keys();
//}
