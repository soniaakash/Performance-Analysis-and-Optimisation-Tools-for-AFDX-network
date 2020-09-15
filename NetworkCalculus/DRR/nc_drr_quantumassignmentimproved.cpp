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

#include "nc_drr_quantumassignmentimproved.h"

NC_DRR_QuantumAssignmentImproved::NC_DRR_QuantumAssignmentImproved()
{
    ncData = nullptr;
}

// initialise data and start computation
void NC_DRR_QuantumAssignmentImproved::initComputation(const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::computeE2E()");

    // **** define computation environment ****

    if(!NC_DRR::initDRRConfig(dataPtr)) return; // init DRR scheduler

    int initialQ = (10244 * 8); // set initial quantum (in bits)

    // set critical class constraints

    // comment the next line to define constraints for the sample network configuration initialised in AFDX::loadConfig_hardcoded()
    qStdOut() << "DEFINE YOUR OWN CRITICAL CLASS CONSTRAINTS :: NC_DRR_QuantumAssignment::initComputation()"; return;

    // ****** sample ******
    //  it belongs to the network example in AFDX::loadConfig_hardcoded()
    setDeadline(1, 1000);
    setDeadline(2, 1000);
    // ****** end of sample ******

    // **** end of define computation environment ****

    qStdOut() << "start QoS tuning!";

    // compute
    computeOptimalQuantumAndItsDistribution(initialQ, dataPtr);
}

// compute the optimal quantum and its distribution
QMap<int, int> NC_DRR_QuantumAssignmentImproved::computeOptimalQuantumAndItsDistribution(const int &initialQuantumValue, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::computeOptimalQuantumAndItsDistribution()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::computeOptimalQuantumAndItsDistribution()");

    // the algorithm aims to improve the quantum assignement result of NC_DRR_QuantumAssignment
    // the improvement is divided into multiple steps (cite: )
    //     step I: compute initial quantum distribution using NC_DRR_QuantumAssignment
    //     step II: improve quantum distribution (loop step III and step IV)
    //     step III:  Minimise critical class service
    //     step IV: Maximise non-critical class service (loop step IVa and step IVb)
    //          step IVa: Increase Cn quantum to maximum possible value
    //          step IVb: Increase the difference between the deadline and the maximum end-to-end delay for the critical class having smallest value of this difference.
    // ***************************************************************************

    //     step I: compute initial quantum distribution using NC_DRR_QuantumAssignment.
    NC_DRR_QuantumAssignment nc_quantumAssignment;
    // **** define computation environment in NC_DRR_QuantumAssignment ****
    DISPLAY_OUTPUT = false;
    nc_quantumAssignment.initDRRConfig(dataPtr); // configure DRR scheduler in NC_DRR_QuantumAssignment
    DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;
    foreach(int const&cID, cID_deadline.uniqueKeys()) { // for each critical class
        nc_quantumAssignment.setDeadline(cID, this->getDeadline(cID));// set deadline in NC_DRR_QuantumAssignment
    }
    // **** end of define computation environment NC_DRR_QuantumAssignment ****

    qStdOut() << "compute initial quantum using NC_DRR_QuantumAssignment";

    // compute intial quantum distribution
    QMap<int,int> initial_cID_quantum = nc_quantumAssignment.computeOptimalQuantumAndItsDistribution(initialQuantumValue, dataPtr);

    // display initial quantum distribution
    qStdOut() << "algorithm input: initial quantum distribution";
    int Q_sum = 0;
    foreach(int const&cID, initial_cID_quantum.uniqueKeys()) Q_sum += initial_cID_quantum.value(cID);
    foreach(int const&cID, initial_cID_quantum.uniqueKeys()) qStdOut() << " C" << cID << " : " << initial_cID_quantum.value(cID) << " bits (" << trimDouble(initial_cID_quantum.value(cID)/8.0) << " bytes)" << " : " << trimDouble(static_cast<double>(initial_cID_quantum.value(cID))/static_cast<double>(Q_sum))*100.0 << "%";
    qStdOut() << " ************** ************** ";

    //     step II: improve quantum distribution
    QMap<int,int> res_cID_quantum = improveQuantumDistrbution(initial_cID_quantum, dataPtr);

    // display improved quantum distribution
    qStdOut() << "algorithm result: improved quantum distribution";
    Q_sum = 0;
    foreach(int const&cID, res_cID_quantum.uniqueKeys()) Q_sum += res_cID_quantum.value(cID);
    foreach(int const&cID, res_cID_quantum.uniqueKeys()) qStdOut() << " C" << cID << " : " << res_cID_quantum.value(cID) << " bits (" << trimDouble(res_cID_quantum.value(cID)/8.0) << " bytes)" << " : " << trimDouble(static_cast<double>(res_cID_quantum.value(cID))/static_cast<double>(Q_sum))*100.0 << "%" ;
    qStdOut() << " ************** ************** ";

    return res_cID_quantum;
}

//     step II: improve quantum distribution
QMap<int, int> NC_DRR_QuantumAssignmentImproved::improveQuantumDistrbution(const QMap<int, int> &initialQunatumDistribution, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::improveQuantumDistrbution()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::improveQuantumDistrbution()");

    qStdOut() << " start : improve quantum distribution";

    QList<int> classes = NC_DRR::getClasses(); // fetch list of class IDs (in ascending order). NOTE : highest class ID is assumed to be non-critical class (Cn)
    if(classes.empty()) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::improveQuantumDistrbution()");
    int Cn = classes.takeLast(); // Cn = non-critical class    

    QMap<int, int> result_cID_quantum = initialQunatumDistribution;  // result : improved quantum distribution

    //     step II: improve quantum distribution (loop step III and step IV)
    while(true) {
        qStdOut() << "***** ***** step : minimise critical class service ***** ***** ";

        //     step III:  Minimise critical class service
        arrangeClasses_basedOnDeviation(classes, result_cID_quantum, dataPtr); // preconfiguration
        foreach(int const&Cx, classes) { // for each critical class
            int Q_Cx = reduceQuantumToMinimum(Cx, result_cID_quantum, dataPtr); // reduce quantum to minimum valid value
            qStdOut() << "C" << Cx << " reduced to " << Q_Cx << " from " << result_cID_quantum.value(Cx) << " bits. (in bytes " << Q_Cx/8.0 << " from " << result_cID_quantum.value(Cx)/8.0 << ")";

            // store results
            result_cID_quantum.insert(Cx, Q_Cx); // critical class
        }

        qStdOut() << "***** ***** step : maximise non-critical class service (loop : inc Q_Cn and adjust Q_Ci) ***** ***** ";

        //     step IV: Maximise non-critical class service (loop step IVa and step IVb)
        int prevQn = result_cID_quantum.value(Cn);
        QMap<int, int> new_cID_quantum = maximiseCnService(result_cID_quantum, dataPtr); // compute quantum distribution that maximise non-critical class service
        if(new_cID_quantum.value(Cn) == prevQn) break; // non-improvable quantum distribution

        qStdOut() << "C" << Cn << " increased to " << new_cID_quantum.value(Cn) << " from " << prevQn << " bits. (in bytes " << trimDouble(new_cID_quantum.value(Cn)/8.0) << " from " << trimDouble(prevQn/8.0) << ")";

        // store results
        result_cID_quantum = new_cID_quantum; // non-critical class
    }

    // return improved quantum distribution
    return result_cID_quantum;
}

//     step III:  Minimise critical class service
int NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum(const int &classID, const QMap<int, int> &refDistribution, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");

    // reduce the quantum of Cx to a minimum value that respect Cx deadline
    //      NOTE: refDistribution must be a valid distribution, i.e. it correspond to quanta that respect deadline in all critical classes and Q_Cn >= lMax_Cn
    // the computation is carried out in multiple steps:
    //     (1) compute maximum dealy D_max_Cx in Cx with the given Q_Cx
    //     (2) adjust Q_Cx
    //          if D_max_Cx is
    //              > Deadline_Cx then increase Q_Cx
    //              < Deadline_Cx then decrease Q_Cx
    //              = Deadline_Cx then stop

    if(!refDistribution.contains(classID)) throw std::runtime_error("Q_Cx not found. reference quantum distribution is invalid :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");
    int res_Q_Cx = refDistribution.value(classID); // result : minimum quantum required by Cx to respect deadline
    double res_deviation = -1; // result : D_max_Cx deviation (in %) with respect to deadline

    // configure DRR scheduler
    foreach (int const&cID, refDistribution.uniqueKeys()) { // for each class
        NC_DRR::cID_quantum_map.insert(cID, refDistribution.value(cID)); // update DRR scheduler
    }

    // set quantum search space
    int Q_Cx = refDistribution.value(classID);
    int up = Q_Cx;
    int down = cID_lMax_map.value(classID);

    if(up == down) return res_Q_Cx; // Q_Cx is already minimum

    double deadline_Cx = getDeadline(classID); // Cx deadline

    qStdOut() << "minimise C" << classID;

    while(true)
    {
        NC_DRR::cID_quantum_map.insert(classID,Q_Cx); // update DRR scheduler

        //     (1) compute maximum dealy D_max_Cx in Cx with the given Q_Cx
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // remove this foreach loop. its only for debug
        {
            qStdOut();
            qStdOutInLine() << "C(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << cID;
            }
            int Cn = getClasses().last();
            qStdOutInLine() << Cn << ") Q(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << trimDouble(cID_quantum_map.value(cID)/8.0) << ((cID==classID)?"*":"");
            }
            qStdOutInLine() << trimDouble(cID_quantum_map.value(Cn)/8.0) << " B) maxD(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                // check deviation with respect to deadline
                double diff = trimDouble((getDeadline(cID) - cID_maxE2E.value(cID))*100.0/getDeadline(cID));
                qStdOutInLine() << cID_maxE2E.value(cID) << " [" << diff << "%]";
            }
        }

        // verify that no concurrent class (!= Cx) misses its deadline
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == classID) continue; // skip Cx
            if(cID_maxE2E.value(cID) > getDeadline(cID)) {
                throw std::runtime_error("concurrent class C" + QString().setNum(cID).toStdString() + " missed deadline. something went wrong in the computation! :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");
            }
        }

        double maxE2E_Cx = cID_maxE2E.value(classID); // max delay in Cx

        // check deviation with respect to deadline
        double deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);

        //     (2) adjust Q_Cx
        if(maxE2E_Cx > deadline_Cx) {
            // Q_Cx must increase
            down = Q_Cx; // update search space
        }
        else if(maxE2E_Cx < deadline_Cx) {
            // Q_Cx must decrease
            up = Q_Cx; // update search space

            // acceptable value
            res_Q_Cx = Q_Cx;
            res_deviation = deviation;
        }
        else // maxE2E == deadline
        {
            // acceptable value
            res_Q_Cx = Q_Cx;
            res_deviation = deviation;
            break;
        }

        int prevQ_Cx = Q_Cx;
        //Q_Cx = floor((up+down)/2); // compute next quantum value within the search space. caution : computation in bits !
        Q_Cx = (floor(((up+down)/2)/8))*8;

        if(prevQ_Cx == Q_Cx) break; // Q_Cx converged
    }

    // check validity of the result
    if(res_deviation < 0) { // result invalid
        throw std::runtime_error("maxDelay < deadline. reference quantum distribution is invalid :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");
    }
    else // result valid
        qStdOut() << "//// end //// minimise C" << classID << " : deviation " << res_deviation << " %";

    return res_Q_Cx;
}

//     step IV: Maximise non-critical class service (loop step IVa and step IVb)
QMap<int, int> NC_DRR_QuantumAssignmentImproved::maximiseCnService(const QMap<int, int> &refDistribution, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::maximiseCnService()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::maximiseCnService()");

    //     (loop step IVa and step IVb)
    //          step IVa: Increase Cn quantum to maximum possible value
    //          step IVb: Increase the difference between the deadline and the maximum end-to-end delay for the critical class having smallest value of this difference.

    QMap<int,int> res_quantumDistribution = refDistribution; // result : quantum distribution with maximised Cn service

    QList<int> classes = NC_DRR::getClasses(); // fetch list of class IDs (in ascending order). NOTE : highest class ID is assumed to be non-critical class (Cn)
    if(classes.empty()) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::maximiseCnService()");
    int Cn = classes.takeLast(); // Cn = non-critical class

    //     (loop step IVa and step IVb)
    static bool firstIteration = true;
    while(true){

        QList<double> deviationFromDeadline; // stop condition parameter : list of deviation from deadline in critical classes

        //          step IVa: Increase Cn quantum to maximum possible value
        int Q_Cn = increaseQuantumToMaximum(Cn, res_quantumDistribution, dataPtr, deviationFromDeadline);
        // store results
        res_quantumDistribution.insert(Cn, Q_Cn); // non-critical class

        {
            // comment the next line to use the default stop condition
            // qStdOut() << " WRITE YOUR STOP CONDITION HERE";

            // default Stop condition :
            //      all the critical classes have a deviation from deadline less than 5%
            //      both operations increaseQuantumToMaximum() and incrementSmallestDifference() are executed once;
            const double acceptable_devation = 5.0;
            bool stopCondition = !firstIteration;
            foreach (double const &deviation, deviationFromDeadline) {
                if(deviation > acceptable_devation) {stopCondition = false; break; }
            }
            // end of default Stop condition

            if(stopCondition) { qStdOut() << "stopCondition satisfied!"; break;} // stop condition
        }

        //          step IVb: Increase the difference between the deadline and the maximum end-to-end delay for the critical class having smallest value of this difference.
        bool improvable = incrementSmallestDifference(res_quantumDistribution, dataPtr); // caution : res_quantumDistribution will be modified (when improvable == true)

        if(!improvable) break; // non-improvable quantum distribution
        firstIteration = false;
    }
    return res_quantumDistribution;
}

int NC_DRR_QuantumAssignmentImproved::increaseQuantumToMaximum(const int &classID, const QMap<int, int> &refDistribution, const NCData *dataPtr, QList<double> &res_deviationList)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::increaseQuantumToMaximum()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::increaseQuantumToMaximum()");

    // increase the quantum of non-critical class Cn to a maximum value that respect the deadline in all critical classes
    //      NOTE: refDistribution must be a valid distribution, i.e. it correspond to quanta that respect deadline in all critical classes and Q_Cn >= lMax_Cn
    // the computation is carried out in multiple steps:
    //     (1) increase Q_Cn untill at least one critical class misses its deadline
    //     (2) adjust Q_Cn
    //          if D_max in any critical class Cx is
    //              > Deadline_Cx then decrease Q_Cn
    //              < Deadline_Cx then increase Q_Cn
    //         continue the adjustment until Q_Cn is converged to a valid value.

    if(!refDistribution.contains(classID)) throw std::runtime_error("Q_Cn not found. reference quantum distribution is invalid :: NC_DRR_QuantumAssignmentImproved::reduceQuantumToMinimum()");
    int initial_Q_Cn = refDistribution.value(classID); // initial valid quantum for non-critical class Cn

    int res_Q_Cn = refDistribution.value(classID); // result : maximum value of a valid quantum for non-critical class Cn

    // configure DRR scheduler
    foreach (int const&cID, refDistribution.uniqueKeys()) { // for each class
        NC_DRR::cID_quantum_map.insert(cID, refDistribution.value(cID)); // update DRR scheduler
    }

    bool improvable = true; // is Q_Cn improvable
    int Q_Cn = refDistribution.value(classID); // Cn quantum

    qStdOut() << "maximise C" << classID << " " << trimDouble(Q_Cn/8.0) << " B";

    //     (1) increase Q_Cn untill at least one critical class misses its deadline
    const double incrementFactor = 1.5;
    do {
        NC_DRR::cID_quantum_map.insert(classID,Q_Cn); // update DRR scheduler

        // compute maximum dealy D_max_Cx in all critical classes Cx with the given quantum distribution
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // check if any critical class (!= Cn) misses its deadline
        QList<double> deviationList_temp;
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == classID) continue; // skip Cn
            if(cID_maxE2E.value(cID) > getDeadline(cID)) { improvable = false; break; }

            double deadline_Cx = getDeadline(cID);
            double maxE2E_Cx = cID_maxE2E.value(cID);
            double deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);
            deviationList_temp.append(deviation);
        }

        if(improvable) {
            res_Q_Cn = Q_Cn; // acceptable value
            res_deviationList = deviationList_temp; // acceptable value
            // Q_Cn = floor(Q_Cn * incrementFactor); // increase Q_Cn
            Q_Cn = (floor((Q_Cn * incrementFactor)/8))*8;
            qStdOutInLine() << " | increment " << trimDouble(Q_Cn/8.0) << " B ";
        }
    } while(improvable);

    // check if initial value is valid
    if(initial_Q_Cn == Q_Cn) throw std::runtime_error("critical class missed deadline with initial Qn! invalid initial Qn in :: NC_DRR_QuantumAssignmentImproved::increaseQuantumToMaximum()");

    // set quantum search space
    int up = Q_Cn; // set upper limit to the non-improvable value
    int down = res_Q_Cn;  // set lower limit to last valid Q_Cn value


    //     (2) adjust Q_Cn
    while(true) {
        //         continue the adjustment until Q_Cn is converged to a valid value.
        //Q_Cn = floor((up+down)/2); // compute next quantum value within the search space. caution : computation in bits !
        Q_Cn = (floor(((up+down)/2)/8))*8;
        if(Q_Cn == down) break; // Q_Cn converged

        NC_DRR::cID_quantum_map.insert(classID, Q_Cn); // update DRR scheduler

        // compute maximum dealy D_max_Cx in all critical classes Cx with the given quantum distribution
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // remove this foreach loop. its only for debug
        {
            qStdOut();
            qStdOutInLine() << "C(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << cID;
            }
            int Cn = classID;
            qStdOutInLine() << Cn << ") Q(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << trimDouble(cID_quantum_map.value(cID)/8.0);
            }
            qStdOutInLine() << trimDouble(cID_quantum_map.value(Cn)/8.0) << "* B) maxD(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                // check deviation with respect to deadline
                double diff = trimDouble((getDeadline(cID) - cID_maxE2E.value(cID))*100.0/getDeadline(cID));
                qStdOutInLine() << cID_maxE2E.value(cID) << " [" << diff << "%]";
            }
        }

        // check if any critical class (!= Cn) misses its deadline
        QList<double> deviationList_temp;
        double smallestDeviation = 1.0/0.0; // deviation with respect to deadline
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == classID) continue; // skip Cn
            double deadline_Cx = getDeadline(cID);
            double maxE2E_Cx = cID_maxE2E.value(cID);
            double deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);
            deviationList_temp.append(deviation);
            if(deviation < smallestDeviation) { smallestDeviation = deviation; }
        }

        //  if D_max in any critical class Cx is
        //      > Deadline_Cx then decrease Q_Cn
        //      < Deadline_Cx then increase Q_Cn
        if(smallestDeviation < 0) { // D_max_Cx > deadline_Cx
            // Q_Cn must decrease
            up = Q_Cn; // update search space
        }
        else if(smallestDeviation > 0) { // D_max_Cx < deadline_Cx
            // Q_Cn must increase
            down = Q_Cn; // update search space

            // acceptable value
            res_Q_Cn = Q_Cn;
            res_deviationList = deviationList_temp;
        }
        else // D_max_Cx == deadline_Cx
        {
            // acceptable value
            res_Q_Cn = Q_Cn;
            res_deviationList = deviationList_temp;
            break;
        }

    }

    qStdOut() << "//// end //// maximise C" << classID << " : res Q_Cn " << trimDouble(res_Q_Cn/8.0) << " bytes";
    return res_Q_Cn;
}

bool NC_DRR_QuantumAssignmentImproved::incrementSmallestDifference(QMap<int, int> &refDistribution, const NCData *dataPtr)
{
    // prerequisite
    if(dataPtr == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::incrementSmallestDifference()");
    if(!NC_DRR::configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::incrementSmallestDifference()");

    // increase the quantum of critical class Cx so that the difference (Diff) between its deadline and its maximum end-to-end delay is increased.
    //      Cx is the class having smallest difference between its deadline and its maximum end-to-end delay.
    //      NOTE: refDistribution must be a valid distribution, i.e. it correspond to quanta that respect deadline in all critical classes and Q_Cn >= lMax_Cn
    // the computation is carried out in multiple steps:
    //     (1) identify the critical class Cx that corresponds to smallest Diff
    //     (2) double the Q_Cx value. it will increase the difference Diff_Cx
    //     (3) if any critical class (Ci != Cx) missed its deadline then adjust Q_Cx. otherwise accept 2*initial(Q_Cx) as a valid result
    //              then adjust Q_Cx
    //                  if D_max in any critical class Ci is
    //                      > Deadline_Ci then decrease Q_Cx
    //                      < Deadline_Ci then increase Q_Cx
    //         continue the adjustment until Q_Cx is converged to a valid value between initial(Q_Cx) and 2*initial(Q_Cx).


    QMap<int,int> res_distribution = refDistribution; // result : qunatum distribution with incremented smallest difference
    double res_deviation = -1; // result (local) : difference between the Cx deadline and the maximum end-to-end delay).

    // configure DRR scheduler
    foreach (int const&cID, res_distribution.uniqueKeys()) { // for each class
        NC_DRR::cID_quantum_map.insert(cID, res_distribution.value(cID)); // update DRR scheduler
    }

    int Cn = getClasses().last(); // non-critical class ID

    //     (1) identify the critical class Cx that corresponds to smallest Diff (difference between the deadline and the maximum end-to-end delay).
    int Cx = -1;
    double largestDeviation = 0.0; // upper limit for Cx manipulation
    {
        // compute maximum dealy D_max_Cx in all critical classes Cx with the given quantum distribution
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        qStdOut();

        // check if any critical class (!= Cn) misses its deadline and fetch class ID having smallest deviation
        double smallestDeviation = 1.0/0.0; // deviation with respect to deadline
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == Cn) continue; // skip Cn
            double deadline_Cx = getDeadline(cID);
            double maxE2E_Cx = cID_maxE2E.value(cID);
            if(maxE2E_Cx > deadline_Cx) { throw std::runtime_error("critical class missed deadline with initial quantum distribution! invalid initial distribution in :: NC_DRR_QuantumAssignmentImproved::incrementSmallestDifference()"); }

            // fetch critical class having smallest difference between the deadline and the maximum end-to-end delay.
            double deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);
            if(deviation < smallestDeviation) { smallestDeviation = deviation; Cx = cID; }
            if(deviation > largestDeviation) { largestDeviation = deviation; }

            qStdOutInLine() << "C" << cID << " (" << trimDouble(cID_quantum_map.value(cID)/8.0) << " B)" << " Diff " << deviation;
        }
        if(Cx == -1) throw std::runtime_error("unable to compute smallest differenc. invalid initial distribution in :: NC_DRR_QuantumAssignmentImproved::incrementSmallestDifference()");

        qStdOut() << "manipulate margin C" << Cx << " Diff " << smallestDeviation << " %";
    }

    //     (2) double the Q_Cx value. it will increase the difference Diff_Cx
    const double initial_Q_Cx = res_distribution.value(Cx);
    // double Q_Cx = 2.0 * initial_Q_Cx;
    double Q_Cx = (floor((2.0*initial_Q_Cx)/8))*8;
    { // check validity of 2*initial(Q_Cx) as a result

        NC_DRR::cID_quantum_map.insert(Cx, Q_Cx); // update DRR scheduler

        // compute maximum dealy D_max_Ci in all critical classes Ci with the given quantum distribution
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // check if any critical class misses its deadline
        double smallestDeviation = 1.0/0.0; // deviation with respect to deadline
        double Cx_deviation = -1;
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == Cn) continue; // skip Cn
            double deadline_Ci = getDeadline(cID);
            double maxE2E_Ci = cID_maxE2E.value(cID);
            double deviation = trimDouble((deadline_Ci - maxE2E_Ci)*100.0/deadline_Ci);
            if(deviation < smallestDeviation) { smallestDeviation = deviation; }
            if(cID == Cx) Cx_deviation = deviation;
        }

        if(smallestDeviation >= 0 && Cx_deviation <= largestDeviation) { // if 2*initial(Q_Cx) is valid then accept it as a result
            // acceptable value
            res_distribution.insert(Cx, Q_Cx);
            refDistribution = res_distribution;

            double deadline_Cx = getDeadline(Cx);
            double maxE2E_Cx = cID_maxE2E.value(Cx);
            res_deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);
            qStdOut() << "//// end //// margin C" << Cx << " : Q doubled " << trimDouble(Q_Cx/8.0) << " bytes. Diff " << res_deviation << " %";
            return true;
        }
    } // Otherwise


    //     (3) adjust Q_Cx
    // set quantum search space
    int up = Q_Cx; // set upper limit to the non-improvable value
    int down = initial_Q_Cx;  // set lower limit to last valid Q_Cx value

    while(true){
        //         continue the adjustment until Q_Cx is converged to a valid value.
        // Q_Cx = floor((up+down)/2); // compute next quantum value within the search space. caution : computation in bits !
        Q_Cx = (floor(((up+down)/2)/8))*8;
        if(Q_Cx == down) break; // Q_Cx converged

        NC_DRR::cID_quantum_map.insert(Cx, Q_Cx); // update DRR scheduler

        // compute maximum dealy D_max_Ci in all critical classes Ci with the given quantum distribution
        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        // remove this foreach loop. its only for debug
        {
            qStdOut();
            qStdOutInLine() << "C(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << cID;
            }
            int Cn = getClasses().last();
            qStdOutInLine() << Cn << ") Q(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                qStdOutInLine() << trimDouble(cID_quantum_map.value(cID)/8.0) << ((cID==Cx)?"*":"");
            }
            qStdOutInLine() << trimDouble(cID_quantum_map.value(Cn)/8.0) << " B) maxD(";
            foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
                // check deviation with respect to deadline
                double diff = trimDouble((getDeadline(cID) - cID_maxE2E.value(cID))*100.0/getDeadline(cID));
                qStdOutInLine() << cID_maxE2E.value(cID) << " [" << diff << "%]";
            }
        }


        // check if any critical class misses its deadline
        double smallestDeviation = 1.0/0.0; // deviation with respect to deadline
        double Cx_deviation = -1;
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            if(cID == Cn) continue; // skip Cn
            double deadline_Ci = getDeadline(cID);
            double maxE2E_Ci = cID_maxE2E.value(cID);
            double deviation = trimDouble((deadline_Ci - maxE2E_Ci)*100.0/deadline_Ci);
            if(deviation < smallestDeviation) { smallestDeviation = deviation; }
            if(cID == Cx) Cx_deviation = deviation;
        }


        //      Qcx increase <-                    -> Qcx decreas
        //       +ve deviation     Deadline Cx     -ve deviation
        //                              |
        //      | largest deviation
        //                          | initial Cx deviation
        //   | Cx deviation      OR         | Cx/smallest deviation     we are here when
        //      ||||||||||||||||||||||||| valid
        //  Cx > largest : dec_Q
        //  Cx < largest : inc_Q
        //  smallest < 0 & smallest = Cx : inc_Q
        //  smallest < 0 & smallest != Cx : dec_Q
        //  smallest > 0 & smallest = Cx : check largest Cx
        //  smallest > 0 & smallest != Cx : check largest Cx

        //  if D_max in any critical class Ci (!= Cx) is
        //      > Deadline_Ci then must decrease Q_Cx
        //  if D_max in critical class Cx is
        //      > Deadline_Ci then must increase Q_Cx
        //  if D_max in any critical class is
        //      < Deadline_Ci then can increase/decrease Q_Cx based on largestDeviation
        //      if largestDeviation
        //          < Cx_deviation then decrease Q_Cx
        //          > Cx_deviation then increase Q_Cx
        if(smallestDeviation < 0) { // deadline miss
            if(smallestDeviation == Cx_deviation) { // D_max_Cx > deadline_Cx
                // Q_Cx must increase
                down = Q_Cx; // update search space

                qStdOutInLine() << " inc Q_C" << Cx;

            }
            else { // D_max_Ci > deadline_Ci
                // Q_Cx must decrease
                up = Q_Cx; // update search space

                qStdOutInLine() << " dec Q_C" << Cx;

            }
        } else if(smallestDeviation >= 0) { // D_max_all < deadline_all

            if(largestDeviation > Cx_deviation) { // valid case
                // Q_Cx must increase
                down = Q_Cx; // update search space

                qStdOutInLine() << " inc Q_C" << Cx;

                // acceptable value
                res_distribution.insert(Cx, Q_Cx);
                res_deviation = Cx_deviation;

            } else if(largestDeviation < Cx_deviation){ // invalid case
                // Q_Cx must decrease
                up = Q_Cx; // update search space

                qStdOutInLine() << " dec Q_C" << Cx;

            } else { // valid case
                // acceptable value
                res_distribution.insert(Cx, Q_Cx);
                res_deviation = Cx_deviation;

                break;
            }
        }

        //        //  if D_max in any critical class Ci is
        //        //      > Deadline_Ci then decrease Q_Cx
        //        //      < Deadline_Ci then increase Q_Cx
        //        if(smallestDeviation < 0) { // D_max_Ci > deadline_Ci
        //            // Q_Cx must decrease
        //            up = Q_Cx; // update search space
        //        }
        //        else if(smallestDeviation > 0) { // D_max_Ci < deadline_Ci
        //            // Q_Cn must increase
        //            down = Q_Cx; // update search space

        //            // acceptable value
        //            res_distribution.insert(Cx, Q_Cx);

        //            double deadline_Cx = getDeadline(Cx);
        //            double maxE2E_Cx = cID_maxE2E.value(Cx);
        //            res_deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);
        //        }
        //        else // D_max_Cx == deadline_Cx
        //        {
        //            // acceptable value
        //            res_distribution.insert(Cx, Q_Cx);

        //            double deadline_Cx = getDeadline(Cx);
        //            double maxE2E_Cx = cID_maxE2E.value(Cx);
        //            res_deviation = trimDouble((deadline_Cx - maxE2E_Cx)*100.0/deadline_Cx);

        //            break;
        //        }
    }

    if(Q_Cx == initial_Q_Cx) {
        qStdOut() << "//// end //// margin C" << Cx << " : non-improvable";
        return false;} // non-improvable distribution

    qStdOut() << "//// end //// margin C" << Cx << " : res " << trimDouble(Q_Cx/8.0) << " bytes. Diff " << res_deviation << " %";

    // update result
    refDistribution = res_distribution;
    return true; // improvable distribution
}

// arrange the classes in increasing/decreasing order of deviation with respect to deadline
void NC_DRR_QuantumAssignmentImproved::arrangeClasses_basedOnDeviation(QList<int> &classes, const QMap<int, int> &refDistribution, const NCData *dataPtr, bool increasing)
{
        qStdOut() << "arrange classes : ";

        // configure DRR scheduler
        foreach (int const&cID, classes) { // for each class
            NC_DRR::cID_quantum_map.insert(cID, refDistribution.value(cID)); // update DRR scheduler
        }

        NCData ncData_temp = *dataPtr; // create local copy of NCData
        DISPLAY_OUTPUT = false;
        QMap<int,double> cID_maxE2E = NC_Optimised::computeMaximumE2E(this, &ncData_temp); // compute maximum delay in all critical classes
        DISPLAY_OUTPUT = DISPLAY_OUTPUT_DEFAULT;

        QMultiMap<double, int> deviation_cID;
        foreach (int const& cID, cID_maxE2E.uniqueKeys()) {
            // check deviation with respect to deadline
            double diff = trimDouble((getDeadline(cID) - cID_maxE2E.value(cID))*100.0/getDeadline(cID));
            deviation_cID.insert(diff, cID);
        }

        classes = deviation_cID.values();
        if(!increasing) { // reverse classes list
            int count = classes.count();
            for(int i = 0; i < count; ++i) {
                classes.prepend(classes.takeAt(i));
            }
        }

        // remove this for loop its only for debug
        {
            foreach (int const &cID, classes) {
                qStdOutInLine() << "C" << cID << " (" << deviation_cID.key(cID) << " %)";
            }
            qStdOutInLine() << "[";
            foreach (int const &cID, classes) {
                qStdOutInLine() << refDistribution.value(cID);
            }
            qStdOutInLine() << refDistribution.value(getClasses().last()) << " B]";
        }
}

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************   NC_Optimised   ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //
//
// NC_Optimised is the re-implementation of NC_DRR_Optimised adapted for the quantum assignment algorithm.

QMap<int, double> NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeMaximumE2E(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeMaximumE2E()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeMaximumE2E()");

    // compute E2E delay only in critical classes
    //      NOTE : the non-critical class Cn is assumed to have "infinite traffic"
    //             this assumption allows computation of the quantum distribution without precise knowledge of non-critical traffic. only max frame size in Cn is needed.
    //             this assumption is implemented by considering that Cn consumes its full quantum Qn in each scheduling round.
    //
    // the E2E delay computation is divided into multiple steps
    //
    // Step 1 : select a flow (Vref)
    // Step 2 : select a path (Pi_Vref)
    // Step 3 : compute worst-case delay at each node in the path (a.k.a. feed forward)
    //        Note: in DRR scheduling, computation is "per class basis"
    //  Step 3a: compute overall arrival curve (alpha_o)
    //  Step 3b: compute service curve (beta_Vref)
    //  Step 3c: compute horizontal distance alpha_o and beta_Vref
    //  Step 3d: optimise the computed worst-case delay at the given node
    // Step 4 : compute e2e delay.

    QMap<int,double> res_cID_maxE2Edelay; // result : maximum end-to-end delay in each critical class

    int Cn = parent->getClasses().last();

    foreach(NCData::NCFlow const& flow, ncData->flows) { // Step 1 : select a flow (flow = Vref)

        int Cx = parent->getClassID(flow.getFlowID()); // fetch class ID
        if(Cx == Cn) continue; // skip non-critical class Cn flows

        int pathCount = flow.getPathCount();
        for(int pNo = 0; pNo < pathCount; ++pNo) {       // Step 2 : select a path (Pi_Vref)
            PATH const &path = flow.getPath(pNo);        // (path = Pi_Vref)

            double e2eDelay = 0;
            foreach (NODE_t const &node, path) {         // Step 3 : compute worst-case delay at each node in the path
                double delay = NC_Optimised::computeNodeDelay(parent, ncData, flow, node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

                e2eDelay += delay;                       // Step 4 : compute e2e delay.

                qStdOut() << flow.getFlowID() << " NODE " << NODE_t_toString(node) << " delay " << delay << " microSec";
            }
            qStdOut() << flow.getFlowID() << " path " << PATH_toString(path) << " E2E delay " << e2eDelay << " microSec";

            // update result
            if(res_cID_maxE2Edelay.value(Cx,-1) < e2eDelay) res_cID_maxE2Edelay.insert(Cx, e2eDelay);
        }
    }

//    foreach (int const &cID, res_cID_maxE2Edelay.uniqueKeys()) {
//        qDebug() << cID << " : " << res_cID_maxE2Edelay.value(cID) << " :: " << parent->cID_quantum_map.value(cID);
//    }
//    qDebug() << Cn << " :: " << parent->cID_quantum_map.value(Cn);

    return res_cID_maxE2Edelay;
}

// compute worst-case delay for refFlow at the given node
double NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeNodeDelay(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeNodeDelay()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeNodeDelay()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    double delay = 0;
    // check if the delay is already computed
    if(ncNode.getComputedDelay(node.second, refFlow.getFlowID(), delay)) {
        return delay;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
    //        Note: in DRR scheduling, computation is "per class basis"
    //  Step 3b: compute service curve (beta_Vref = beta_Cx)
    //  Step 3c: compute horizontal distance alpha_o and beta_Cx.
    //  Step 3d: optimise the computed worst-case delay at the given node
    //          it includes computation of over-estimated load from competing classes (Ci != Cx)
    //      NOTE : the non-critical class Cn is assumed to have "infinite traffic"
    //             this assumption allows computation of the quantum distribution without precise knowledge of non-critical traffic. only max frame size in Cn is needed.
    //             this assumption is implemented by considering (over-estimated load from Cn) = 0

    ConcaveCurve alpha_o = NC_Optimised::computeOverallArrivalCurve(parent, ncData, refFlow, node); //  Step 3a: compute overall arrival curve (alpha_o)
    ConvexCurve beta = NC_Optimised::computeServiceCurve(parent, ncData, refFlow, node);            //  Step 3b: compute service curve (beta_Cx)
    delay = maxHorizontalDistance(alpha_o, beta, refFlow.getFlowID(), node); //  Step 3c: compute horizontal distance alpha_o and beta_Vref.

    // store result
    ncNode.addComputedDelay(node.second, refFlow.getFlowID(), delay);

    delay = NC_Optimised::computeOptimisedNodeDelay(parent, ncData, refFlow, node);

    // store optimised result
    ncNode.addComputedDelay(node.second, refFlow.getFlowID(), delay);

    return delay;
}

//  Step 3a: compute overall arrival curve (alpha_o)
ConcaveCurve NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOverallArrivalCurve(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOverallArrivalCurve()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOverallArrivalCurve()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    ConcaveCurve alpha_o; // overall arrival curve
    // check if the overall arrival curve for the refFlow is already computed
    if(ncNode.getComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o)) {
        return alpha_o;
    }

    //  Step 3a: compute overall arrival curve (alpha_o = alpha_Cx)
    //      alpha_o computation is divided into multiple parts:
    //          Note: the overall arrival curve for Vref corresponds to the curve (alpha_Cx) of its corresponding class (Cx)
    //      (1) compute arival curve of each flow in Cx arriving at this output port.
    //          it includes "jitter" integration.
    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //      (2) compute cumulative curve from all the arrival curves.
    //          it includes "serialisation" integration.
    //          Note: AFDX end-systems implement only FIFO
    // ----------------------------------------------------------------------------


    //      (1) compute arival curve of each flow in Cx arriving at this output port.
    //                  jitter integration increases the burst (https://ieeexplore-ieee-org.gorgone.univ-toulouse.fr/document/5524098)
    //              alpha = rate*(t + jitter) + burst
    //                  burst = (max frame size)
    //                  rate = (max frame size)/BAG
    //                  increased burst = (rate * jitter) + burst
    //          Note: AFDX end-systems implement only FIFO
    const int Cx = parent->getClassID(refFlow.getFlowID());
    foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Cx at this output port
        if(ncNode.getNodeType() == NODE_TYPE::SW){
            if(parent->getClassID(fID) != Cx) continue;
        }
        NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
        double burst = flow.getLMax(); // burst = (max frame size)
        double BAG = flow.getBAG();
        double rate = trimDouble(flow.getLMax()/BAG); // rate = (max frame size)/BAG
        double jitter = computeJitter(parent, ncData, flow, node);
        burst = trimDouble(rate*jitter) + burst; // increased burst = (rate * jitter) + burst
        ConcaveCurve alpha(burst, rate);
        ncNode.addComputedArrivalCurve(node.second, fID, alpha);
    }

    //      (2) compute cumulative curve from all the arrival curves.
    //          at a switch output port
    //              based on serialisation effect: (https://hal.archives-ouvertes.fr/hal-02270458)
    //                  the flows sharing the same input port are serialised and cannot create a sum of burst at an output port.
    //              (a) the flows sharing the same input port are groupped together V_grp_input.
    //              (b) the burst due to V_grp_input is limited to maximum burst in a group. : maxBurst_V_group_input
    //              (c) the arrival rate is limited by the link rate : R
    //              (d) the arrival curve of V_grp_input is
    //                  alpha_grp_input = min(arrival_curve(rate = R, burst = maxBurst_V_group_input), sum(arrival curves of the flows in V_grp_input))
    //              (e) the overall arrival curve is
    //                  alpha_o = sum(alpha_grp_input1, alpha_grp_input2, ...)
    //          at an end-system (AFDX end-systems implement only FIFO)
    //                  alpha_o = sum of arrival curves of all the flows

    //          at an end-system (AFDX end-systems implement only FIFO))
    if(ncNode.getNodeType() == NODE_TYPE::ES) { // qStdOut() << "ES alpha_o " << NODE_t_toString(node);
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow at this output port
            ConcaveCurve alpha;
            if(ncNode.getComputedArrivalCurve(node.second, fID, alpha)) { // get arrival curve
                if (alpha_o.isBaseCurve()) alpha_o = alpha;
                else alpha_o += alpha; // alpha_o = sum of arrival curves of all the flows
            } else throw std::runtime_error("arrival curve not found :: computeOverallArrivalCurve()");
        }

        // store result
        ncNode.addComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o);
        return alpha_o;
    }

    //          at a switch output port (DRR scheduling)
    if(ncNode.getNodeType() == NODE_TYPE::SW) { // qStdOut() << "SW alpha_o " << NODE_t_toString(node);
        //              (a) the flows sharing the same input port are groupped together V_grp_input.
        QMultiMap<NODE_t, int> inputNode_fID_map; // <inputNode, flowID>
        foreach (int const&fID, ncNode.getFlowList(node.second)) { // for each flow in Cx at this output port
            if(parent->getClassID(fID) != Cx) continue;
            NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID); // fetch flow with selected fID
            NODE_t inputNode = flow.getPreviousNode(node); // fetch input node
            inputNode_fID_map.insert(inputNode, fID); // group fIDs based on input node
        }

        // make group arrival curve
        QList<ConcaveCurve> groupArrivalCurves; // list of group arrival curves
        foreach (NODE_t const&inputNode, inputNode_fID_map.uniqueKeys()) {
            //          (b) the burst due to V_grp_input is limited to maximum burst in a group. : maxBurst_V_group_input
            double maxBurst = 0.0;

            ConcaveCurve alpha_sum; // sum(arrival curves of the flows in V_grp_input)
            foreach (int const&fID, inputNode_fID_map.values(inputNode)) {
                ConcaveCurve alpha;
                if(ncNode.getComputedArrivalCurve(node.second, fID, alpha)) { // get arrival curve
                    if(alpha_sum.isBaseCurve()) alpha_sum = alpha;
                    else alpha_sum += alpha; // make sum
                    double burst = alpha.getBurstValue();
                    if(maxBurst < burst) maxBurst = burst; // get maxBurst
                } else throw std::runtime_error("arrival curve not found :: computeOverallArrivalCurve()");
            }

            //          (c) the arrival rate is limited by the link rate : R
            double R = ncNode.getLinkRate(node.second);

            //          (d) the arrival curve of group is alpha_grp_input = min(alpha(maxBurst, R), alpha_sum)
            ConcaveCurve alpha(maxBurst, R);
            groupArrivalCurves.append(min(alpha,alpha_sum));
        }

        //              (e) the overall arrival curve is
        //                  alpha_o = sum(alpha_grp_input1, alpha_grp_input2, ...)
        foreach (ConcaveCurve const&alpha_grp, groupArrivalCurves) {
            if(alpha_o.isBaseCurve()) alpha_o = alpha_grp;
            else alpha_o += alpha_grp; // make sum
        }

        // store result
        ncNode.addComputedOverallArrivalCurve(node.second, refFlow.getFlowID(), alpha_o);
        return alpha_o;
    }

    return alpha_o;
}

// compute the jitter upper bound for refFlow at the given node
double NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeJitter(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeJitter()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeJitter()");

    //          Note: jitter computation require worst-case delay computed at previous nodes (which require recursive computations)
    //  (cite :: ...)
    //   jitter for a flow V_ref at a node h is computed as:
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
        sum_Dmax += computeNodeDelay(parent, ncData, refFlow, prevNode); // make sum

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
ConvexCurve NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeServiceCurve(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeServiceCurve()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeServiceCurve()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  Step 3b: compute service curve (beta_Vref = beta_Cx)
    //          at a switch output port
    //               beta_Cx computation is divided into multiple parts:
    //                  Note: the service of Vref corresponds to the service (beta_Cx) provided to its corresponding class (Cx)
    //              (1) compute residual service rate rho_Cx
    //              (2) compute scheduler latency theta_Cx
    //              (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
    //
    //          at an end-system (AFDX end-systems implement only FIFO)
    //              beta_Vref computation is straight forward in FIFO.
    //              beta = R[t - sl]+
    // ----------------------------------------------------------------------------

    //          at an end-system
    if(ncNode.getNodeType() == NODE_TYPE::ES) {
        //              beta_Vref computation is straight forward in FIFO.
        //              beta = R[t - sl]+
        double R = ncNode.getLinkRate(node.second);
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta(sl,R);
        return beta;
    }

    //          at a switch output port
    if(ncNode.getNodeType() == NODE_TYPE::SW) {
        //      (1) compute residual service rate rho_Cx
        //              rho_Cx = (quantum_Cx / sum of quanta of all the classes) linkRate
        const int Cx = parent->getClassID(refFlow.getFlowID());
        double Q_Cx = parent->cID_quantum_map.value(Cx);
        double Q_sum = 0.0;
        foreach (double const&Q_Ci, parent->cID_quantum_map.values()) {
            Q_sum += Q_Ci; // make sum
        }
        double R = ncNode.getLinkRate(node.second);
        double rho_Cx = trimDouble(trimDouble(Q_Cx/Q_sum) * R);

        //      (2) compute scheduler latency
        //          scheduler latency is divided into two parts: (https://ieeexplore.ieee.org/document/8603222)
        //          (a) X_Cx = sum(Q_Ci + maximum deficit of Ci) / R       (Caution :  Ci != Cx)
        //                  maximum deficit of Ci = (max frame size in bytes) - 1 (in bytes) = (max frame size in bits) - 8 (in bits)
        //          (b) Y_Cx = ((Q_Cx - max deficit of Cx) + sum(Q_Ci))/R - (Q_Cx - max deficit of Cx)/rho_Cx
        //            theta_Cx = X_Cx + Y_Cx
        double X_Cx = 0.0;
        foreach (int const&Ci, parent->cID_quantum_map.keys()) {
            if(Ci == Cx) continue;
            double Q_Ci = parent->cID_quantum_map.value(Ci); // in bits
            double lMax_Ci = parent->cID_lMax_map.value(Ci); // in bits
            X_Cx += trimDouble((Q_Ci + lMax_Ci - 8)/R);
        }

        double Y_Cx = 0.0;
        double lMax_Cx = parent->cID_lMax_map.value(Cx); // in bits
        Y_Cx += trimDouble((Q_Cx - (lMax_Cx - 8) + (Q_sum - Q_Cx))/R);
        Y_Cx -= trimDouble((Q_Cx - (lMax_Cx - 8))/rho_Cx);

        //            theta_Cx = X_Cx + Y_Cx
        double theta_Cx = X_Cx + Y_Cx;

        //      (3) residual service for C_x flows is beta_Cx = rho_Cx [t - sl - theta_Cx]+
        double sl = ncNode.getSwitchingLatnecy();
        ConvexCurve beta((sl+theta_Cx),rho_Cx);
        return beta;
    }

    throw std::runtime_error("Unable to compute service curve. Invalid node type. :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeServiceCurve()");
}

//  Step 3d: optimise the computed worst-case delay at the given node
double NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOptimisedNodeDelay(NC_DRR_QuantumAssignmentImproved *parent, NCData *ncData, const NCData::NCFlow &refFlow, const NODE_t &node)
{
    // prerequisite
    if(ncData == nullptr) throw std::runtime_error("Invalid NCData in :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOptimisedNodeDelay()");
    if(!parent->configReady) throw std::runtime_error("Invalid DRR config :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOptimisedNodeDelay()");
    NCData::NCNode &ncNode = ncData->selectNCNode_ref(node.first);

    //  for a flow (Vref) having a service curve beta_Cx and an overall arrival curve alpha_Cx
    //  whose pessimistic delay computed by classical approach is D_Vref
    //  the optimised delay computation is divided into multiple parts:    (https://ieeexplore.ieee.org/document/8603222)
    //       (1) compute service load SL_Ci of competing class Ci
    //              it is the amount of Ci traffic considered by beta_Cx in the duration [0, D_servicepoint]
    //       (2) compute actual load L_Ci of competing class Ci
    //              it is the upper-bound on the amount of traffic that can arrive in Ci in the duration [0, D_servicepoint]
    //       (3) compute over-estimated load of competing class Ci
    //              it is the difference between the service load SL_Ci and the actual load L_Ci
    //       (4) compute optimised delay delay_opt
    //              it is obtained by substracting the transmission time of the over-estimated load of each cometing class
    //              from the delay D_Vref
    //
    //      NOTE : the non-critical class Cn is assumed to have "infinite traffic"
    //             this assumption allows computation of the quantum distribution without precise knowledge of non-critical traffic. only max frame size in Cn is needed.
    //             this assumption is implemented by considering (over-estimated load from Cn) = 0
    //
    //
    //   D_servicepoint is the horizontal distance between x-axis and the point on service curve
    //   where worst-case delay D_Vref is computed.
    //          it is >= worst-case delay
    //
    //  a supplementary optimisation is applied to compute tighter delay bound (https://ieeexplore.ieee.org/abstract/document/8756011)
    //     it based on the consideration that the transmission rate for Cx flows
    //     is equal to the link rate (R) but not the residual service rate (rho_Cx)
    //       (5) introduce supplementary optimisation
    //             the classical NC approach assumes rate-latency curve
    //             this optimisation removes the horizontal difference between
    //             the staircase curve and the rate-latency curve in the last service round
    //
    //  Note: the optimisation is applicable only to the output ports implementing DRR scheduling
    // ----------------------------------------------------------------------------

    //          at an end-system
    if(ncNode.getNodeType() == NODE_TYPE::ES) { // (AFDX end-systems implement only FIFO)
        return NC_Optimised::computeNodeDelay(parent, ncData, refFlow, node);
    }

    //          at a switch output port
    if(ncNode.getNodeType() == NODE_TYPE::SW) {

        int Cn = parent->getClasses().last(); // fetch non-critical class ID

        // compute service point
        ConcaveCurve alpha_Cx = NC_Optimised::computeOverallArrivalCurve(parent, ncData, refFlow, node);
        ConvexCurve beta_Cx = NC_Optimised::computeServiceCurve(parent, ncData, refFlow, node);
        double D_servicepoint = maxHorizontalDistance_X0(alpha_Cx, beta_Cx, refFlow.getFlowID(), node);

        // prerequisite
        const int Cx = parent->getClassID(refFlow.getFlowID()); // fetch Vref class ID
        double Q_sum = 0.0;                             // compute sum of quanta
        foreach (double const&Q_Ci, parent->cID_quantum_map.values()) {
            Q_sum += Q_Ci; // make sum
        }
        double R = ncNode.getLinkRate(node.second);     // fetch link rate
        double lMax_Cx = parent->cID_lMax_map.value(Cx);        // in bits

        double X_Cx = 0.0; // compute X_Cx
        foreach (int const&Cj, parent->cID_quantum_map.keys()) {
            if(Cj == Cx) continue;
            double Q_Cj = parent->cID_quantum_map.value(Cj); // in bits
            double lMax_Cj = parent->cID_lMax_map.value(Cj); // in bits
            X_Cx += trimDouble((Q_Cj + lMax_Cj - 8)/R);
        }

        double overestimatedLoad = 0.0;
        foreach (int const&Ci, parent->cID_quantum_map.keys()) { // for each competing class Ci
            if(Ci == Cx) continue;

            if(Ci == Cn) continue; //   NOTE : the non-critical class Cn is assumed to have "infinite traffic". this assumption is implemented by considering (over-estimated load from Cn) = 0

            //       (1) compute service load SL_Ci of competing class Ci
            //              it is the amount of Ci traffic considered by beta_Cx in the duration [0, D_servicepoint]
            //          SL_Ci = 0                                               , D_servicepoint < X_Cx
            //                = Q_Ci + (max_deficit_Ci)                         , X_Cx <= D_servicepoint < t_N
            //                = Q_Ci + (max_deficit_Ci) + ( 1 + floor(Z) ) Q_Ci , t_N <= D_servicepoint
            //
            //      Z = R * (D_servicepoint - t_N) / Q_sum
            //      t_N = X_Cx + (Q_sum - max_deficit_Cx) / R


            // compute t_N = X_Cx + (Q_sum - max_deficit_Cx) / R
            double t_N = X_Cx + trimDouble((Q_sum - lMax_Cx - 8)/R);

            // compute SL_Ci
            double SL_Ci = 0.0;
            if(D_servicepoint < X_Cx) {
                SL_Ci = 0.0;
            } else if (D_servicepoint < t_N) {
                SL_Ci = trimDouble(parent->cID_quantum_map.value(Ci) + parent->cID_lMax_map.value(Ci) - 8);
            } else { // D_servicepoint >= t_N
                // compute Z = R * (D_servicepoint - t_N) / Q_sum
                double Z = trimDouble(R * trimDouble((D_servicepoint - t_N)/Q_sum));
                SL_Ci = trimDouble(parent->cID_quantum_map.value(Ci) + parent->cID_lMax_map.value(Ci) - 8);
                SL_Ci += trimDouble((1 + floor(Z))*parent->cID_quantum_map.value(Ci)); // ( 1 + floor(Z) ) Q_Ci
            }

            //       (2) compute actual load L_Ci of competing class Ci
            //              it is the upper-bound on the amount of traffic that can arrive in Ci in the duration [0, D_servicepoint]
            //              it can be obtained from overall arrival curve of any flow in Ci as
            //                  L_Ci = alpha_Ci(D_servicepoint)
            double L_Ci = 0.0;
            int fID_Ci = -1;
            foreach(const int &fID, ncNode.getFlowList(node.second)) { // fetch flowID of any Ci flow at the given node
                if(parent->getClassID(fID) == Ci) {fID_Ci = fID; break; }
            }
            if(fID_Ci == -1) { L_Ci = 0.0; }
            else {
                NCData::NCFlow const &flow = ncData->selectNCFlow_ref(fID_Ci); // fetch flow with selected fID
                ConcaveCurve alpha_Ci = NC_Optimised::computeOverallArrivalCurve(parent, ncData, flow, node); // fetch overall arrival curve

                // compute L_Ci = alpha_Ci(D_servicepoint)
                RayList rl_Ci = alpha_Ci.getRayList();
                RayList::iterator it = rl_Ci.begin(); RayList::iterator itEnd = rl_Ci.end();
                for(; it != itEnd; ++it) { // look for the line corresponding to D_servicepoint
                    if(it->p.x > D_servicepoint) break;
                } --it;
                L_Ci = trimDouble(it->k * (D_servicepoint - it->p.x)) + it->p.y; // fetch y-value at x = D_servicepoint
            }

            //       (3) compute over-estimated load of competing class Ci
            //              it is the difference between the service load SL_Ci and the actual load L_Ci
            double overestimatedLoad_Ci = trimDouble(SL_Ci - L_Ci);

            if(overestimatedLoad_Ci < 0) { // service load is less than the actual traffic : no overestimated traffic.
                overestimatedLoad_Ci = 0;
            }
            overestimatedLoad += overestimatedLoad_Ci;
        }

        //       (4) compute optimised delay delay_opt
        //              it is obtained by substracting the transmission time of the over-estimated load of each cometing class
        //              from the delay D_Vref
        double D_Vref = NC_Optimised::computeNodeDelay(parent, ncData, refFlow, node);
        double delay_opt = trimDouble(D_Vref - trimDouble(overestimatedLoad/R));

        //       (5) introduce supplementary optimisation (https://ieeexplore.ieee.org/abstract/document/8756011)
        //             the classical NC approach assumes rate-latency curve with service rate rho_Cx
        //             this optimisation considers staircase curve with transmission rate of Cx flows equal to the link rate R
        //             it comes down to removal of horizontal distance between
        //                  the staircase curve (rate R) and the rate-latency curve (rate rho_Cx) in the last service round
        //             from the delay D_Vref.
        //           (a) indentify the last service round of Cx in the duration [0, D_servicepoint]
        //           (b) remove the horizontal distance between staircase curve and the rate-latency curve in the last service round.
        //                  horizontal distance = (Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R , when last round != first round
        //                                       = (Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R - (theta_Cx - X_Cx), when last round == first round
        //           (c) compute optimised delay
        //              Dopt = D_Vref - (((b-a)/rho) - ((b-a)/R))                         when b > (Q_Cx - max_deficit_Cx)
        //              Dopt = D_Vref - (((b-a)/rho) - ((b-a)/R)) + (theta_Cx - X_Cx)     when b <= (Q_Cx - max_deficit_Cx)
        //                  b = total bytes served in Cx
        //                  a = total bytes served in Cx until the begining of last round
        //                  b = rho_Cx (D_servicepoint - theta_Cx)
        // ----------------------------------------------------------------------------

        //           (a) indentify the last service round of Cx in the duration [0, D_servicepoint]
        int rounds = 1;
        double rho_Cx = beta_Cx.getInitialRateValue(); // fetch residual service rate of Cx
        double theta_Cx = beta_Cx.getLatencyValue(); // fetch latency of Cx
        double Q_Cx = parent->cID_quantum_map.value(Cx);
        double max_deficit_Cx = trimDouble(parent->cID_lMax_map.value(Cx) - 8);

        double bitsServed_Cx = trimDouble((D_servicepoint - theta_Cx)*rho_Cx); // compute the amount of Cx data transmitted in the duration [0, D_servicepoint] :: y1 = 0, y2 = (x2-x1)*slope
        bitsServed_Cx = round(bitsServed_Cx); // To prevent bits in decimal

        double bitsServed_beforeLastRound_Cx = 0; // compute the amount of Cx data transmitted before the begining of the last service round.
        if(bitsServed_Cx > (trimDouble(Q_Cx - max_deficit_Cx))) {
            int n = ceil(trimDouble((bitsServed_Cx - (Q_Cx - max_deficit_Cx))/Q_Cx));
            bitsServed_beforeLastRound_Cx = trimDouble(Q_Cx - max_deficit_Cx) + trimDouble((n-1)*Q_Cx);
            rounds += n;
        }

        double bitsServed_lastRound_Cx = trimDouble(bitsServed_Cx - bitsServed_beforeLastRound_Cx); // compute the amount of Cx data transmitted in the last round

        //           (b) remove the horizontal difference between staircase curve and the rate-latency curve in the last service round.
        //                  horizontal distance = (Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R , when last round != first round
        //                                       = ((Cx bytes served in last round)/rho_Cx - (Cx bytes served in last round)/R) + (theta_Cx - X_Cx), when last round == first round
        double hDistance = 0.0;
        if(rounds > 1) { hDistance = trimDouble(bitsServed_lastRound_Cx * trimDouble((R - rho_Cx)/(R*rho_Cx))); }
        else {hDistance = trimDouble(bitsServed_lastRound_Cx * trimDouble((R - rho_Cx)/(R*rho_Cx))) + (theta_Cx - X_Cx);}

        //           (c) compute optimised delay
        delay_opt -= hDistance;
        return delay_opt;
    }

    throw std::runtime_error("Unable to compute optimised delay. Invalid node type. :: NC_DRR_QuantumAssignmentImproved::NC_Optimised::computeOptimisedNodeDelay()");

}
