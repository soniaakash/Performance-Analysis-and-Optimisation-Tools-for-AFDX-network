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

#ifndef NCCURVES_H
#define NCCURVES_H

#include <QVector>

#include "SharedData/global_typedef.h"

// the arrival curves and service curves are the piecewise linear functions
//      each "piece" (called "line") of these functions is made up of:
//      (1) a starting "point" (x,y)
//      (2) a "slope" (k) in forward (time) direction
//
struct Point {
    Point() = default;
    Point(double x, double y) : x(x), y(y) {}

    bool operator==(Point const &rhs) const { return ((x == rhs.x) && (y == rhs.y)); }

    double x, y;
};

// a "piece" of a piecewise linear function
struct Line {
    Line() = default;
    Line(Point p, double k) : p{p}, k{k} {}
    Line(double x, double y, double k) : p{x,y}, k{k} {}

    bool operator==(Line const &rhs) const { return ((p == rhs.p) && (k == rhs.k)); }

    Point p;
    double k;
};

// in arrival curves:
//      the slope is known as "rate"
//      the (non-zero) starting point is known as "burst"
//      Note: arrival curve starts at origin (zero point)
struct RateBurst {
    RateBurst() = default;
    RateBurst(double r, double b) : rate{r}, burst{b} {}

    double rate;
    double burst;
};

// in service curve:
//      the slope is known as "rate"
//      the (non-zero) starting point is known as "latency"
//      Note: service curve starts at origin (zero point)
struct RateLatency {
    RateLatency() = default;
    RateLatency(double rate, double latency) :
        rate(rate), latency(latency) {}

    double rate, latency;
};

// a piecewise linear function is defined as a continuous sequence of "lines" called "RayList"
using RayList = QList<Line>;
using RateBurstList = QList<RateBurst>;
using RateLatencyList = QList<RateLatency>;

// validate the line (x,y):k
//      (x,y) shound be finite and >= 0
//      k should be >=0
bool validateRay(const Line &l);

// add two piecewise linear functions. (only concave)
//      it is used to find sum of two arrival curves.
//        if(rl1.x == rl2.x) => merge, otherwise => insert both rl1.x and rl2.x
//        sum of y is computed based on relative x position and slope values
RayList addRayList( RayList const&rl1, RayList const&rl2);

// minimum of the two piecewise linear functions. (only concave)
//      based on (min,+) algebra https://en.wikipedia.org/wiki/Tropical_semiring)
//      Caution : res may NOT be continous. slopes of res MUST BE ADJUSTED SEPARATELY to make it continuous
void minRayList(const RayList &rl1, const RayList &rl2, RayList &res);

// maximum of the two piecewise linear functions. (only wide-sense increasing)
//      based on (min,+) algebra https://en.wikipedia.org/wiki/Tropical_semiring)
//      Caution : res may NOT be continous. slopes of res MUST BE ADJUSTED SEPARATELY to make it continuous
void maxRayList(const RayList &rl1, const RayList &rl2, RayList &res);

// adjust slopes of a piecewise linear function
//      it is used to make arrival curves continous afer min/max operations
void makeContinuous(RayList &rl);

// adjust slopes of a piecewise linear function and eliminate "pieces" which induce non-concavity
//      it is used to make aggregated arrival curves concave afer makeAggregatedArrivalCurve()/max() operations
void makeConcave(RayList &rl);

// print: for debuging purpose
void printRayList(RayList const &rL);

/*
 *  The concaveCurve is a curve formed by
 *  a sequence of rays ordered by decreasing slopes.
 *
 * Concave curves are build from either:
 *  - list of rays, or
 *  - a rate-burst pairs (which is a unique feature of concaveCurve).
 *
 * Elements (r,b) of a valid rate-burst pair
 *  shall belong to the following domain:
 *  r >= 0 && 0 <= b < inf
 *
 * Pair (r,b) corresponds to a bi-segment curve:
 *  - the first one starts at origin and coinsides with 0Y axis;
 *  - the second one starts at (0,b) and has a slope equal to r.
 *  * on introducing offset, this curve is moved on x-axis by offset value
 *
 * an arrival curve is a causal concaveCurve
 */
class ConcaveCurve {
public:
    // by default each arrival curve is assumed to start at (0,0) with slope +inf
    ConcaveCurve() { makeBaseCurve(); }

    // Base curve + predefined initial burst and initial rate
    ConcaveCurve(double initialBurst, double initialRate);

    // add line l at the end of the curve (assures concativity with respect to last "piece" of curve)
    void appendRay(Line const&l);

    // a concave curve with offset starts at (0,0) but it has an initial burst at (x = offset) followed by an initialRate
    // the segment (0,0) to (offset,0) makes it non-concave. starting from (offset,0), the rest is concave.
    void setOffset(double const &offset) {
        if(offset <= 0) return; // invalid offset value
        auto it = rayList.begin(); auto itEnd = rayList.end();
        while(++it != itEnd) { it->p.x += offset; }  // right shift the curve by offset value. except point (0,0)
        rayList.insert(1, Line(Point(0,10.0/pow(10, 10)), 0.0)); // redundant point to eliminate error in sum operation
        makeContinuous(rayList);
    }

    // aggregated arrival curve is the sum of arrival curves right-shifted by unique offset values.
    //      Caution : res may NOT be concave. concavity can be established separately using makeConcave().
    void makeAggregatedArrivalCurve(QList<ConcaveCurve> const &curves_withOffset) {
        if (curves_withOffset.empty()) throw std::runtime_error("empty list of arrival curves. unable to compute aggregated arrival curve!");

        // compute sum of arrival curves
        //      NOTE: concavity validation is ignored since it may not exist in an aggregated arrival curve
        this->makeBaseCurve(); // reset raylist to BaseCurve
        foreach (ConcaveCurve const &alpha, curves_withOffset) {
            if(this->isBaseCurve()) this->rayList = alpha.getRayList();
            else this->rayList = addRayList(this->getRayList(), alpha.getRayList()); // make sum
            makeContinuous(this->rayList); // make sum
        }
    }

    // fetch the curve
    RayList const& getRayList() const { return rayList; }

    // overwrite the curve with the given rayList. (sequentially appendRay())
    void setRayList(RayList const &rL);

    // verify concavity if line "l" is inserted at position "pos". used by appendRay();
    bool validateConcavity(Line const&l, int const&pos) const;

    // verify continuity if line "l" is inserted at position "pos". used by appendRay();
    bool validateContinuity(Line const&l, int const&pos) const;

    // verify causality if line "l" is inserted at position "pos = 0". used by appendRay();
    bool validateCausality(Line const&l) const;

    // get initial burst (irrespective of offset)
    double getBurstValue() const { auto it = rayList.begin(); while(it != rayList.end()) {if(it->p.y > 0.0) return it->p.y; ++it;} return 0.0; }

    // get initial rate (after initial burst)
    double getInitialRateValue() const { auto it = rayList.begin(); while(it != rayList.end()) {if(it->p.y > 0.0) return it->k; ++it;} return 0.0; }

    // get final rate
    double getFinalRateValue() const { if(!rayList.empty()) return rayList.last().k; return 0.0; }

    // get offset
    double getOffsetValue() const { auto it = rayList.begin(); while(it != rayList.end()) {if(it->p.y > 0.0) { return it->p.x;} ++it;} return 0.0; }

    // addition of two arrival curves (based on (min,+) algebra https://en.wikipedia.org/wiki/Tropical_semiring)
    ConcaveCurve operator + (const ConcaveCurve &cc2) const;
    void operator += (const ConcaveCurve &cc2 );

    // a based curve added to any curve results in a non-causal curve
    bool isBaseCurve() { return (rayList.size() == 1) && (rayList.first() == Line(Point(0.0,0.0),1.0/0.0)); }

private:
    RayList rayList; // concave curve is defined as a continuous sequence of "lines" called "RayList"

    // by default each arrival curve is assumed to start at (0,0) with slope +inf
    void makeBaseCurve() { rayList.clear(); rayList.append(Line(Point(0.0,0.0),1.0/0.0)); /* Base Curve */ }
    // by default each arrival curve with offset is assumed to start at (0,0) with slope 0
    // void makeBaseCurveWithOffset(double offset) { rayList.append(Line(Point(0.0,0.0),0.0)); rayList.append(Line(Point(offset,0.0),1.0/0.0)); /* Base Curve */ }

};

/*
 *  The convexCurve is a curve formed by
 *  a sequence of rays ordered by increasing slopes.
 *
 * Convex curves are build from either:
 *  - list of rays, or
 *  - a rate-latency pairs (which is a unique feature of convexCurve).
 *
 * Elements (r,l) of a valid rate-latency pair
 *  shall belong to the following domain:
 *  r >= 0 && 0 <= l < inf
 *
 *
 * The implementation of ConvexCurve in this projected
 *      is limited to a latency-rate server.
 *
 */

class ConvexCurve {
public:
    // by default each service curve is assumed to start at (0,0) with slope +inf
    ConvexCurve() { makeBaseCurve(); }

    // Base curve + predefined initial latency and initial rate
    ConvexCurve(double initialLatency, double initialRate);

    // fetch the curve
    RayList const& getRayList() const { return rayList; }

    // overwrite the curve with the given rayList. (assures latency-rate server service curve)
    void setRayList(RayList const &rL);

    // overwrite the curve with the given rayList.
    void setRayList_piecewiseConvex(RayList const &rL);

    // verify concavity if line "l" is inserted at position "pos". used by setRayList();
    static bool validateLatencyRateServer(RayList const&rL);

    // get latency
    double getLatencyValue() const { auto it = rayList.begin(); while(it != rayList.end()) {if(it->p.x > 0.0) { if(it->p.y == 0.0) return it->p.x; else return 0.0; } ++it;} return 0.0; }

    // get initial rate
    double getInitialRateValue() const { if(!rayList.empty() && getLatencyValue() == 0.0) { return rayList.first().k; } auto it = rayList.begin(); while(it != rayList.end()) {if(it->p.x > 0.0 && it->p.y == 0.0) { return it->k; }  ++it;}  return 0.0; }

private:
    RayList rayList; // convex curve is defined as a continuous sequence of "lines" called "RayList"

    // by default each service curve is assumed to start at (0,0) with slope 0
    RayList const& makeBaseCurve() { rayList.clear(); rayList.append(Line(Point(0.0,0.0),0.0)); return rayList; /* Base Curve */ }

};

// *** Curve Operations ***
// minimum of the two ConcaveCurve.
//      it is used to integrate serialization in overall arrival curve (https://hal.archives-ouvertes.fr/hal-02270458)
ConcaveCurve min(ConcaveCurve const &conc1, ConcaveCurve const &conc2);

// maximum horizontal distance between overall arrival curve and service curve (i.e. worst-case delay)
//       the computation of max horizontal distance in this function
//       is limited to a latency-rate server.
double maxHorizontalDistance(ConcaveCurve const &arrivalCurve, ConvexCurve const &serviceCurve, int const &flowID, NODE_t const &node);


// maximum horizontal distance between overall arrival curve and service curve (i.e. worst-case delay)
//       the computation of max horizontal distance in this function
//       is applicable to NON latency-rate server as welll.
double maxHorizontalDistance_nonLR(ConcaveCurve const &arrivalCurve, ConvexCurve const &serviceCurve, int const &flowID, NODE_t const &node);


// horizontal distance between x-axis and the point on service curve where worst-case delay is computed
//          it is >= worst-case delay
//          it is used only in optimised NC approach with DRR scheduling (https://ieeexplore.ieee.org/abstract/document/8603222/) and WRR scheduling (https://doi.org/10.1145/3273905.3273925)
// the computation in this function
//          is limited to a latency-rate server.
double maxHorizontalDistance_X0(ConcaveCurve const &arrivalCurve, ConvexCurve const &serviceCurve, int const &flowID, NODE_t const &node);


#endif // NCCURVES_H
