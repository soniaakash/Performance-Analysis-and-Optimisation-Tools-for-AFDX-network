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

#include "nccurves.h"

//      (x,y) shound be finite and >= 0
//      k should be >=0
bool validateRay(const Line &l)
{
    if (!std::isfinite(l.p.x) || l.p.x < 0.0) { qStdOut() << "x = " << l.p.x; throw std::runtime_error("invalid x");}
    if (!std::isfinite(l.p.y) || l.p.y < 0.0) { qStdOut() << "y = " << l.p.y; throw std::runtime_error("invalid y");}
    if (!std::isgreaterequal(l.k, 0.0)) { qStdOut() << "k = " << l.k; throw std::runtime_error("invalid k");}

    return true;
}

// add two piecewise linear functions (curves).
RayList addRayList(const RayList &rl1, const RayList &rl2)
{

    RayList res;

    // Mark the iterator of RayList at begin and end in reverse order
    auto it1 = rl1.crbegin(), end1 = rl1.crend();
    auto it2 = rl2.crbegin(), end2 = rl2.crend();

    // If one of the curve has inf final slope,
    // find a point on x-axis, where x-vlaue of other curve is <= that of the curve with inf final slope.
    if (!std::isfinite(it1->k)) while (it2->p.x > it1->p.x) ++it2;
    if (!std::isfinite(it2->k)) while (it1->p.x > it2->p.x) ++it1;

    res.clear();

    // Add the consecutive pieces.
    // Algorithm proceeds from right to left on the curves.
    while (true) {

        double x_diff = trimDouble(it1->p.x - it2->p.x);
        double y_sum = it1->p.y + it2->p.y; // (b1 + b2)
        double k_sum = it1->k + it2->k;     // (r1 + r2)

        if (approxEquality(x_diff, 0.0)) { // when crv1 & crv2 have common point
            Line l(it1->p.x, y_sum, k_sum); // mark new y-vlaue
            if(!res.empty()) { // correct slope
                double y_diff = trimDouble(res.last().p.y - y_sum);
                double x_diff = trimDouble(res.last().p.x - it1->p.x);
                double slope = trimDouble(y_diff/x_diff);
                if(slope < 0) slope *= -1;
                l.k = slope;
            }
            res.append(l);
            // qStdOut() << "0 x : " << l.p.x << ", y : " << l.p.y << ", k : " << l.k << ", x_diff" << x_diff;
            if (++it1 == end1 || ++it2 == end2) break; // end of curves

        } else if (x_diff < 0.0) {  // when crv1 is on left of crv2
            y_sum -= trimDouble(x_diff * it1->k);   // adjust y value change due to slope.
            Line l(it2->p.x, y_sum, k_sum); // mark a new y-value at x point of curv2 position
            if(!res.empty()) { // correct slope
                double y_diff = trimDouble(res.last().p.y - y_sum);
                double x_diff = trimDouble(res.last().p.x - it2->p.x);
                double slope = trimDouble(y_diff/x_diff);
                if(slope < 0) slope *= -1;
                l.k = slope;
            }
            // qDebug() << "1 x : " << l.p.x << ", y : " << l.p.y << ", k : " << l.k << ", x_diff" << x_diff;
            res.append(l);
            ++it2;

        } else /*if (x_diff > 0.0)*/ {      // when crv1 is on right of crv2
            y_sum += trimDouble(x_diff * it2->k);       // adjust y value change due to slope.
            Line l(it1->p.x, y_sum, k_sum); // mark a new y-value at x point of curv1 position
            if(!res.empty()) { // correct slope
                double y_diff = trimDouble(res.last().p.y - y_sum);
                double x_diff = trimDouble(res.last().p.x - it1->p.x);
                double slope = trimDouble(y_diff/x_diff);
                if(slope < 0) slope *= -1;
                l.k = slope;
            }
            // qDebug() << "2 x : " << l.p.x << ", y : " << l.p.y << ", k : " << l.k << ", x_diff" << x_diff;
            res.append(l);
            ++it1;
        }
    }

    std::reverse(res.begin(), res.end());

    return res;
}

// compare the ends of two piecewise linear functions
//    l1 and l2 are the end-piece of their respective piecewise linear functions
//    return : true when l1 ends above l2
static
bool ends_above(const Line &l1, const Line &l2) {
    // unequal end-slopes
    if (l1.k != l2.k) return l1.k > l2.k; // function with the greater end-slope always terminates above the other
    // equal and infinite end-slopes
    if (!std::isfinite(l1.k)) return l1.p.x < l2.p.x; // out of two infinite end-slopes, the one starting before the other always terminates above the other
    // equal and finite end-slopes
    return l2.p.y < (l1.p.y + trimDouble((l2.p.x - l1.p.x) * l1.k)); // compare the y position of the two lines at the same x position
}

// compute intersection of the two lines
//      required :: l1 == "lower", l2 == "upper"
static
Point crosspoint(const Line &l1, const Line &l2) { // it was advised to not use trimDouble() in this function! I used it anyway

    // lo = l1, up = l2

    if(!(l1.k != l2.k)) { // parallel or coincident lines.
        double temp = trimDouble(trimDouble(l2.p.y - l1.p.y)/trimDouble(l2.p.x - l1.p.x));
        if(temp < 0) temp = trimDouble(temp * -1);
        if(approxEquality(l1.p.x, l2.p.x) && approxEquality(l1.p.y, l2.p.y)) { // coincident lines.
            return l1.p; // assuming crosspoint same as the initiation point.
        }
        else if(approxEquality(l1.k, temp)) { // non-coincident but overlapping lines.
            if(l2.p.y > l1.p.y) return l2.p; // assuming l2 lies on l1.
            else return l1.p; // assuming l1 lies on l2.
        }
        else { // parallel lines. crosspoint cannot be computed
            qStdOut() << "parallel lines with slopes " << l1.k << " and " << l2.k;
            qStdOut() << "with origins (" << l1.p.x << "," << l1.p.y << ") and (" << l2.p.x << "," << l2.p.y << ")";
            throw std::runtime_error("ASSERT is false : k1 != k2 :: Crosspoint will be at +-inf and/or at multiple points");
        }
    }

    if (!std::isfinite(l1.k)) { // infintie slope l1
        double y = l2.p.y + trimDouble((l1.p.x - l2.p.x)*l2.k); // l2 incident on l1
        return {l1.p.x, y};
    } else if (!std::isfinite(l2.k)) { // infinite slope l2
        double y = l1.p.y + trimDouble((l2.p.x - l1.p.x)*l1.k); // l1 incident on l2
        return {l2.p.x, y};
    } else { // finite slopes
        // (y - y_up)/(x - x_up) = k_up => y = k_up (x - x_up) + y_up
        // (y - y_lo)/(x - x_lo) = k_lo => y = k_lo (x - x_lo) + y_lo
        //    k_up (x - x_up) + y_up =  k_lo (x - x_lo) + y_lo
        //    x (k_up - k_lo) + (k_lo*x_lo - k_up*x_up) = y_lo - y_up
        //      x = ((y_lo - y_up) - (k_lo*x_lo - k_up*x_up))/ (k_up - k_lo)
        double b1 = l1.p.y - trimDouble(l1.p.x * l1.k);
        double b2 = l2.p.y - trimDouble(l2.p.x * l2.k);
        double x = trimDouble((b2 - b1) / (l1.k - l2.k));
        double y = b1 + trimDouble(x * l1.k);
        return {x, y};
    }

}

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************    ConcaveCurve   ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //


// minimum of the two piecewise linear functions. (only concave)
void minRayList(const RayList &rl1, const RayList &rl2, RayList &res) {

    // mark the iterator of RayList at begin and end in reverse order
    auto upper = std::make_pair(rl1.crbegin(), rl1.crend());
    auto lower = std::make_pair(rl2.crbegin(), rl2.crend());

    // "upper" and "lower" represent the respective "piece" above and under one another.
    if (ends_above(*lower.first, *upper.first)) std::swap(upper, lower);

    // select the lower pieces. Note: slopes are not adjusted in this function to compute min piecewise linear function.
    // Algorithm proceeds from right to left on the curves.
    res.clear();
    while (true) {
        auto &up = *upper.first, &lo = *lower.first;
        double x_diff = up.p.x - lo.p.x;

        double epsilon = trimDouble(1/PRECISION); // to consider the defects of floating point precision

        if (x_diff > (0.0 + epsilon)) { // "upper" piece starts after "lower" piece
            if (up.p.y < lo.p.y + trimDouble(x_diff * lo.k)) { // when starting y-point of "upper" is below the y of "lower" at the same x position
                Line l(crosspoint(lo, up), lo.k); // line starting from their intersection with smaller slope is the resulting lower piece (partial)
                res.push_back(l);
                res.push_back(*upper.first++); // another lower piece (partial) is the part of "upper" till the intersection
                std::swap(upper, lower); // why? because the two curves are concave. (let you do the math)
            } else ++upper.first; // no possible intersection, "lower" (partial) is the part of min
        } else if (x_diff < (0.0 - epsilon)) { // "upper" piece starts before "lower" piece
            if (up.p.y < lo.p.y + trimDouble(x_diff * up.k)) { // when starting y-point of "upper" is below the y of "lower" at the same x position
                Line l(crosspoint(lo, up), lo.k); // line starting from their intersection with smaller slope is the resulting lower piece (partial). no decision can be made (yet) about the other lower piece (partial).
                res.push_back(l);
                ++lower.first;
                std::swap(upper, lower);
            } else res.push_back(*lower.first++); // no possible intersection, "lower" is the part of min
        } else /* if (x_diff == 0.0) */ { // "upper" and "lower" pieces start at the same x-position
            if (up.p.y < lo.p.y) { // when starting y-point of "upper" is below the starting y-point of "lower"
                Line l(crosspoint(lo, up), lo.k); // line starting from their intersection with smaller slope is the resulting lower piece (partial)  // another lower piece (partial) is the part of "upper" till the intersection (which is added after this "if" statement (thanks to the tricky use of swap() function))
                res.push_back(l);
                std::swap(upper, lower);
            }
            res.push_back(*lower.first);

            // reached at the origin of the curves
            if (++lower.first == lower.second) break;
            if (++upper.first == upper.second) break;
        }

    }

    // Caution : res may NOT be continous. slopes of res MUST BE ADJUSTED SEPARATELY to make it continuous
    std::reverse(res.begin(), res.end());
}

// maximum of the two piecewise linear functions. (only wide-sense increasing)
void maxRayList(const RayList &rl1, const RayList &rl2, RayList &res) {

    // mark the iterator of RayList at begin and end in reverse order
    auto upper = std::make_pair(rl1.crbegin(), rl1.crend());
    auto lower = std::make_pair(rl2.crbegin(), rl2.crend());

    // "upper" and "lower" represent the respective "piece" above and under one another.
    if (ends_above(*lower.first, *upper.first)) std::swap(upper, lower);

    // select the upper pieces. Note: slopes are not adjusted in this function to compute max piecewise linear function.
    // Algorithm proceeds from right to left on the curves.
    res.clear();
    while (true) {
        auto &up = *upper.first, &lo = *lower.first;
        double x_diff = up.p.x - lo.p.x;

        if (approxEquality(x_diff, 0.0)) { // xDiff == 0 // "upper" and "lower" pieces start at the same x-position

            if (up.p.y < lo.p.y) {  // when starting y-point of "upper" is below the starting y-point of "lower"
                Line l(crosspoint(lo, up), up.k); // line starting from their intersection with greater slope is the resulting upper piece (partial)  // another upper piece (partial) is the part of "lower" till the intersection (which is added after this "if" statement (thanks to the tricky use of swap() function))
                res.push_back(l);
                std::swap(upper, lower);
            }
            res.push_back(*upper.first);

            // reached at the origin of the curves
            if (++lower.first == lower.second) break;
            if (++upper.first == upper.second) break;

        } else  if (x_diff > 0.0) {  // "upper" piece starts after "lower" piece
            if (up.p.y < lo.p.y + trimDouble(x_diff * lo.k)) { // when starting y-point of "upper" is below the y of "lower" at the same x position
                Line l(crosspoint(lo, up), up.k); // line starting from their intersection with greater slope is the resulting upper piece (partial).  no decision can be made (yet) about the other upper piece (partial).
                res.push_back(l);
                ++upper.first;
                std::swap(upper, lower);
            } else res.push_back(*upper.first++); // no possible intersection, "upper" is the part of max
        } else /*if (x_diff < 0.0)*/ { // "upper" piece starts before "lower" piece
            if (up.p.y < lo.p.y + trimDouble(x_diff * up.k)) { // when starting y-point of "upper" is below the y of "lower" at the same x position
                Line l(crosspoint(lo, up), up.k); // line starting from their intersection with greater slope is the resulting upper piece (partial).
                res.push_back(l);
                res.push_back(*lower.first++);  // another upper piece (partial) is the part of "lower" till the intersection
                std::swap(upper, lower);
            } else ++lower.first; // no possible intersection, "upper" (partial) is the part of max
        }

    }

    // Caution : res may NOT be continous. slopes of res MUST BE ADJUSTED SEPARATELY to make it continuous
    std::reverse(res.begin(), res.end());

}

// adjust slopes of a piecewise linear function
void makeContinuous(RayList &rl)
{
    // correct slope to connect point (x_i,y_i) to the point (x_(i-1), y_(i-1))
    for(int i = 1; i < rl.size(); ++i) {
        double x_diff = (rl.begin()+i)->p.x - (rl.begin()+i-1)->p.x;
        double y_diff = (rl.begin()+i)->p.y - (rl.begin()+i-1)->p.y;
        double slope = trimDouble(y_diff/x_diff);
        Line l((rl.begin()+i-1)->p, slope);
        rl.replace(i-1, l);
    }

    // remove redundant points (if any)
    bool redundant = false;
    for(int i = 1; i < rl.size(); ++i) {

        double slope = (rl.begin()+i)->k;
        if(std::isnan(slope)) { // multiple points with same (x,y) values
            rl.removeAt(i); --i;
            redundant = true;
            continue;
        }

        double slope_prev = (rl.begin()+i-1)->k;
        if(approxEquality(slope, slope_prev)) { // same slope at different X values
            rl.removeAt(i); --i;
            redundant = true;
        }
    }

    // removal of redundant points may disrupt continuity.
    if(redundant) makeContinuous(rl);
}

// adjust slopes of a piecewise linear function (only concave)
void makeConcave(RayList &rl)
{
    // check if already concave
    bool isConcave = true;
    {
        makeContinuous(rl);
        for(int i = 1; i < rl.size(); ++i) {
            double slope = (rl.begin()+i)->k;
            double slope_prev = (rl.begin()+i-1)->k;
            if(slope > slope_prev) {
                isConcave = false; break;
            }
        }
        if(isConcave) return;
    }

    // make concave :
    //     for each point p_i on the curve
    //      (1) caclulate slopes with respect to all the other points p_j to the right of p_i
    //      (2) select the point p_j which makes the largest slope with p_i
    //      (3) remove all the points between p_i and p_j
    //     connect all the remaining points
    //     correct non-conacavity (if any) with respect to the slope of last point on the curve.
    // *************************************************************************************

    for (int pi = 0; pi < rl.size(); ++pi) { // for each point p_i on the curve
        double largestSlope = -1;
        int pj_largestSlope = -1;
        //  (1) caclulate slopes with respect to all the other points p_j to the right of p_i
        for (int pj = pi+1; pj < rl.size(); ++pj) {
            double x_diff = (rl.begin()+pj)->p.x - (rl.begin()+pi)->p.x; // xj - xi
            double y_diff = (rl.begin()+pj)->p.y - (rl.begin()+pi)->p.y; // yj - yi
            double slope = trimDouble(y_diff/x_diff);

            //  (2) select the point p_j which makes the largest slope with p_i
            if((slope > largestSlope) || approxEquality(slope, largestSlope)) { largestSlope = slope; pj_largestSlope = pj; }
        }

        //  (3) remove all the points between p_i and p_j
        if(pj_largestSlope != -1) {
            int removeCount = pj_largestSlope - (pi+1);
            for(int x = 0; x < removeCount; ++x) { rl.removeAt(pi+1); }
        }
    }

    // connect all the remaining points
    makeContinuous(rl);

    // correct non-conacavity (if any) with respect to the slope of last point on the curve.
    while((rl.end()-1)->k > (rl.end()-2)->k) { double finalSlope = rl.last().k; rl.removeLast(); rl.last().k = finalSlope;
        if(rl.size() < 2) throw std::runtime_error("final slope is larger than initial slope. unable to make concave! nccurves::makeConcave() ");
    }
}

// print: for debuging purpose
void printRayList(RayList const &rL) {
    foreach(Line const&l, rL) {
        qStdOut() << "k : " << QString().setNum(l.k, 'f', 5) << " (" << QString().setNum(l.p.x, 'f', 5) << "," << QString().setNum(l.p.y, 'f', 5) << ")";
    }
}

// Base curve + predefined initial burst and initial rate
ConcaveCurve::ConcaveCurve(double initialBurst, double initialRate) {
    makeBaseCurve();
    appendRay(Line(Point(0.0, initialBurst), initialRate));
}

// add line l at the end of the curve (assures concavity and continuity with respect to last "piece" of curve)
void ConcaveCurve::appendRay(const Line &l)
{
    if(rayList.empty()) validateCausality(l);
    if(!validateConcavity(l, rayList.size())) throw std::runtime_error("ConcaveCurve :: Unable to appendRay(). Concavity Validation Error.");
    if(!validateContinuity(l, rayList.size())) {
        qStdOut() << "append (" << l.p.x << "," << l.p.y << "):" << l.k << " to ";
        printRayList(rayList);
        throw std::runtime_error("ConcaveCurve :: Unable to appendRay(). Continuity Validation Error.");
    }

    rayList.append(l);
}

// overwrite the curve with the given rayList. (sequentially appendRay())
void ConcaveCurve::setRayList(const RayList &rL)
{
    rayList.clear();
    foreach (Line const& l, rL) {
        appendRay(l);
    }
}

// verify concavity if line "l" is inserted at position "pos". used by appendRay();
bool ConcaveCurve::validateConcavity(Line const&l, int const&pos) const
{
    // invalid line or position
    if((pos < 0) || !validateRay(l)) return false;

    // Note: on insertion "l" gets index "pos" and previous line at "pos" moves to "pos+1".

    if(!rayList.empty()) {
        if(pos == 0) // case : insertion as first line
            return l.k > rayList.first().k; // previous first line must have a slope smaller than new first line

        if(pos > 0 && pos < rayList.size()) // case : insertion between extremities
            return (rayList.at(pos - 1).k > l.k) && (l.k > rayList.at(pos/*+1*/).k); // assure decreasing slopes

        if(pos == rayList.size()) // case : insertion as end-piece
            return rayList.last().k > l.k;
    }

    return true;

}

// verify continuity if line "l" is inserted at position "pos". used by appendRay();
bool ConcaveCurve::validateContinuity(Line const&l, int const&pos) const
{
    // invalid line or position
    if((pos < 0) || !validateRay(l)) return false;

    // Note: on insertion "l" gets index "pos" and previous line at "pos" moves to "pos+1".
    // to be continouous
    //      l must intersect the starting point of previous line at "pos"
    //          (y_{pos} - y_l)/(x_{pos} - x_l) = k_l
    //      line at "pos-1" must intersect the starting point of l
    //          (y_l - y_{pos-1})/(x_l - x_{pos-1}) = k__{pos-1}

    if(!rayList.empty()) {
        if(pos == 0) { // case : insertion as first line
            //      l must intersect the starting point of line at "pos"
            return approxEquality(trimDouble((rayList.at(pos).p.y - l.p.y)/(rayList.at(pos).p.x - l.p.x)), l.k);
        }

        if(pos > 0 && pos < rayList.size()) { // case : insertion between extremities
            //      line at "pos-1" must intersect the starting point of l. and
            //      l must intersect the starting point of previous line at "pos".
            return (approxEquality(trimDouble((l.p.y - rayList.at(pos-1).p.y)/(l.p.x - rayList.at(pos-1).p.x)), rayList.at(pos-1).k) && approxEquality(trimDouble((rayList.at(pos).p.y - l.p.y)/(rayList.at(pos).p.x - l.p.x)), l.k));
        }

        if(pos == rayList.size()) { // case : insertion as end-piece
            //      end-line must intersect the starting point of l.
            return approxEquality(trimDouble((l.p.y - rayList.last().p.y)/(l.p.x - rayList.last().p.x)), rayList.last().k);
        }
    }

    return true;
}

// verify causality if line "l" is inserted at position "pos = 0". used by appendRay() and setRayList();
bool ConcaveCurve::validateCausality(Line const&l) const
{
    return ((l.p == Point(0.0,0.0)) && ((l.k == 0.0) || (l.k == 1.0/0.0)))?true:throw std::runtime_error("line insertion destroys causality");
}

// addition of two arrival curves
ConcaveCurve ConcaveCurve::operator +(const ConcaveCurve &cc2 ) const
{
    ConcaveCurve result = *this;
    result += cc2;
    return result;
}

// addition of two arrival curves
void ConcaveCurve::operator +=(const ConcaveCurve &cc2)
{
    if(this->isBaseCurve()) { //replace base curve by cc2
        this->setRayList(cc2.getRayList());
    } else { // replace this curve by the sum
        RayList res = addRayList(this->getRayList(), cc2.getRayList()); // make sum
        makeConcave(res); // make result concave
        this->setRayList(res); // replace this curve by the sum
    }
}

// ***************** **************** ***************** //
// ***************** **************** ***************** //
// *****************    ConvexCurve   ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //

// Base curve + predefined initial latency and initial rate
ConvexCurve::ConvexCurve(double initialLatency, double initialRate) {
    RayList rL; if(initialLatency > 0.0) {rL = makeBaseCurve();}
    rL.append(Line(Point(initialLatency, 0.0), initialRate)); setRayList(rL);
}

// overwrite the curve with the given rayList. (assures latency-rate server service curve)
void ConvexCurve::setRayList(const RayList &rL) {
    if(validateLatencyRateServer(rL))
        rayList = rL;
}

void ConvexCurve::setRayList_piecewiseConvex(const RayList &rL)
{
    if(!rL.empty()) {
        // validate "pieces" of the curve
        foreach (Line const&l, rL) { validateRay(l); }

        // validate causality
        if(!(rL.first().p == Point(0.0,0.0))) throw std::runtime_error("service curve is not causal :: setRayList() ");

        // verify format
        // (0,0):0 -> (latency,0):rate
        if(rL.size() > 1) if(!((rL.first().k == 0.0)&&(rL.at(1).p.x > 0.0))) throw std::runtime_error("service curve is not a latency-rate server :: setRayList() ");

    } else throw std::runtime_error("invalid raylist :: ConvexCurve::setRayList() ");

    rayList = rL;
}

// verify concavity if line "l" is inserted at position "pos". used by setRayList();
bool ConvexCurve::validateLatencyRateServer(const RayList &rL)
{
    if(!rL.empty()) {
        // validate "pieces" of the curve
        foreach (Line const&l, rL) { validateRay(l); }

        // validate causality
        if(!(rL.first().p == Point(0.0,0.0))) throw std::runtime_error("service curve is not causal :: setRayList() ");

        // verify format
        // (0,0):0 -> (latency,0):rate
        if(rL.size() > 1) if(!((rL.first().k == 0.0)&&(rL.at(1).p.x > 0.0))) throw std::runtime_error("service curve is not a latency-rate server :: setRayList() ");
//        if(rL.size() > 2) throw std::runtime_error("service curve is not a latency-rate server :: setRayList() ");

    } else throw std::runtime_error("invalid raylist :: ConvexCurve::setRayList() ");

    return true;
}


// ***************** **************** ***************** //
// ***************** **************** ***************** //
// ***************** Curve operations ***************** //
// ***************** **************** ***************** //
// ***************** **************** ***************** //

// minimum of the two ConcaveCurve.
ConcaveCurve min(ConcaveCurve const &conc1, ConcaveCurve const &conc2)
{
    ConcaveCurve resConc;
    RayList const &r1 = conc1.getRayList();
    RayList const &r2 = conc2.getRayList();
    RayList res = resConc.getRayList();

    minRayList(r1, r2, res);

    makeContinuous(res); // suggestion

    resConc.setRayList(res);
    return resConc;
}

// maximum horizontal distance between overall arrival curve and service curve (only latency-rate service curve) (i.e. worst-case delay)
double maxHorizontalDistance(const ConcaveCurve &arrivalCurve, const ConvexCurve &serviceCurve, const int &flowID, const NODE_t &node)
{
    if(arrivalCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty arrival curves. unable to compute horizontal distance!");
    } else if(serviceCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty service curves. unable to compute horizontal distance!");
    }

    // validation
    foreach (Line const&l, arrivalCurve.getRayList()) {
        validateRay(l);
    }
    // validation
    ConvexCurve::validateLatencyRateServer(serviceCurve.getRayList());

    // maximum horizontal distance between a concave (arrival) and a convex (service) curve
    // corresponds to the unique points on the two curves where they start to converge towards each other.
    // in piecewise linear curves it belongs to the "pieces" (line) where l_concave.k < l_convex.k

    // case I : initial slope of Convex Curve is greater than that of Concave Curve
    //       horizontal distance = 0
    if (arrivalCurve.getRayList().first().k <= serviceCurve.getRayList().first().k) {
        qStdOut() << "unusual case : slope(arrival_curve) <= slope(service_curve) at t = 0";
        return 0.0;
    }

    // case II : no convergence. final slope of Concave Curve is greater than that of Convex Curve
    //       horizontal distance = +inf
    if (arrivalCurve.getRayList().last().k >= serviceCurve.getRayList().last().k) {
        qStdOut() << "no convergence : slope(arrival_curve)(=" << arrivalCurve.getRayList().last().k << ") >= slope(service_curve)(=" << serviceCurve.getRayList().last().k << ") at t = +inf";
        return 1.0/0.0;
    }

    // case III : valid convergence.
    // the horizontal distance is computed as
    //     (1) look for the lines where the two curves start to converge.
    //     (2) identify (x_concave,y_concave) where concave curve begin to converge.
    //     (3) identify x_convex where y_convex == y_concave
    //     (4) max horizontal distance = x_convex - x_concave
    //
    // this computation is limited to a latency-rate server.
    //

    //     (1) look for the lines where the two curves start to converge.
    auto ccIterator = arrivalCurve.getRayList().cbegin(), ccEnd = arrivalCurve.getRayList().cend();
    auto cvIterator = serviceCurve.getRayList().cbegin(), cvEnd = serviceCurve.getRayList().cend();
    do {
        double cvY = cvIterator->p.y;
        double ccY = ccIterator->p.y;
        double yDiff = cvY - ccY;

        if (yDiff >= 0.0 && ++ccIterator == ccEnd) { // CV same(or)above CC
            // CC reached end
            // find the line in CV where slope(concave) < slope(convex)
            for(--ccIterator; ccIterator->k >= cvIterator->k; ++cvIterator);

            break; // break do-while loop
        }

        if (yDiff <= 0.0 && ++cvIterator == cvEnd) { // CC same(or)above CV
            // CV reached end
            // find the line in CC where slope(concave) < slope(convex)
            for(--cvIterator; ccIterator->k >= cvIterator->k; ++ccIterator);

            break; // break do-while loop
        }

    } while (ccIterator->k >= cvIterator->k);    /* while slope of concave curve is greater than (or) equal to slope of convex curve*/

    //     (2) identify (x_concave,y_concave) where concave curve begins to converge.
    double x_concave = ccIterator->p.x;
    double y_concave = ccIterator->p.y;

    //     (3) identify x where y == y_concave on line l of convex curve
    //          (y_concave - l.y)/(x - l.x) = l.k
    //          => x = l.x + (y_concave - l.y)/l.k
    double x = cvIterator->p.x + trimDouble((y_concave - cvIterator->p.y)/cvIterator->k);

    //     (4) max horizontal distance = x_convex - x_concave
    double hDistance = x - x_concave;
    return hDistance;

}

// maximum horizontal distance between overall arrival curve and service curve (i.e. worst-case delay)
double maxHorizontalDistance_nonLR(const ConcaveCurve &arrivalCurve, const ConvexCurve &serviceCurve, const int &flowID, const NODE_t &node)
{
    if(arrivalCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty arrival curves. unable to compute horizontal distance! :: maxHorizontalDistance_nonLR");
    } else if(serviceCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty service curves. unable to compute horizontal distance! :: maxHorizontalDistance_nonLR");
    }

    // validation
    foreach (Line const&l, arrivalCurve.getRayList()) {
        validateRay(l);
    }
    // validation
    {
        // ******************************
        // validate convexity here
        // ******************************
    }

    // maximum horizontal distance between a concave (arrival) and a convex (service) curve
    // corresponds to the unique points on the two curves where they start to converge towards each other.
    // in piecewise linear curves it belongs to the "pieces" (line) where l_concave.k < l_convex.k

    // case I : initial slope of Convex Curve is greater than that of Concave Curve
    //       horizontal distance = 0
    if (arrivalCurve.getRayList().first().k <= serviceCurve.getRayList().first().k) {
        qStdOut() << "unusual case : slope(arrival_curve) <= slope(service_curve) at t = 0";
        return 0.0;
    }

    // case II : no convergence. final slope of Concave Curve is greater than that of Convex Curve
    //       horizontal distance = +inf
    if (arrivalCurve.getRayList().last().k >= serviceCurve.getRayList().last().k) {
        qStdOut() << "no convergence : slope(arrival_curve)(=" << arrivalCurve.getRayList().last().k << ") >= slope(service_curve)(=" << serviceCurve.getRayList().last().k << ") at t = +inf";
        return 1.0/0.0;
    }

    // case III : valid convergence.
    // the max horizontal distance is computed as
    //     (1) identify the convergence point in concave curve
    //     (2) verify and adapt computation with respect to the convergence point in convex curve
    //     (3) compute max horizontal distance
    // ------------------------------------------------------------------------------------------------

    //     (1) identify the convergence point in concave curve
    //          (1a) find convergence point (cc_p_convergence) on concave curvve
    //          (1b) find line (cv_l) on convex curve having same y-value as cc_p_convergence
    double x_convergence = -1;
    double y_convergence = -1;
    Line l;

    auto ccIterator = arrivalCurve.getRayList().cbegin(), ccEnd = arrivalCurve.getRayList().cend();
    auto cvIterator = serviceCurve.getRayList().cbegin(), cvEnd = serviceCurve.getRayList().cend();
    {
        //          (1a) find convergence point (cc_p_convergence) on concave curvve
        do {
            ++ccIterator;
            //          (1b) find line (cv_l) on convex curve having same y-value as cc_p_convergence
            while((ccIterator->p.y > cvIterator->p.y) && ((cvIterator+1) != cvEnd)) ++cvIterator;
            if(ccIterator->p.y < cvIterator->p.y) --cvIterator;

        }while((ccIterator->k > cvIterator->k) && (ccIterator+1 != ccEnd));

        x_convergence = ccIterator->p.x;
        y_convergence = ccIterator->p.y;
        l = *cvIterator;
    }
    {
        //     (2) verify and adapt computation with respect to the convergence point in convex curve
        //          if convex is still diverging at cc_p_convergence
        //             then locate next point on convex curve where it starts to converge
        //          otherwise
        //             no change
        if(ccIterator->k > cvIterator->k){
            while(ccIterator->k > cvIterator->k) { //  locate next point on convex curve where it starts to converge
                ++cvIterator; if(cvIterator == cvEnd) throw std::runtime_error(" Something went wrong :: maxHorizontalDistance_nonLR");
            }

            // adapt computation with respect to the convergence point in convex curve
            x_convergence = cvIterator->p.x;
            y_convergence = cvIterator->p.y;
            l = *ccIterator;
        }
    }

    //     (3) compute max horizontal distance
    //          find x-value on line (l) where y = y_convergence. i.e.
    //             (y_convergence - l.y)/(x - l.x) = l.k
    //          => x = l.x + (y_convergence - l.y)/l.k
    double x = l.p.x + trimDouble((y_convergence - l.p.y)/l.k);
    double maxhDistance = (x > x_convergence)?(x - x_convergence):(x_convergence - x);
    return maxhDistance;
}

// horizontal distance between x-axis and the point on service curve where worst-case delay is computed
double maxHorizontalDistance_X0(const ConcaveCurve &arrivalCurve, const ConvexCurve &serviceCurve, const int &flowID, const NODE_t &node)
{
    if(arrivalCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty arrival curves. unable to compute horizontal distance!");
    } else if(serviceCurve.getRayList().empty()){
        qStdOut() << "error computing horizontal distance for flow " << flowID << " at " << NODE_t_toString(node);
        throw std::runtime_error("empty service curves. unable to compute horizontal distance!");
    }

    // validation
    foreach (Line const&l, arrivalCurve.getRayList()) {
        validateRay(l);
    }
    // validation
    ConvexCurve::validateLatencyRateServer(serviceCurve.getRayList());

    // steps followed in this function are same as those in maxHorizontalDistance();


    // case I : initial slope of Convex Curve is greater than that of Concave Curve
    //       horizontal distance = 0
    if (arrivalCurve.getRayList().first().k <= serviceCurve.getRayList().first().k) {
        qStdOut() << "unusual case : slope(arrival_curve) <= slope(service_curve) at t = 0";
        return 0.0;
    }

    // case II : no convergence. final slope of Concave Curve is greater than that of Convex Curve
    //       horizontal distance = +inf
    if (arrivalCurve.getRayList().last().k >= serviceCurve.getRayList().last().k) {
        qStdOut() << "no convergence : slope(arrival_curve) >= slope(service_curve) at t = +inf";
        return 1.0/0.0;
    }

    // case III : valid convergence.
    // the horizontal distance in the function maxHorizontalDistance() is computed as
    //     (1) look for the lines where the two curves start to converge.
    //     (2) identify (x_concave,y_concave) where concave curve begin to converge.
    //     (3) identify x_convex where y_convex == y_concave
    //     (4) max horizontal distance = x_convex - x_concave
    // whereas, maxHorizontalDistance_X0 is
    //     (5) hDistance_X0 = x_concave + max horizontal distance
    //
    //
    // this computation is limited to a latency-rate server.
    //

    //     (1) look for the lines where the two curves start to converge.
    auto ccIterator = arrivalCurve.getRayList().cbegin(), ccEnd = arrivalCurve.getRayList().cend();
    auto cvIterator = serviceCurve.getRayList().cbegin(), cvEnd = serviceCurve.getRayList().cend();
    do {
        double cvY = cvIterator->p.y;
        double ccY = ccIterator->p.y;
        double yDiff = cvY - ccY;

        if (yDiff >= 0.0 && ++ccIterator == ccEnd) { // CV same(or)above CC
            // CC reached end
            // find the line in CV where slope(concave) < slope(convex)
            for(--ccIterator; ccIterator->k >= cvIterator->k; ++cvIterator);

            break; // break do-while loop
        }

        if (yDiff <= 0.0 && ++cvIterator == cvEnd) { // CC same(or)above CV
            // CV reached end
            // find the line in CC where slope(concave) < slope(convex)
            for(--cvIterator; ccIterator->k >= cvIterator->k; ++ccIterator);

            break; // break do-while loop
        }

    } while (ccIterator->k >= cvIterator->k);    /* while slope of concave curve is greater than (or) equal to slope of convex curve*/

    //     (2) identify (x_concave,y_concave) where concave curve begins to converge.
    double x_concave = ccIterator->p.x;
    double y_concave = ccIterator->p.y;

    //     (3) identify x where y == y_concave on line l of convex curve
    //          (y_concave - l.y)/(x - l.x) = l.k
    //          => x = l.x + (y_concave - l.y)/l.k
    double x = cvIterator->p.x + trimDouble((y_concave - cvIterator->p.y)/cvIterator->k);

    //     (4) max horizontal distance = x_convex - x_concave
    double hDistance = x - x_concave;

    //     (5) hDistance_X0 = x_concave + max horizontal distance
    double hDistanceX0 = x_concave + hDistance;
    return hDistanceX0;
}
