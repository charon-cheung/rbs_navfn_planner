// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//

#include "rbs_navfn_planner/navfn.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include "nav2_core/planner_exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rbs_navfn_planner
{

constexpr double kInvSqrt2 = 0.707106781;

// NavFn approximates the two-neighbor potential increment as a fraction of the cell cost.
// The polynomial input is the difference between neighbor potentials divided by the cell cost.
constexpr double kQuadraticTerm = -0.2301;
constexpr double kLinearTerm = 0.5307;
constexpr double kConstantTerm = 0.7040;

//
// create nav fn buffers
//

NavFn::NavFn(int xs, int ys)
{
  // Keep the priority buffers owned locally until the cell arrays are ready.
  std::unique_ptr<int[]> priority1(new int[kPriorityBufferSize]);
  std::unique_ptr<int[]> priority2(new int[kPriorityBufferSize]);
  std::unique_ptr<int[]> priority3(new int[kPriorityBufferSize]);

  // create cell arrays
  costarr = nullptr;
  potarr = nullptr;
  pending = nullptr;
  gradx = grady = nullptr;
  setNavArr(xs, ys);

  pb1 = priority1.release();
  pb2 = priority2.release();
  pb3 = priority3.release();

  // for Dijkstra (breadth-first), set to kNeutralCost
  // for A* (best-first), set to kNeutralCost
  priInc = 2 * kNeutralCost;

  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;


  // path buffers
  npathbuf = npath = 0;
  pathx = pathy = nullptr;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// set goal, start positions for the nav fn
//

void
NavFn::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// Set/Reset map size
//

void
NavFn::setNavArr(int xs, int ys)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  const int cells = xs * ys;
  auto new_costs = std::make_unique<COSTTYPE[]>(cells);
  std::unique_ptr<float[]> new_potentials(new float[cells]);
  auto new_pending = std::make_unique<bool[]>(cells);
  std::unique_ptr<float[]> new_gradx(new float[cells]);
  std::unique_ptr<float[]> new_grady(new float[cells]);

  delete[] costarr;
  delete[] potarr;
  delete[] pending;
  delete[] gradx;
  delete[] grady;
  costarr = new_costs.release();
  potarr = new_potentials.release();
  pending = new_pending.release();
  gradx = new_gradx.release();
  grady = new_grady.release();
  nx = xs;
  ny = ys;
  ns = cells;
}


//
// set up cost array, usually from ROS
//

void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS-type cost array
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // Inscribed and lethal cells map to kObstacleCost.
        // Scale traversable ROS costs below that threshold.
        *cm = kObstacleCost;
        int v = *cmap;
        if (v < kRosObstacleThreshold) {
          v = kNeutralCost + kCostFactor * v;
          if (v >= kObstacleCost) {
            v = kObstacleCost - 1;
          }
          *cm = v;
        } else if (v == kRosUnknownCost && allow_unknown) {
          v = kObstacleCost - 1;
          *cm = v;
        }
      }
    }
  } else {  // not a ROS map, just a PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = kObstacleCost;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // don't do borders
        }
        int v = *cmap;
        if (v < kRosObstacleThreshold) {
          v = kNeutralCost + kCostFactor * v;
          if (v >= kObstacleCost) {
            v = kObstacleCost - 1;
          }
          *cm = v;
        } else if (v == kRosUnknownCost) {
          v = kObstacleCost - 1;
          *cm = v;
        }
      }
    }
  }
}

bool
NavFn::calcNavFnDijkstra(std::function<bool()> cancelChecker, bool atStart)
{
  setupNavFn(true);

  // calculate the nav fn and path
  return propNavFnDijkstra(std::max(nx * ny / 20, nx + ny), cancelChecker, atStart);
}


//
// calculate navigation function, given a costmap, goal, and start
//

bool
NavFn::calcNavFnAstar(std::function<bool()> cancelChecker)
{
  setupNavFn(true);

  // calculate the nav fn and path
  return propNavFnAstar(std::max(nx * ny / 20, nx + ny), cancelChecker);
}

//
// returning values
//

float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}

void
NavFn::pushCell(int cell, int * buffer, int & count)
{
  if (cell >= 0 && cell < ns && !pending[cell] &&
    costarr[cell] < kObstacleCost && count < kPriorityBufferSize)
  {
    buffer[count++] = cell;
    pending[cell] = true;
  }
}

// Set up navigation potential arrays for new propagation

void
NavFn::setupNavFn(bool keepit)
{
  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) {
    potarr[i] = kUnassignedPotential;
    if (!keepit) {
      costarr[i] = kNeutralCost;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // outer bounds of cost array
  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = kObstacleCost;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = kObstacleCost;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = kObstacleCost;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = kObstacleCost;
  }

  // priority buffers
  curT = kObstacleCost;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  std::memset(pending, 0, ns * sizeof(bool));

  // set goal
  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);

  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= kObstacleCost) {
      ntot++;  // number of cells that are obstacles
    }
  }
  nobs = ntot;
}


// initialize a goal-type cost for starting propagation

void
NavFn::initCost(int k, float v)
{
  potarr[k] = v;
  pushCell(k + 1, curP, curPe);
  pushCell(k - 1, curP, curPe);
  pushCell(k - nx, curP, curPe);
  pushCell(k + nx, curP, curPe);
}


//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

inline void
NavFn::updateCell(int n)
{
  // get neighbors
  const float l = potarr[n - 1];
  const float r = potarr[n + 1];
  const float u = potarr[n - nx];
  const float d = potarr[n + nx];

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}

  // do planar wave update
  if (costarr[n] < kObstacleCost) {  // don't propagate into obstacles
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      const float div = dc / hf;
      const float v = kQuadraticTerm * div * div + kLinearTerm * div + kConstantTerm;
      pot = ta + hf * v;
    }


    // now add affected neighbors to priority blocks
    if (pot < potarr[n]) {
      float le = kInvSqrt2 * static_cast<float>(costarr[n - 1]);
      float re = kInvSqrt2 * static_cast<float>(costarr[n + 1]);
      float ue = kInvSqrt2 * static_cast<float>(costarr[n - nx]);
      float de = kInvSqrt2 * static_cast<float>(costarr[n + nx]);
      potarr[n] = pot;
      if (pot < curT) {  // low-cost buffer block
        if (l > pot + le) {pushCell(n - 1, nextP, nextPe);}
        if (r > pot + re) {pushCell(n + 1, nextP, nextPe);}
        if (u > pot + ue) {pushCell(n - nx, nextP, nextPe);}
        if (d > pot + de) {pushCell(n + nx, nextP, nextPe);}
      } else {  // overflow block
        if (l > pot + le) {pushCell(n - 1, overP, overPe);}
        if (r > pot + re) {pushCell(n + 1, overP, overPe);}
        if (u > pot + ue) {pushCell(n - nx, overP, overPe);}
        if (d > pot + de) {pushCell(n + nx, overP, overPe);}
      }
    }
  }
}

//
// Use A* method for setting priorities
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

inline void
NavFn::updateCellAstar(int n)
{
  // get neighbors
  float l = potarr[n - 1];
  float r = potarr[n + 1];
  float u = potarr[n - nx];
  float d = potarr[n + nx];

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}

  // do planar wave update
  if (costarr[n] < kObstacleCost) {  // don't propagate into obstacles
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      const float div = dc / hf;
      const float v = kQuadraticTerm * div * div + kLinearTerm * div + kConstantTerm;
      pot = ta + hf * v;
    }


    // now add affected neighbors to priority blocks
    if (pot < potarr[n]) {
      float le = kInvSqrt2 * static_cast<float>(costarr[n - 1]);
      float re = kInvSqrt2 * static_cast<float>(costarr[n + 1]);
      float ue = kInvSqrt2 * static_cast<float>(costarr[n - nx]);
      float de = kInvSqrt2 * static_cast<float>(costarr[n + nx]);

      // calculate distance
      int x = n % nx;
      int y = n / nx;
      float dist = std::hypot(x - start[0], y - start[1]) * static_cast<float>(kNeutralCost);

      potarr[n] = pot;
      pot += dist;
      if (pot < curT) {  // low-cost buffer block
        if (l > pot + le) {pushCell(n - 1, nextP, nextPe);}
        if (r > pot + re) {pushCell(n + 1, nextP, nextPe);}
        if (u > pot + ue) {pushCell(n - nx, nextP, nextPe);}
        if (d > pot + de) {pushCell(n + nx, nextP, nextPe);}
      } else {
        if (l > pot + le) {pushCell(n - 1, overP, overPe);}
        if (r > pot + re) {pushCell(n + 1, overP, overPe);}
        if (u > pot + ue) {pushCell(n - nx, overP, overPe);}
        if (d > pot + de) {pushCell(n + nx, overP, overPe);}
      }
    }
  }
}


//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnDijkstra(int cycles, std::function<bool()> cancelChecker, bool atStart)
{
  int nwv = 0;  // max priority block size
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // set up start cell
  int startCell = start[1] * nx + start[0];

  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (cycle % terminal_checking_interval == 0 && cancelChecker()) {
      throw nav2_core::PlannerCancelled("Planner was cancelled");
    }

    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      break;
    }

    // stats
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // reset pending flags on current priority buffer
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCell(*pb++);
    }

    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (atStart) {
      if (potarr[startCell] < kUnassignedPotential) {
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  return (cycle < cycles) ? true : false;
}

//
// main propagation function
// A* method, best-first
// uses Euclidean distance heuristic
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnAstar(int cycles, std::function<bool()> cancelChecker)
{
  int nwv = 0;  // max priority block size
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // set initial threshold, based on distance
  float dist = std::hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(kNeutralCost);
  curT = dist + curT;

  // set up start cell
  int startCell = start[1] * nx + start[0];

  // do main cycle
  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (cycle % terminal_checking_interval == 0 && cancelChecker()) {
      throw nav2_core::PlannerCancelled("Planner was cancelled");
    }

    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      break;
    }

    // stats
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // reset pending flags on current priority buffer
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCellAstar(*pb++);
    }

    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (potarr[startCell] < kUnassignedPotential) {
      break;
    }
  }

  last_path_cost_ = potarr[startCell];

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  if (potarr[startCell] < kUnassignedPotential) {
    return true;  // finished up here}
  } else {
    return false;
  }
}


float NavFn::getLastPathCost()
{
  return last_path_cost_;
}


//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int
NavFn::calcPath(int n, int * st)
{
  // test write

  // check path arrays
  if (npathbuf < n) {
    std::unique_ptr<float[]> new_pathx(new float[n]);
    std::unique_ptr<float[]> new_pathy(new float[n]);
    delete[] pathx;
    delete[] pathy;
    pathx = new_pathx.release();
    pathy = new_pathy.release();
    npathbuf = n;
  }

  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation
  if (st == nullptr) {st = start;}
  npath = 0;
  if (st[0] < 0 || st[0] >= nx || st[1] < 0 || st[1] >= ny) {return 0;}
  int stc = st[1] * nx + st[0];

  // set up offset
  float dx = 0;
  float dy = 0;

  // go for <n> cycles at most
  for (int i = 0; i < n; i++) {
    // check if near goal
    const int x_cell = stc % nx, y_cell = stc / nx;
    const int nearest_x = x_cell + static_cast<int>(std::round(dx));
    const int nearest_y = y_cell + static_cast<int>(std::round(dy));
    if (nearest_x < 0 || nearest_x >= nx || nearest_y < 0 || nearest_y >= ny) {return 0;}
    const int nearest_point = nearest_y * nx + nearest_x;
    if (potarr[nearest_point] < kNeutralCost) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;  // done!
    }

    // A border goal can finish above. Continuing needs a complete 3x3 neighborhood.
    if (x_cell == 0 || x_cell == nx - 1 || y_cell == 0 || y_cell == ny - 1) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }

    // add to path
    pathx[npath] = x_cell + dx;
    pathy[npath] = y_cell + dy;
    npath++;

    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // check for potentials at eight positions near cell
    if (potarr[stc] >= kUnassignedPotential ||
      potarr[stc + 1] >= kUnassignedPotential ||
      potarr[stc - 1] >= kUnassignedPotential ||
      potarr[stcnx] >= kUnassignedPotential ||
      potarr[stcnx + 1] >= kUnassignedPotential ||
      potarr[stcnx - 1] >= kUnassignedPotential ||
      potarr[stcpx] >= kUnassignedPotential ||
      potarr[stcpx + 1] >= kUnassignedPotential ||
      potarr[stcpx - 1] >= kUnassignedPotential ||
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);

      // check eight neighbors to find the lowest
      int minc = stc;
      float minp = potarr[stc];
      int sti = stcpx - 1;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti++;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti++;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti = stc - 1;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti = stc + 1;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti = stcnx - 1;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti++;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      sti++;
      if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      if (potarr[stc] >= kUnassignedPotential) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");
        return 0;
      }
    } else {  // have a good gradient here
      // get grad at four positions near cell
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);


      // get interpolated gradient
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y


      // check for zero gradient, failed
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      float ss = pathStep / std::hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
  return 0;  // out of cycles, return failure
}


//
// gradient calculations
//

// calculate gradient at a cell
// positive value are to the right and down
float
NavFn::gradCell(int n)
{
  if (n < nx || n >= ns - nx || n % nx == 0 || n % nx == nx - 1) {
    return 0.0;
  }
  if (gradx[n] != 0.0f || grady[n] != 0.0f) {
    return 1.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;

  // check for in an obstacle
  if (cv >= kUnassignedPotential) {
    if (potarr[n - 1] < kUnassignedPotential) {
      dx = -kObstacleCost;
    } else if (potarr[n + 1] < kUnassignedPotential) {
      dx = kObstacleCost;
    }
    if (potarr[n - nx] < kUnassignedPotential) {
      dy = -kObstacleCost;
    } else if (potarr[n + nx] < kUnassignedPotential) {
      dy = kObstacleCost;
    }
  } else {  // not in an obstacle
    // dx calc, average to sides
    if (potarr[n - 1] < kUnassignedPotential) {
      dx += potarr[n - 1] - cv;
    }
    if (potarr[n + 1] < kUnassignedPotential) {
      dx += cv - potarr[n + 1];
    }

    // dy calc, average to sides
    if (potarr[n - nx] < kUnassignedPotential) {
      dy += potarr[n - nx] - cv;
    }
    if (potarr[n + nx] < kUnassignedPotential) {
      dy += cv - potarr[n + nx];
    }
  }

  // normalize
  float norm = std::hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}

}  // namespace rbs_navfn_planner
