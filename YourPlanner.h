#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

protected:
  //extension switches
  static constexpr bool USE_EXHAUSTED = true;  // Ext2
  static constexpr bool USE_KNN       = false;  // Ext3
  static constexpr bool USE_WEIGHTED  = true;   // Ext4

  ::rl::math::Real weightedDistance(const ::rl::math::Vector& a,
                                   const ::rl::math::Vector& b) const;

  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  //Extension-Exhausted Nodes: skip the exhausted node when looking for the nearest neighbor
  virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen) override;

private:
  //Extension-Exhausted Nodes: recording data structure of exhausted nodes,
  //use map to distinguish start tree and goal tree,
  //store exhausted nodes' vertex Id
  std::map<const Tree*, std::set<Vertex>> exhausted;
  std::map<const Tree*, std::map<Vertex, int>> failCount;
};

#endif // _YOUR_PLANNER_H_