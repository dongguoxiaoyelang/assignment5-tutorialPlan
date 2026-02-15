#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <random>
#include <limits>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <set>
#include <map>
#include <cmath>
#include <vector>


YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here

  //1st.Extansion: Goal Bias
  //use "static" to initialize the random engine only once per program run,
  //This ensures better random distribution and performance.
  static std::random_device rd;  // Non-deterministic random seed
  static std::mt19937 gen(rd()); // Standard mersenne_twister_engine
  static std::uniform_real_distribution<double> dist(0.0, 1.0);

  //10% probability to sample the Goal Configuration
  double goalBias = 0.1;

  if (dist(gen) < goalBias)
  {
    // specific sampling: try to connect directly to the goal
    chosen = *this->goal;

    // RRT-Connect works by growing two trees: Tree A (Start) and Tree B (Goal).
    // Tree B's root IS the Goal configuration.
    // If we sample the EXACT Goal configuration while extending Tree B,
    // the distance between the tree node and the sample becomes 0.
    // This causes a "Division by Zero" or NaN error in the connection vector normalization,
    // which immediately crashes the application.
    //Add a tiny amount of noise (perturbation) to the goal sample.
    for (std::size_t i = 0; i < this->model->getDof(); ++i)
    {
      // Add noise in the range of [-0.0001, 0.0001] radians.
      // This is small enough to be considered "the goal" for planning,
      // but large enough to prevent math errors.
      double noise = (dist(gen) - 0.5) * 0.0002;
      chosen(i) += noise;
    }
  }
  else
  {
    // 90% probability: Uniform Sampling (Exploration)
    // Call the base class implementation which uses the Sampler (YourSampler)
    RrtConConBase::choose(chosen);
  }
}

::rl::math::Real
YourPlanner::weightedDistance(const ::rl::math::Vector& a,
                              const ::rl::math::Vector& b) const
{
  // Puma560 (6 DoF) recommended "front-heavy" weights
  // J1-3: 1.0, J4-6: 0.5
  static const double w[6] = {1.0, 1.0, 1.0, 0.8, 0.8, 0.8};

  ::rl::math::Real sum = 0;

  const std::size_t n = static_cast<std::size_t>(a.size());
  for (std::size_t i = 0; i < n; ++i)
  {
    const ::rl::math::Real wi = (i < 6) ? w[i] : 1.0;
    const ::rl::math::Real d  = a(i) - b(i);
    sum += wi * d * d;
  }

  return sum;
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  //kNN
  if (!USE_KNN)
  {
    Neighbor best(Vertex(), std::numeric_limits< ::rl::math::Real >::max());

    const std::set<Vertex>* currentExhausted = NULL;
    if (USE_EXHAUSTED)
    {
      currentExhausted = &this->exhausted[&tree];
    }

    VertexIteratorPair it = ::boost::vertices(tree);
    for (VertexIterator i = it.first; i != it.second; ++i)
    {
      if (!tree[*i].q)
        continue;

      if (USE_EXHAUSTED && currentExhausted)
      {
        if (currentExhausted->find(*i) != currentExhausted->end())
          continue;
      }

      ::rl::math::Real d;

      if (USE_WEIGHTED)
      {
        d = this->weightedDistance(chosen, *tree[*i].q); // real distance
      }
      else
      {
        // original metric for comparison
        d = this->model->transformedDistance(chosen, *tree[*i].q);
        d = this->model->inverseOfTransformedDistance(d);
      }

      if (d < best.second)
      {
        best.first = *i;
        best.second = d;
      }
    }

    if (best.first == Vertex())
    {
      return RrtConConBase::nearest(tree, chosen);
    }

    return best;
  }

  // -------- KNN path: keep your old logic (vector + sort) --------
  std::vector<std::pair<Vertex, ::rl::math::Real>> candidates;

  const std::set<Vertex>* currentExhausted = NULL;
  if (USE_EXHAUSTED)
  {
    currentExhausted = &this->exhausted[&tree];
  }

  VertexIteratorPair it = ::boost::vertices(tree);
  for (VertexIterator i = it.first; i != it.second; ++i)
  {
    if (!tree[*i].q)
      continue;

    if (USE_EXHAUSTED && currentExhausted)
    {
      if (currentExhausted->find(*i) != currentExhausted->end())
        continue;
    }

    ::rl::math::Real d;

    if (USE_WEIGHTED)
    {
      d = this->weightedDistance(chosen, *tree[*i].q);
    }
    else
    {
      d = this->model->transformedDistance(chosen, *tree[*i].q);
      d = this->model->inverseOfTransformedDistance(d);
    }

    candidates.push_back(std::make_pair(*i, d));
  }

  if (candidates.empty())
  {
    return RrtConConBase::nearest(tree, chosen);
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const std::pair<Vertex, ::rl::math::Real>& a,
       const std::pair<Vertex, ::rl::math::Real>& b)
    {
      return a.second < b.second;
    });

  const int k = std::min(5, static_cast<int>(candidates.size()));
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, k - 1);

  auto selected = candidates[dist(gen)];

  Neighbor result;
  result.first = selected.first;
  result.second = selected.second;
  return result;
}



RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  RrtConConBase::Vertex newVertex = RrtConConBase::connect(tree, nearest, chosen);

  if (USE_EXHAUSTED)
  {
    const int THRESH = 15;

    if (NULL == newVertex)
    {
      int& cnt = this->failCount[&tree][nearest.first];
      cnt++;

      if (cnt >= THRESH)
      {
        this->exhausted[&tree].insert(nearest.first);
      }
    }
    else
    {
      this->failCount[&tree][nearest.first] = 0;
    }
  }

  return newVertex;
}


RrtConConBase::Vertex
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  RrtConConBase::Vertex newVertex = RrtConConBase::extend(tree, nearest, chosen);

  if (USE_EXHAUSTED)
  {
    const int THRESH = 15;

    if (NULL == newVertex)
    {
      int& cnt = this->failCount[&tree][nearest.first];
      cnt++;

      if (cnt >= THRESH)
      {
        this->exhausted[&tree].insert(nearest.first);
      }
    }
    else
    {
      this->failCount[&tree][nearest.first] = 0;
    }
  }

  return newVertex;
}

bool
YourPlanner::solve()
{
  //your modifications here

  //clean up the exhausted nodes recording before the start of new plan for each time
   if (USE_EXHAUSTED)
    {
      this->exhausted.clear();
      this->failCount.clear();
    }

  return RrtConConBase::solve();
}

