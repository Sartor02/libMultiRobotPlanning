// ECBS Search (High-level)
#ifndef ECBSSEARCH_H
#define ECBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <sparsehash/dense_hash_map>
#include <cstring>
#include <climits>
#include <tuple>
#include <string>
#include <vector>
#include <list>
#include <fstream>
#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_agent_ecbs.h"
#include "ecbs_node.h"
#include "map_loader.h"
#include "agents_loader.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;
using std::endl;
using google::dense_hash_map;

class ECBSSearch {
 public:
  typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
  typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
  typedef dense_hash_map<ECBSNode*, ECBSNode*, ECBSNode::ECBSNodeHasher, ECBSNode::ecbs_eqnode> hashtable_t;

  const MapLoader* ml;
  const AgentsLoader* al;
  const EgraphReader* egr;

  // Anytime version.
  int search_iterations;
  vector < vector< vector<int> > > solution_iterations;  // solution_iteration[i] contains one path per agent.
  vector<double> cost_iterations;
  vector<double> f_weights;  // subopt bound
  vector<double> focal_bounds;  // FOCAL's lower bound (=f_weight*min_f_val)
  vector<double> solution_subopt_bounds;  // solution actual LB (=path_cost/min_f_val)
  vector<double> min_f_vals;  // solution min_f_val (upon termination).
  vector<double> runtimes;
  vector<int> hl_exp_iterations;
  vector<int> hl_gen_iterations;
  vector<int> ll_exp_iterations;
  vector<int> ll_gen_iterations;
  vector<int> num_nodes_removed_iterations;

  double hl_focal_w = 1.0;

  double ll_focal_w = 1.0;
  double hwy_w = 1.0;
  vector<int> agents_order;
  double focal_list_threshold;
  double min_sum_f_vals;

  vector < vector<int>* > paths;  // agents paths (each entry [i] is a vector<int> which specify the locations on the path of agent i)
  vector < vector<int>* > paths_found_initially;  // contain initial paths found
  bool solution_found;
  double solution_cost;

  ECBSNode* dummy_start;
  ECBSNode* most_recent_goal;  // Used to draw statistics from in the anytime version.
  vector <int> start_locations;
  vector <int> most_goal_locations;

  const bool* my_map;
  int map_size;
  int num_of_agents;
  const int* actions_offset;

  uint64_t HL_num_expanded = 0;
  uint64_t HL_num_generated = 0;
  uint64_t LL_num_expanded = 0;
  uint64_t LL_num_generated = 0;

  heap_open_t open_list;
  heap_focal_t focal_list;
  hashtable_t allNodes_table;

  // used in hash table and would be deleted from the d'tor
  ECBSNode* empty_node;
  ECBSNode* deleted_node;

  vector < SingleAgentECBS* > search_engines;  // used to find (single) agents' paths
  vector <double> ll_min_f_vals_found_initially;  // contains initial ll_min_f_vals found
  vector <double> ll_min_f_vals;  // each entry [i] represent the lower bound found for agent[i]
  vector <double> paths_costs_found_initially;
  vector <double> paths_costs;

  tuple<int, int, int, int, int> earliest_conflict;  // saves the earliest conflict (updated in every call to extractCollisions()).

  string dot_filename;

  ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr,
    double e_w, double ll_w_f, double hl_w_f);

  inline double compute_g_val();
  inline double compute_hl_lower_bound();
  inline void updatePaths(ECBSNode* curr , ECBSNode* root_node);
  inline bool updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node);

  /* Run an anytime ECBS
      time_limit in seconds is the overall time cutoff (for all iterations)
      initial_focal_w is the focal factor from which we start.
  */
  bool anytimeFindPath(double time_limit, double initial_focal_w);

  /*
  Run one iteration of ECBS.
    time_limit_cutoff is the time we have for this iteration (at most).
    sol_cost_ub is solution cost upper bound (will not expand or insert to OPEN nodes with cost higher than it).
  */
  bool runECBSSearch(double time_limit_cutoff, double sol_cost_ub);

  inline bool switchedLocations(int agent1_id, int agent2_id, size_t timestep);
  inline int getAgentLocation(int agent_id, size_t timestep);
  vector< tuple<int, int, int, int, int> >* extractCollisions();
  void printPaths();
  void savePaths(ofstream& res_f);
  void printResTable(bool* res_table, size_t max_plan_len);

  size_t getPathsMaxLength();
  void updateReservationTable(bool* res_table, size_t max_plan_len, int exclude_agent);

  /* Iterates over OPEN and adds to FOCAL all nodes that qualifies due to the new bound.
   */
  void updateFocalListLBIncreased(double old_lower_bound, double new_lower_bound);

  /* (Eager) Anytime update of FOCAL.
     Iterates over FOCAL and remove all nodes that are below new_focal_bound.
     Returns the number of nodes removed from FOCAL.
  */
  int updateFocalListFBDecreased(double new_focal_bound, double sol_cost_ub);


  int computeNumOfCollidingAgents();
  int compute_h2();
  int compute_h3();

  void printAnytimeStats();

  inline void releaseClosedListNodes();

  ~ECBSSearch();
};

#endif
