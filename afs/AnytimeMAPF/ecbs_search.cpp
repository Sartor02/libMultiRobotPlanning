#include "ecbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <ctime>
#include <climits>
#include <algorithm>
#include <random>
#include <iterator>
#include <iomanip>


void ECBSSearch::printPaths() {
  for (size_t i = 0; i < paths.size(); i++) {
    cout << "AGENT " << i << " Path: ";
    for (vector<int>::const_iterator it = paths[i]->begin(); it != paths[i]->end(); ++it)
      std::cout << *it << ' ';
    cout << endl;
  }
}


void ECBSSearch::savePaths(ofstream& res_f) {
  for (size_t i = 0; i < paths.size(); i++) {
    res_f << "AGENT " << i << " Path: ";
    for (vector<int>::const_iterator it = paths[i]->begin(); it != paths[i]->end(); ++it)
      res_f << ml->row_coordinate(*it) << ',' << ml->col_coordinate(*it) << " ; ";
    res_f << endl;
  }
}


void ECBSSearch::printResTable(bool* res_table, size_t max_plan_len) {
  cout << "MAP_SIZE=" << map_size << " ; MAX_PLAN_LEN=" << max_plan_len << endl;
  for (size_t t = 0; t < max_plan_len; t++) {
    for (size_t id = 0; id < (size_t)map_size; id++) {
      if ( res_table[id + t*map_size] == false )
        cout << '_';
      else
        cout << '*';
    }
    cout << endl;
  }
}


inline void ECBSSearch::releaseClosedListNodes() {
  hashtable_t::iterator it;
  for (it=allNodes_table.begin(); it != allNodes_table.end(); it++) {
    delete ( (*it).first );  // should it be .second?
  }
}


// computes g_val based on current paths (WRONG!)
inline double ECBSSearch::compute_g_val() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += paths[i]->size();
  return retVal;
}


// computes High-Level lower-bound based
inline double ECBSSearch::compute_hl_lower_bound() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += ll_min_f_vals[i];
  return retVal;
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ECBSSearch::updateFocalListLBIncreased(double old_lower_bound, double new_lower_bound) {
  for (ECBSNode* n : open_list) {
    if ( n->sum_min_f_vals > old_lower_bound &&
         n->sum_min_f_vals <= new_lower_bound )
      n->focal_handle = focal_list.push(n);
  }
}

// Eager version of the anytime update
int ECBSSearch::updateFocalListFBDecreased(double new_focal_bound, double sol_cost_ub) {
  vector<ECBSNode*> nodes_to_remove;
  for (ECBSNode* n : focal_list) {
    if (n->sum_min_f_vals > new_focal_bound){// || n->g_val >= sol_cost_ub) {  /* 2nd part - Upper Bound Improvement */
      nodes_to_remove.push_back(n);
    }
  }
  for (ECBSNode* n : nodes_to_remove) {
    focal_list.erase(n->focal_handle);
  }
  return nodes_to_remove.size();
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ECBSSearch::updatePaths(ECBSNode* curr, ECBSNode* root_node) {
  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;
  vector<bool> updated_ids(num_of_agents, false);  // initialized for false
  /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
   * because younger nodes take into account ancesstors' nodes constraints. */
  while ( curr != root_node ) {
    if (updated_ids[curr->agent_id] == false) {
      paths[curr->agent_id] = &(curr->path);
      ll_min_f_vals[curr->agent_id] = curr->ll_min_f_val;
      paths_costs[curr->agent_id] = curr->path_cost;
      updated_ids[curr->agent_id] = true;
    }
    curr = curr->parent;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool ECBSSearch::updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node) {
  // extract all constraints on leaf_node->agent_id
  list < tuple<int, int, int> > constraints;  // each constraint is <L1,L2,T>
  int agent_id = leaf_node->agent_id;
  //  cout << " update ECBS node for agent:" << agent_id << endl;
  ECBSNode* curr = leaf_node;
  //  cout << "  Find all constraints on him:" << endl;
  while (curr != root_node) {
    if (curr->agent_id == agent_id) {
      constraints.push_front(curr->constraint);
      //      cout << "   L1:" << get<0>(curr->constraint) << " ; L2:" << get<1>(curr->constraint) << " ; T:" << get<2>(curr->constraint) << endl;
    }
    curr = curr->parent;
  }
  // cout << "  OVERALL #CONS:" << constraints.size() << endl;

  // calc constraints' max_timestep
  int max_timestep = -1;
  for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
    if ( get<2>(*it) > max_timestep )
      max_timestep = get<2>(*it);
  // cout << "  Latest constraint's timestep:" << max_timestep << endl;

  // initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
  //  cout << "  Creating a list of constraints (per timestep):" << endl;
  vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > > ( max_timestep+1, list< pair<int, int> > () );
  for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) {
    //    cout << "   PUSHING a constraint for time:" << get<2>(*it) << " ; (constraint is [" << get<0>(*it) << "," << get<1>(*it) << "])" << endl;
    cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
  }

  // build reservation table
  size_t max_plan_len = getPathsMaxLength();
  bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
  updateReservationTable(res_table, max_plan_len, agent_id);

  //  printResTable(res_table, max_plan_len);

  // find a path w.r.t cons_vec (and prioretize by res_table).
  bool foundSol = search_engines[agent_id]->findPath(ll_focal_w, cons_vec, res_table, max_plan_len);
  LL_num_expanded += search_engines[agent_id]->num_expanded;
  LL_num_generated += search_engines[agent_id]->num_generated;

#ifndef NDEBUG
  cout << "Run search for AG" << agent_id << " ; found solution? " << std::boolalpha << foundSol;
#endif
  // update leaf's path to the one found and its low-level search's min f-val
  if (foundSol) {
    leaf_node->path = vector<int>(*(search_engines[agent_id]->getPath()));
    leaf_node->ll_min_f_val = search_engines[agent_id]->min_f_val;
    leaf_node->path_cost = search_engines[agent_id]->path_cost;
#ifndef NDEBUG
    cout << " ; path-cost=" << (leaf_node->path_cost) << " ; for which min-f-val=" << leaf_node->ll_min_f_val << " ; The path is:";
    for (vector<int>::iterator it = leaf_node->path.begin(); it != leaf_node->path.end(); it++)
      cout << *it << " ";
#endif
  }
  //  cout << endl;

  // release memory allocated here and return
  delete (cons_vec);
  delete[] res_table;
  return foundSol;
}
////////////////////////////////////////////////////////////////////////////////

/*
  return agent_id's location for the given timestep
  Note -- if timestep is longer than its plan length,
          then the location remains the same as its last cell)
 */
inline int ECBSSearch::getAgentLocation(int agent_id, size_t timestep) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= paths[agent_id]->size())
    return paths[agent_id]->at(paths[agent_id]->size()-1);
  // otherwise, return its location for that timestep
  return paths[agent_id]->at(timestep);
}


/*
  return true iff agent1 and agent2 switched locations at timestep [t,t+1]
 */
inline bool ECBSSearch::switchedLocations(int agent1_id, int agent2_id, size_t timestep) {
  // if both agents at their goal, they are done moving (cannot switch places)
  if ( timestep >= paths[agent1_id]->size() && timestep >= paths[agent2_id]->size() )
    return false;
  if ( getAgentLocation(agent1_id, timestep) == getAgentLocation(agent2_id, timestep+1) &&
       getAgentLocation(agent1_id, timestep+1) == getAgentLocation(agent2_id, timestep) )
    return true;
  return false;
}


/*
  Emulate agents' paths and returns a vector of collisions
  Note - a collision is a tuple of <int agent1_id, agent2_id, int location1, int location2, int timestep>).
  Note - the tuple's location_2=-1 for vertex collision.
 */
vector< tuple<int, int, int, int, int> >* ECBSSearch::extractCollisions() {
  vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
  earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ) {
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           -1,  // vertex collision (hence loc2=-1)
                                           timestep) );
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, timestep), -1, timestep);
        }
        if ( switchedLocations(a1, a2, timestep) ) {
          cons_found->push_back(make_tuple(a1,
                                           a2,
                                           getAgentLocation(a1, timestep),
                                           getAgentLocation(a2, timestep),
                                           timestep) );
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, timestep), getAgentLocation(a2, timestep), timestep);
        }
      }
    }
  }
  return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Returns the maximal path length (among all agent)
size_t ECBSSearch::getPathsMaxLength() {
  size_t retVal = 0;
  for (int ag = 0; ag < num_of_agents; ag++)
    if ( paths[ag] != NULL && paths[ag]->size() > retVal )
      retVal = paths[ag]->size();
  return retVal;
}


// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
void ECBSSearch::updateReservationTable(bool* res_table, size_t max_path_len, int exclude_agent) {
  for (int ag = 0; ag < num_of_agents; ag++) {
    if (ag != exclude_agent && paths[ag] != NULL) {
      for (size_t timestep = 0; timestep < max_path_len; timestep++) {
        int id = getAgentLocation(ag, timestep);
        res_table[id + timestep*map_size] = true;
      }
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// h_lir (worked well in terms of trade off between time to compute and informativeness. Also, might be admissible...)
// it's basically a quick approximation of the vertex cover
int ECBSSearch::computeNumOfCollidingAgents() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
             switchedLocations(a1, a2, timestep) ) {
          //          cout << "  A" << a1 << " -- A" << a2 << endl;
          retVal++;
          // break to the outer (a1) loop
          timestep = max_path_length;
          a2 = num_of_agents;
        }
      }
    }
  }
  //  cout << "   *-*-* h_lir = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute the number of agents that has at least one conflict (h_2 in ECBS's paper)
int ECBSSearch::compute_h2() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = 0; a2 < num_of_agents; a2++) {
      if (a1 != a2) {
        size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
        for (size_t timestep = 0; timestep < max_path_length; timestep++) {
          //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
          if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
               switchedLocations(a1, a2, timestep) ) {
            //          cout << "  A" << a1 << " -- A" << a2 << endl;
            retVal++;
            timestep = max_path_length;  // break this loop
            a2 = num_of_agents;  // break to the outer (a1) loop
          }
        }
      }
    }
  }
  //  cout << "   *-*-* h_2 = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute the number of pairs of agents colliding (h_3 in ECBS's paper)
int ECBSSearch::compute_h3() {
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
             switchedLocations(a1, a2, timestep) ) {
          //          cout << "  A" << a1 << " -- A" << a2 << endl;
          retVal++;
          timestep = max_path_length;  // break this loop
        }
      }
    }
  }
  //  cout << "   *-*-* h_3 = " << retVal << endl;
  return retVal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
ECBSSearch::ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double ll_f_w, double hl_f_w) :
  ml(&ml), al(&al), egr(&egr) {
  hl_focal_w = hl_f_w;
  ll_focal_w = ll_f_w;
  hwy_w = e_w;
  HL_num_expanded = 0;
  HL_num_generated = 0;
  LL_num_expanded = 0;
  LL_num_generated = 0;
  num_of_agents = al.num_of_agents;
  map_size = ml.rows*ml.cols;
  solution_found = false;
  solution_cost = -1;

  // Anytime stuff (we start from iteration 1 so the following pushes doesn't count).
  search_iterations = 0;
  solution_iterations = vector< vector< vector<int> > >(1, vector< vector<int> >(1, vector<int>(1)) );  // Init a 1x1x1 3D-vec.
  cost_iterations.push_back(std::numeric_limits<double>::max());
  f_weights.push_back(0);
  focal_bounds.push_back(0);
  min_f_vals.push_back(0);
  solution_subopt_bounds.push_back(0);
  runtimes.push_back(0);
  hl_exp_iterations.push_back(0);
  hl_gen_iterations.push_back(0);
  ll_exp_iterations.push_back(0);
  ll_gen_iterations.push_back(0);
  num_nodes_removed_iterations.push_back(0);

  ll_min_f_vals = vector <double> (num_of_agents);
  paths_costs = vector <double> (num_of_agents);
  ll_min_f_vals_found_initially = vector <double> (num_of_agents);
  paths_costs_found_initially = vector <double> (num_of_agents);
  search_engines = vector < SingleAgentECBS* > (num_of_agents);
  for (int i = 0; i < num_of_agents; i++) {
    int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
    int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
    ComputeHeuristic ch(goal_loc, ml.get_map(), ml.rows*ml.cols, ml.actions_offset, e_w, &egr);
    search_engines[i] = new SingleAgentECBS(init_loc, goal_loc,
                                            ch.getHVals(),
                                            ml.get_map(), ml.rows*ml.cols, ml.actions_offset,
                                            &egr,
                                            e_w);
  }

  // initialize allNodes_table (hash table)
  empty_node = new ECBSNode();
  empty_node->time_generated = -2; empty_node->agent_id = -2;
  deleted_node = new ECBSNode();
  deleted_node->time_generated = -3; deleted_node->agent_id = -3;
  allNodes_table.set_empty_key(empty_node);
  allNodes_table.set_deleted_key(deleted_node);

  // initialize all initial paths to NULL
  paths_found_initially.resize(num_of_agents);
  for (int ag=0; ag < num_of_agents; ag++)
    paths_found_initially[ag] = NULL;

  // create the sequential (0,1,2,...,num_of_agents-1) agents ordering
  agents_order = vector<int>(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
    agents_order.at(i) = i;

  // initialize paths_found_initially
  for (int i : agents_order) {
    //    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
    paths = paths_found_initially;
    size_t max_plan_len = getPathsMaxLength();
    bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
    updateReservationTable(res_table, max_plan_len, i);
    //    cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
    //    printResTable(res_table, max_plan_len);
    if ( search_engines[i]->findPath ( ll_focal_w, NULL, res_table, max_plan_len ) == false)
      cout << "NO SOLUTION EXISTS";
    paths_found_initially[i] = new vector<int> (*(search_engines[i]->getPath()));
    ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
    paths_costs_found_initially[i] = search_engines[i]->path_cost;
    LL_num_expanded += search_engines[i]->num_expanded;
    LL_num_generated += search_engines[i]->num_generated;
    delete[] res_table;
    //    cout << endl;
  }

  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;

  // generate dummy start and update data structures
  dummy_start = new ECBSNode();
  dummy_start->agent_id = -1;
  dummy_start->g_val = 0;
  for (int i = 0; i < num_of_agents; i++)
    dummy_start->g_val += paths_costs[i];
  dummy_start->ll_min_f_val = 0;
  dummy_start->sum_min_f_vals = compute_hl_lower_bound();
  dummy_start->open_handle = open_list.push(dummy_start);
  dummy_start->focal_handle = focal_list.push(dummy_start);
  HL_num_generated++;
  dummy_start->time_generated = HL_num_generated;
  allNodes_table[dummy_start] = dummy_start;

  most_recent_goal = nullptr;

  min_sum_f_vals = dummy_start->sum_min_f_vals;
  focal_list_threshold = hl_focal_w * dummy_start->sum_min_f_vals;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ECBSSearch::printAnytimeStats() {
  for (int i=1; i<=search_iterations; i++) {
    cout << "\tIt.#" << i
         << " ; T=" << runtimes[i]
         << " ; Cost=" << cost_iterations[i]
         << " ; min-f-val=" << min_f_vals[i]
         << " ; focal_bound=" << focal_bounds[i]
         << " ; actual_subopt=" << solution_subopt_bounds[i]
         << " ; focal_weight=" << f_weights[i]
         << " ; HL_EXP=" << hl_exp_iterations[i]
         << " ; HL_GEN=" << hl_gen_iterations[i]
         << " ; LL_EXP=" << ll_exp_iterations[i]
         << " ; LL_GEN=" << ll_gen_iterations[i]
         << " ; #Nodes_Removed=" << num_nodes_removed_iterations[i];
    // If we just "proved" that the previous solution was optimal, add * before ending the line.
    if (i > 1 && cost_iterations[i] == cost_iterations[i-1]) {
      cout << " ; *" << endl;
    } else {
      cout << endl;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::anytimeFindPath(double time_limit, double initial_focal_w) {
  std::clock_t total_start, single_run_start;
  double total_duration = 0;
  double single_run_duration = 0;
  double curr_focal_w = initial_focal_w;
  double sol_cost_ub = std::numeric_limits<double>::max();
  bool sol_found = false;
  total_start = std::clock();
  while (total_duration < time_limit &&                          // if not above the time limit
         min_sum_f_vals < cost_iterations[search_iterations] &&  // if the last solution we've found is not proved optimal
         open_list.size() > 0) {                                 // if search is not at dead end
    search_iterations++;
    hl_focal_w = curr_focal_w;
    f_weights.push_back(curr_focal_w);
    sol_cost_ub = cost_iterations[search_iterations-1];  // the upper bound is the sol-cost found in the previous search iteration.
    min_sum_f_vals = open_list.top()->sum_min_f_vals;
    focal_list_threshold = curr_focal_w * min_sum_f_vals;
    // Run a findPath iteration (and time it).
    single_run_start = std::clock();
    if (search_iterations > 1) {
      int num_nodes_removed = updateFocalListFBDecreased(focal_list_threshold, sol_cost_ub);  // focal_list_threshold is the new focal bound for the upcoming search.
      num_nodes_removed_iterations.push_back(num_nodes_removed);
    } else {
      num_nodes_removed_iterations.push_back(0);  // We do not remove any nodes during the first search.
    }
    // handle the case where FOCAL is empty and OPEN is not when starting a search.
    if (focal_list.size() == 0) {
      ECBSNode* open_head = open_list.top();
      open_head->focal_handle = focal_list.push(open_head);
    }
    sol_found = runECBSSearch(time_limit-total_duration, sol_cost_ub);

//      cout << "*** |OPEN|=" << open_list.size() << " ; |FOCAL|=" << focal_list.size()
//           << " ; min_sum_f_vals=" << min_sum_f_vals << " ; open_head->g=" << open_list.top()->g_val
//           << " ; open_head->sum_min_f_vals=" << open_list.top()->sum_min_f_vals << endl;

    single_run_duration = (std::clock() - single_run_start) / (double) CLOCKS_PER_SEC;
    runtimes.push_back(single_run_duration);
    // Update parameters from the search that just ended (via most_recent_goal pointer).
    focal_bounds.push_back(focal_list_threshold);  // should be equal to min_f_vals[search_iterations]*f_weights[search_iteration]
    hl_exp_iterations.push_back(HL_num_expanded);
    hl_gen_iterations.push_back(HL_num_generated);
    ll_exp_iterations.push_back(LL_num_expanded);
    ll_gen_iterations.push_back(LL_num_generated);

    if (sol_found == false) {  // if no solution found
      cost_iterations.push_back(-1);
      if (open_list.size() == 0) {                 // Finished with no solution and no more options (empty OPEN).
        min_f_vals.push_back(-1);
        solution_subopt_bounds.push_back(-1);
      } else if ( (min_sum_f_vals >= sol_cost_ub) || // Finished with proving that the previous iteration's solution is optimal.
                  (focal_list.size() == 0 && open_list.size() > 0) ) {  // Same as above but due to truncation (2).
        min_f_vals.push_back(sol_cost_ub);
        solution_subopt_bounds.push_back(1);
      } else {                                     // Timeout
        min_f_vals.push_back(min_sum_f_vals);
        solution_subopt_bounds.push_back(-1);
      }
      break;
    }
    // If we got here then a new (and strictly cheaper) solution was found in this iteration.
    min_f_vals.push_back(min_sum_f_vals);
    cost_iterations.push_back(most_recent_goal->g_val);
    solution_subopt_bounds.push_back( cost_iterations[search_iterations] / min_f_vals[search_iterations] );
    // Reset the next search's parameters.
    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    most_recent_goal = nullptr;

    curr_focal_w = solution_subopt_bounds[search_iterations] - 0.0001;
    if (curr_focal_w <= 1)  // if optimal solution found
      curr_focal_w = 1;
    total_duration = (std::clock() - total_start) / (double) CLOCKS_PER_SEC;
  }
  //printAnytimeStats();
  return sol_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// returns true only if a *new* solution was found.
// (that is, returns false if we only "proved" that the previous iteration's solution is optimal.)
bool ECBSSearch::runECBSSearch(double time_limit_cutoff, double sol_cost_ub) {
  // set timer
  std::clock_t start;
  double duration;
  start = std::clock();

  solution_found = false;

  // start is already in the open_list
  while ( !focal_list.empty() && !solution_found && open_list.size() > 0 ) {

    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    if (duration > time_limit_cutoff) {  // timeout after 5 minutes
      min_sum_f_vals = open_list.top()->sum_min_f_vals;
      return false;
    }

    // Check whether the lower bound has a cost as the solution upper bound
    // (which means we already found an optimal solution in the previous iteration
    // and only now "proved" it).
    if (open_list.top()->sum_min_f_vals >= sol_cost_ub) {
      min_sum_f_vals = open_list.top()->sum_min_f_vals;
      return false;
    }

    // Pop a node from the head of Focal (and "truncate" it if it's a surplus node).
    ECBSNode* curr = focal_list.top();
    focal_list.pop();
    open_list.erase(curr->open_handle);
    ///*  Upper Bound Improvement
    if (curr->sum_min_f_vals >= sol_cost_ub) {
      continue;
    }
    //*/
    HL_num_expanded++;
    curr->time_expanded = HL_num_expanded;
//        cout << "Expanding: (" << curr << ")" << *curr << " at time:" << HL_num_expanded << endl;

    // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
    updatePaths(curr, dummy_start);
    //    printPaths();

    vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions();  // check for collisions on updated paths
#ifndef NDEBUG
    cout << endl << "****** Expanded #" << curr->time_expanded << " with cost " << curr->g_val << " and # Collisions " << collision_vec->size() << " and |FOCAL|=" << focal_list.size() << " and focal-threshold=" << focal_list_threshold << endl;
#endif
    /*    
    cout << "Collision found in the expanded node's paths:" << endl;
    for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
      cout << "   A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;
    cout << "Overall Col_Vec.size=" << collision_vec->size() << endl;
     */

    if ( collision_vec->size() == 0 ) {  // found a solution (and finish the while look)
      solution_found = true;
      solution_cost = curr->g_val;
      most_recent_goal = curr;
    } else {  // generate the two successors that resolve one of the conflicts
      int agent1_id, agent2_id, location1, location2, timestep;
      tie(agent1_id, agent2_id, location1, location2, timestep) = earliest_conflict;  // choose differently? (used to be collision_vec->at(0))
#ifndef NDEBUG
      cout << "   Earliest collision -- A1:" << agent1_id << " ; A2: " << agent2_id
	   << " ; L1:" << location1 << " ; L2:" << location2 << " ; T:" << timestep << endl;
#endif
      ECBSNode* n1 = new ECBSNode();
      ECBSNode* n2 = new ECBSNode();
      n1->agent_id = agent1_id;
      n2->agent_id = agent2_id;
      if (location2 == -1) {  // generate vertex constraint
        n1->constraint = make_tuple(location1, -1, timestep);
        n2->constraint = make_tuple(location1, -1, timestep);
      } else {  // generate edge constraint
        n1->constraint = make_tuple(location1, location2, timestep);
        n2->constraint = make_tuple(location2, location1, timestep);
      }
      n1->parent = curr;
      n2->parent = curr;
      //      cout << "*** Before solving, " << endl << *n1;
      // find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them. Also updates n1's g_val
      if ( updateECBSNode(n1, dummy_start) == true ) {
        // new g_val equals old g_val plus the new path length found for the agent minus its old path length
        n1->g_val = curr->g_val - paths_costs[n1->agent_id] + n1->path_cost;
        n1->sum_min_f_vals = curr->sum_min_f_vals - ll_min_f_vals[n1->agent_id] + n1->ll_min_f_val;
        if (n1->sum_min_f_vals >= sol_cost_ub) {  // Upper Bound Improvement (2)
          delete(n1);
        } else {
          // update n1's path for computing the num of colliding agents
          vector<int>* temp_old_path = paths[n1->agent_id];
          paths[n1->agent_id] = &(n1->path);
          n1->h_val = computeNumOfCollidingAgents();
          paths[n1->agent_id] = temp_old_path;  // restore the old path (for n2)
          // update lower bounds and handles
          n1->open_handle = open_list.push(n1);
          HL_num_generated++;
          n1->time_generated = HL_num_generated;
          if ( n1->sum_min_f_vals <= focal_list_threshold )
            n1->focal_handle = focal_list.push(n1);
          allNodes_table[n1] = n1;
#ifndef NDEBUG
	cout << endl << "   First node generated for A" << n1->agent_id << ": g-val=" << n1->g_val << " ; h-val=" << n1->h_val << " ; LB=" << n1->sum_min_f_vals << endl;
#endif
        }
      } else {  // if updateECBSNode failed
        delete (n1);
      }
      // same for n2
      //      cout << "*** Before solving, " << endl << *n2;
      if ( updateECBSNode(n2, dummy_start) == true ) {
        n2->g_val = curr->g_val - paths_costs[n2->agent_id] + n2->path_cost;
        n2->sum_min_f_vals = curr->sum_min_f_vals - ll_min_f_vals[n2->agent_id] + n2->ll_min_f_val;
        if (n2->sum_min_f_vals >= sol_cost_ub) {  // Upper Bound Improvement (2)
          delete(n2);
        } else {
          vector<int>* temp_old_path = paths[n2->agent_id];
          paths[n2->agent_id] = &(n2->path);
          n2->h_val = computeNumOfCollidingAgents();
          paths[n2->agent_id] = temp_old_path;
          n2->open_handle = open_list.push(n2);
          HL_num_generated++;
          n2->time_generated = HL_num_generated;
          if ( n2->sum_min_f_vals <= focal_list_threshold )
            n2->focal_handle = focal_list.push(n2);
          allNodes_table[n2] = n2;
#ifndef NDEBUG        
	cout << endl << "   Second node generated for A" << n2->agent_id << ": g-val=" << n2->g_val << " ; h-val=" << n2->h_val << " ; LB=" << n2->sum_min_f_vals << endl;
#endif
        }
      } else {  // if updateECBS failed
        delete (n2);
      }

      //            cout << "It has found the following paths:" << endl;
      //            printPaths();
      //            cout << "Focal threshold: (before) " << focal_list_threshold;

      if (open_list.size() == 0) {
        min_sum_f_vals = -1;
        solution_found = false;
        break;
      }

      ECBSNode* open_head = open_list.top();
      //cout << open_head->sum_min_f_vals << endl;
    if ( open_head->sum_min_f_vals > min_sum_f_vals ) {
#ifndef NDEBUG
	cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
#endif
        min_sum_f_vals = open_head->sum_min_f_vals;
        double new_focal_list_threshold = min_sum_f_vals * hl_focal_w;
        updateFocalListLBIncreased(focal_list_threshold, new_focal_list_threshold);
        focal_list_threshold = new_focal_list_threshold;
	// cout << focal_list.size() << endl;
      }
      //            cout << " ; (after) " << focal_list_threshold << endl << endl;
    }  // end generating successors
    delete (collision_vec);
  }  // end of while loop
  
  // get time
  duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

  // update min_sum_f_vals before returning
//  if (open_list.size() != 0) {
//    cout << "UPDATING MIN_SUM to " << open_list.top()->sum_min_f_vals << endl;
//    min_sum_f_vals = open_list.top()->sum_min_f_vals;
//  }
  
  return solution_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ECBSSearch::~ECBSSearch() {
  for (size_t i = 0; i < search_engines.size(); i++)
    delete (search_engines[i]);
  for (size_t i = 0; i < paths_found_initially.size(); i++)
    delete (paths_found_initially[i]);
  //  for (size_t i=0; i<paths.size(); i++)
  //    delete (paths[i]);
  releaseClosedListNodes();
  delete (empty_node);
  delete (deleted_node);
}
