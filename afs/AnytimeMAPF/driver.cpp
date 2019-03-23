#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
#include "ecbs_search.h"
#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <boost/program_options.hpp>

using namespace boost::program_options;
using namespace std;

int main(int argc, char** argv) {

  // Reading arguments ----------------------------------------------------------------
  string map_fname, agents_fname, hwy_fname, results_fname;
  double w_hwy, ll_w_focal, hl_w_focal;
  int time_limit;  // random restarts iterations number
  try {
    options_description desc("Options");
    desc.add_options() 
      ("help", "Print help messages")
      ("map", value<string>(&map_fname)->required(), "Map filename")
      ("agents", value<string>(&agents_fname)->required(), "Agents filename")
      ("highway", value<string>(&hwy_fname)->default_value("EMPTY"), "Highway filename or EMPTY")
      ("ll_focal_w", value<double>(&ll_w_focal)->default_value(1), "Low-level Focal weight (fixed).")
      ("hl_focal_w", value<double>(&hl_w_focal)->default_value(10.0), "Initial High-level Focal weight (anytime part).")
      ("highway_w", value<double>(&w_hwy)->default_value(1.0), "Highway weight")
      ("time_limit",value<int>(&time_limit)->default_value(300), "Time limit cutoff [seconds]")
      ("export_results", value<string>(&results_fname)->default_value("NONE"), "Results filename")
      ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) { 
      cout << endl << desc << endl;
      return 0;
    }
    notify(vm);
  } catch(boost::program_options::required_option& e) {
    cout << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return 0;
  }
  catch (exception& e) {
    cerr << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return -1;
  }
  // ---------------------------------------------------------------------------------- 

  // read the map file and construct its two-dim array
  MapLoader ml = MapLoader(map_fname);
  
  // read agents' start and goal locations
  AgentsLoader al = AgentsLoader(agents_fname);

  // read the egraph (egraph file, experience_weight, weigthedastar_weight)
  EgraphReader egr;
  if (hwy_fname.compare("EMPTY") != 0) {
    cout << "\t Reading EGraph." << endl;
    EgraphReader egr = EgraphReader(hwy_fname);
  }

  cout << "\nMap ; Agents ; hwy_fname ; w_hwy ; ll_w_focal ; hl_w_focal ; time_limit ;" << endl;
  cout << map_fname << " ; "
       << agents_fname << " ; "
       << hwy_fname << " ; "
       << w_hwy << " ; "
       << ll_w_focal << " ; "
       << hl_w_focal << " ; "
       << time_limit << endl;

  std::clock_t start_time = std::clock();
  ECBSSearch ecbs = ECBSSearch(ml, al, egr, w_hwy, ll_w_focal, hl_w_focal);
  double build_root_duration = (std::clock() - start_time) / (double) CLOCKS_PER_SEC;

  bool res = ecbs.anytimeFindPath(time_limit-build_root_duration, hl_w_focal);
  vector<double> durations(ecbs.runtimes.size());
  // runtimes have individual iteration runtimes.
  // durations will have cumulative.
  durations[1] = build_root_duration + ecbs.runtimes[1];
  for (int i=2; i<(int)ecbs.runtimes.size(); i++)
    durations[i] = durations[i-1] + ecbs.runtimes[i];

  // same for LL EXP.
  vector<long int> total_ll_exp(ecbs.ll_exp_iterations.size());
  total_ll_exp[1] = ecbs.ll_exp_iterations[1];
  for (int i=2; i<(int)ecbs.ll_exp_iterations.size(); i++)
    total_ll_exp[i] = total_ll_exp[i-1] + ecbs.ll_exp_iterations[i];

/*
  for (int i=1; i<=ecbs.search_iterations; i++) {
    cout << "\tIt.#" << i
    << " ; T=" << durations[i]
    << " ; Cost=" << ecbs.cost_iterations[i]
    << " ; min-f-val=" << ecbs.min_f_vals[i]
    << " ; focal_bound=" << ecbs.focal_bounds[i]
    << " ; actual_subopt=" << ecbs.solution_subopt_bounds[i]
    << " ; focal_weight=" << ecbs.f_weights[i]
    << " ; HL_EXP=" << ecbs.hl_exp_iterations[i]
    << " ; HL_GEN=" << ecbs.hl_gen_iterations[i]
    << " ; LL_EXP=" << ecbs.ll_exp_iterations[i]
    << " ; LL_GEN=" << ecbs.ll_gen_iterations[i]
    << " ; #Nodes_Removed=" << ecbs.num_nodes_removed_iterations[i];
    // If we just "proved" that the previous solution was optimal, add * before ending the line.
    if (i > 1 && ecbs.cost_iterations[i] == ecbs.cost_iterations[i-1]) {
      cout << " ; *" << endl;
    } else {
      cout << endl;
    }
  }
*/

  if (results_fname.compare("NONE") != 0 ) {
    ofstream res_f;
    res_f.open(results_fname, ios::trunc);  // append the results file.
    res_f << "IT,TIME,COST,SUBOPT,EXP(LL)\n";  // add header.
    for (int i=1; i<=ecbs.search_iterations; i++) {
      res_f << i << "," << durations[i] << "," << ecbs.cost_iterations[i]
      << "," << ecbs.solution_subopt_bounds[i] << "," << total_ll_exp[i] << "\n";
    }
    res_f.close();
  }
}
