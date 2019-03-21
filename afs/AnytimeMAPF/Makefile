CXX = g++ # the compiler 
CXXFLAGS = -Wall -DNDEBUG -I /usr/include/boost/ -I /usr/local/lib -std=c++14 -lboost_program_options -lboost_graph -O3

all: executable

map_reader.o: map_loader.cpp map_loader.h
	$(CXX) $(CXXFLAGS) -c map_loader.cpp

agents_loader.o: agents_loader.cpp agents_loader.h
	$(CXX) $(CXXFLAGS) -c agents_loader.cpp

compute_heuristic.o: compute_heuristic.cpp compute_heuristic.h
	$(CXX) $(CXXFLAGS) -c compute_heuristic.cpp

node.o: node.cpp node.h
	$(CXX) $(CXXFLAGS) -c node.cpp

egraph_reader.o: egraph_reader.cpp egraph_reader.h
	$(CXX) $(CXXFLAGS) -c egraph_reader.cpp

single_agent_ecbs.o: single_agent_ecbs.cpp single_agent_ecbs.h
	$(CXX) $(CXXFLAGS) -c single_agent_ecbs.cpp

ecbs_node.o: ecbs_node.cpp ecbs_node.h
	$(CXX) $(CXXFLAGS) -c ecbs_node.cpp

ecbs_search.o: ecbs_search.cpp ecbs_search.h
	$(CXX) $(CXXFLAGS) -c ecbs_search.cpp

driver.o: driver.cpp
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c driver.cpp

clean:
	rm -rf *o *~ driver driver.log core mapf_gui hwy_driver

backup:
	tar czvf backup.tar.gz *h *cpp Makefile

executable: driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o single_agent_ecbs.o ecbs_node.o ecbs_search.o
	$(CXX) driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o single_agent_ecbs.o ecbs_node.o ecbs_search.o $(CXXFLAGS) -o driver
