#include "push_and_rotate.h"

PushAndRotate::PushAndRotate()
{
    search = nullptr;
}

PushAndRotate::PushAndRotate (ISearch *Search)
{
    search = Search;
}

PushAndRotate::~PushAndRotate()
{
    if (search)
        delete search;
}

void PushAndRotate::clear() {
    agentsPaths.clear();
    agentsMoves.clear();
}

bool PushAndRotate::clearNode(const Map &map, AgentSet &agentSet, Node &nodeToClear, const std::unordered_set<Node>& occupiedNodes) {
    auto isGoal = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
        return !agentSet.isOccupied(cur.i, cur.j);
    };

    Dijkstra dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, nodeToClear.i, nodeToClear.j, 0, 0,
                                                    isGoal, true, true, -1, occupiedNodes);
    if (!searchResult.pathfound) {
        return false;
    }
    auto path = *searchResult.lppath;
    for (auto it = std::next(path.rbegin()); it != path.rend(); ++it) {
        if (agentSet.isOccupied(it->i, it->j)) {
            Node from = *it;
            Node to = *std::prev(it);
            agentSet.moveAgent(from, to, agentsMoves);
        }
    }
    return true;
}

bool PushAndRotate::push(const Map &map, AgentSet &agentSet, Node& from, Node& to, std::unordered_set<Node>& occupiedNodes) {
    if(occupiedNodes.find(to) != occupiedNodes.end()) {
        return false;
    }
    if (agentSet.isOccupied(to.i, to.j)) {
        bool inserted = false;
        if (occupiedNodes.find(from) == occupiedNodes.end()) {
            occupiedNodes.insert(from);
            inserted = true;
        }
        bool canClear = clearNode(map, agentSet, to, occupiedNodes);
        if (inserted) {
            occupiedNodes.erase(from);
        }
        if (!canClear) {
            return false;
        }
    }
    agentSet.moveAgent(from, to, agentsMoves);
    return true;
}

bool PushAndRotate::multipush(const Map &map, AgentSet &agentSet, Node first, Node second, Node& to, std::list<Node>& path) {
    if (path.size() > 1 && *std::next(path.begin()) == second) {
        std::swap(first, second);
        path.pop_front();
    }
    Node prevNode = second;
    for (auto it = path.begin(); it != std::prev(path.end()); ++it) {
        Node curNode = *it;
        Node nextNode = *std::next(it);
        std::unordered_set<Node> occupiedNodes = {prevNode, curNode};
        if (agentSet.isOccupied(nextNode.i, nextNode.j)) {
            if (!clearNode(map, agentSet, nextNode, occupiedNodes)) {
                return false;
            }
        }
        agentSet.moveAgent(curNode, nextNode, agentsMoves);
        agentSet.moveAgent(prevNode, curNode, agentsMoves);
        prevNode = curNode;
    }
    return true;
}

bool PushAndRotate::clear(const Map &map, AgentSet &agentSet, Node& first, Node& second) {
    std::list<Node> successors = search->findSuccessors(first, map);
    std::set<Node> unoccupied;
    for (auto node : successors) {
        if (!agentSet.isOccupied(node.i, node.j)) {
            unoccupied.insert(node);
        }
    }
    if (unoccupied.size() >= 2) {
        return true;
    }

    std::unordered_set<Node> forbidden = {first, second};
    forbidden.insert(unoccupied.begin(), unoccupied.end());
    for (auto node : successors) {
        if (unoccupied.find(node) == unoccupied.end() && node != second &&
            clearNode(map, agentSet, node, forbidden)) {
            if (unoccupied.size() >= 1) {
                return true;
            }
            unoccupied.insert(node);
            forbidden.insert(node);
        }
    }
    if (unoccupied.empty()) {
        return false;
    }

    Node freeNeigh = *unoccupied.begin();
    for (auto node : successors) {
        if (node != second && node != freeNeigh) {
            size_t curSize = agentsMoves.size();
            AgentSet newAgentSet = agentSet;
            if (clearNode(map, newAgentSet, node, {first, second})) {
                if (clearNode(map, newAgentSet, freeNeigh, {first, second, node})) {
                    agentSet = newAgentSet;
                    return true;
                }
            }
            agentsMoves.erase(agentsMoves.begin() + curSize, agentsMoves.end());
            break;
        }
    }

    for (auto node : successors) {
        if (node != second && node != freeNeigh) {
            int curSize = agentsMoves.size();
            AgentSet newAgentSet = agentSet;
            newAgentSet.moveAgent(first, freeNeigh, agentsMoves);
            newAgentSet.moveAgent(second, first, agentsMoves);
            if (clearNode(map, newAgentSet, node, {first, second})) {
                if (clearNode(map, newAgentSet, second, {first, second, node})) {
                    agentSet = newAgentSet;
                    return true;
                }
            }
            agentsMoves.erase(agentsMoves.begin() + curSize, agentsMoves.end());
            break;
        }
    }

    int secondAgentId = agentSet.getAgentId(second.i, second.j);
    if (!clearNode(map, agentSet, second, {first})) {
        return false;
    }
    agentSet.moveAgent(first, second, agentsMoves);
    Node secondPosition = agentSet.getAgent(secondAgentId).getCurPosition();
    if (!clearNode(map, agentSet, freeNeigh, {first, second, secondPosition})) {
        return false;
    }
    for (auto node : successors) {
        if (node != second && node != freeNeigh) {
            agentSet.moveAgent(node, first, agentsMoves);
            agentSet.moveAgent(first, freeNeigh, agentsMoves);
            agentSet.moveAgent(second, first, agentsMoves);
            agentSet.moveAgent(secondPosition, second, agentsMoves);
            return clearNode(map, agentSet, freeNeigh, {first, second, node});
        }
    }
    return false;
}

void PushAndRotate::exchange(const Map &map, AgentSet &agentSet, Node& first, Node& second) {
    std::list<Node> successors = search->findSuccessors(first, map);
    std::vector<Node> freeNeigh;
    for (auto node : successors) {
        if (!agentSet.isOccupied(node.i, node.j)) {
            freeNeigh.push_back(node);
        }
    }
    agentSet.moveAgent(first, freeNeigh[0], agentsMoves);
    agentSet.moveAgent(second, first, agentsMoves);
    agentSet.moveAgent(first, freeNeigh[1], agentsMoves);
    agentSet.moveAgent(freeNeigh[0], first, agentsMoves);
    agentSet.moveAgent(first, second, agentsMoves);
    agentSet.moveAgent(freeNeigh[1], first, agentsMoves);
}

void PushAndRotate::reverse(int begSize, int endSize,
                            int firstAgentId, int secondAgentId, AgentSet &agentSet) {
    for (int i = endSize - 1; i >= begSize; --i) {
        AgentMove pos = agentsMoves[i];
        if (pos.id == firstAgentId) {
            pos.id = secondAgentId;
        } else if (pos.id == secondAgentId) {
            pos.id = firstAgentId;
        }
        Node from = agentSet.getAgent(pos.id).getCurPosition();
        Node to = Node(from.i - pos.di, from.j - pos.dj);
        agentSet.moveAgent(from, to, agentsMoves);
    }
}

bool PushAndRotate::swap(const Map &map, AgentSet &agentSet, Node& first, Node& second) {
    int firstAgentId = agentSet.getAgentId(first.i, first.j);
    int secondAgentId = agentSet.getAgentId(second.i, second.j);

    auto isGoal = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
        return map.getCellDegree(cur.i, cur.j) >= 3;
    };

    Dijkstra dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, first.i, first.j, 0, 0, isGoal);
    while (searchResult.pathfound) {
        int begSize = agentsMoves.size();
        AgentSet newAgentSet = agentSet;
        auto path = *searchResult.lppath;
        Node exchangeNode = path.back();
        if (multipush(map, newAgentSet, first, second, exchangeNode, path)) {
            int exchangeAgentId = newAgentSet.getAgentId(exchangeNode.i, exchangeNode.j);
            int neighAgentId = (exchangeAgentId == firstAgentId) ? secondAgentId : firstAgentId;
            Node neigh = newAgentSet.getAgent(neighAgentId).getCurPosition();
            if (clear(map, newAgentSet, exchangeNode, neigh)) {
                agentSet = newAgentSet;
                int endSize = agentsMoves.size();
                exchange(map, agentSet, exchangeNode, neigh);
                reverse(begSize, endSize, firstAgentId, secondAgentId, agentSet);
                return true;
            }
        }
        searchResult = dijkstraSearch.startSearch(map, agentSet, first.i, first.j, 0, 0, isGoal, false);
    }
    return false;
}

bool PushAndRotate::rotate(const Map &map, AgentSet &agentSet, std::vector<Node> &qPath, int cycleBeg) {
    int size = qPath.size() - cycleBeg;
    for (int i = cycleBeg; i < qPath.size(); ++i) {
        if (!agentSet.isOccupied(qPath[i].i, qPath[i].j)) {
            for (int j = 0; j < size - 1; ++j) {
                int from = cycleBeg + (i - cycleBeg - j - 1 + size) % size;
                int to = cycleBeg + (i - cycleBeg - j + size) % size;
                if (agentSet.isOccupied(qPath[from].i, qPath[from].j)) {
                    agentSet.moveAgent(qPath[from], qPath[to], agentsMoves);
                }
            }
            return true;
        }
    }

    std::unordered_set<Node> cycleNodes(qPath.begin() + cycleBeg, qPath.end());
    for (int i = cycleBeg; i < qPath.size(); ++i) {
        cycleNodes.erase(qPath[i]);
        int firstAgentId = agentSet.getAgentId(qPath[i].i, qPath[i].j);
        int begSize = agentsMoves.size();
        if (clearNode(map, agentSet, qPath[i], cycleNodes)) {
            int endSize = agentsMoves.size();
            int secondAgentIndex = cycleBeg + (i - cycleBeg - 1 + size) % size;
            int secondAgentId = agentSet.getAgentId(qPath[secondAgentIndex].i, qPath[secondAgentIndex].j);
            agentSet.moveAgent(qPath[secondAgentIndex], qPath[i], agentsMoves);
            Node curPosition = agentSet.getAgent(firstAgentId).getCurPosition();
            swap(map, agentSet, qPath[i], curPosition);
            for (int j = 0; j < size - 1; ++j) {
                int from = cycleBeg + (i - cycleBeg - j - 2 + size) % size;
                int to = cycleBeg + (i - cycleBeg - j - 1 + size) % size;
                if (agentSet.isOccupied(qPath[from].i, qPath[from].j)) {
                    agentSet.moveAgent(qPath[from], qPath[to], agentsMoves);
                }
            }
            reverse(begSize, endSize, firstAgentId, secondAgentId, agentSet);
            return true;
        }
        cycleNodes.insert(qPath[i]);
    }
    return false;
}

void PushAndRotate::getPaths(AgentSet &agentSet) {
    agentsPaths.resize(agentSet.getAgentCount());
    std::vector<Node> agentPositions;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Node startPosition = agentSet.getAgent(i).getStartPosition();
        agentPositions.push_back(startPosition);
        agentsPaths[i].push_back(startPosition);
    }
    for (int i = 0; i < agentsMoves.size(); ++i) {
        agentPositions[agentsMoves[i].id].i += agentsMoves[i].di;
        agentPositions[agentsMoves[i].id].j += agentsMoves[i].dj;
        for (int j = 0; j < agentSet.getAgentCount(); ++j) {
            agentsPaths[j].push_back(agentPositions[j]);
        }
    }
}

void PushAndRotate::getParallelPaths(AgentSet &agentSet) {
    int agentCount = agentSet.getAgentCount();
    std::vector<std::vector<Node>> agentsPositions(agentCount);
    std::vector<int> agentInd(agentCount, 0);
    std::unordered_map<Node, std::vector<int>> nodesOccupations;
    std::unordered_map<Node, int> nodeInd;

    agentsPaths.resize(agentSet.getAgentCount());
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Node startPosition = agentSet.getAgent(i).getStartPosition();
        agentsPositions[i].push_back(startPosition);
        agentsPaths[i].push_back(startPosition);
        if (nodesOccupations.find(startPosition) == nodesOccupations.end()) {
            nodesOccupations[startPosition] = {};
            nodeInd[startPosition] = 0;
        }
        nodesOccupations[startPosition].push_back(i);
    }

    for (auto move : agentsMoves) {
        Node cur = agentsPositions[move.id].back();
        cur.i += move.di;
        cur.j += move.dj;
        if (nodesOccupations.find(cur) == nodesOccupations.end()) {
            nodesOccupations[cur] = {};
            nodeInd[cur] = 0;
        }
        if (!nodesOccupations[cur].empty() && nodesOccupations[cur].back() == move.id) {
            while(agentsPositions[move.id].back() != cur) {
                Node curBack = agentsPositions[move.id].back();
                int lastInd;
                for (lastInd = nodesOccupations[curBack].size() - 1;
                     lastInd >= 0 && nodesOccupations[curBack][lastInd] != move.id; --lastInd);
                nodesOccupations[curBack].erase(nodesOccupations[curBack].begin() + lastInd);
                agentsPositions[move.id].pop_back();
            }
        } else {
            agentsPositions[move.id].push_back(cur);
            nodesOccupations[cur].push_back(move.id);
        }
    }

    std::vector<bool> finished(agentCount, false);
    while (true) {
        std::vector<bool> hasMoved(agentCount, false);
        for (int i = 0; i < agentCount; ++i) {
            if (hasMoved[i] || finished[i]) {
                continue;
            }

            std::vector<int> path;
            int curAgent = i;
            bool canMove = true;
            while (true) {
                path.push_back(curAgent);
                Node nextNode = agentsPositions[curAgent][agentInd[curAgent] + 1];
                int lastInd = nodeInd[nextNode];
                if (nodesOccupations[nextNode][lastInd] == curAgent) {
                    break;
                } else if (nodesOccupations[nextNode][lastInd + 1] == curAgent) {
                    int nextAgent = nodesOccupations[nextNode][lastInd];
                    if (finished[nextAgent] || hasMoved[nextAgent] || nextAgent < curAgent ||
                            agentsPositions[nextAgent][agentInd[nextAgent]] != nextNode) {
                        canMove = false;
                        break;
                    }
                    curAgent = nextAgent;
                    if (curAgent == i) {
                        break;
                    }
                } else {
                    canMove = false;
                    break;
                }
            }

            if (canMove) {
                for (int agentId : path) {
                    hasMoved[agentId] = true;
                    ++nodeInd[agentsPositions[agentId][agentInd[agentId]]];
                    ++agentInd[agentId];
                    agentsPaths[agentId].push_back(agentsPositions[agentId][agentInd[agentId]]);
                    if (agentInd[agentId] == agentsPositions[agentId].size() - 1) {
                        finished[agentId] = true;
                    }
                }
            } else {
                agentsPaths[i].push_back(agentsPositions[i][agentInd[i]]);
            }
        }

        if (!std::any_of(hasMoved.begin(), hasMoved.end(), [](const bool x) {return x;})) {
            break;
        }
    }
}

bool PushAndRotate::solve(const Map &map, const Config &config, AgentSet &agentSet, std::chrono::steady_clock::time_point begin) {
    auto comparator = [&agentSet](int id1, int id2) {
        int subgraph1 = agentSet.getAgent(id1).getSubgraph();
        int subgraph2 = agentSet.getAgent(id2).getSubgraph();

        if (subgraph1 != subgraph2) {
            if (subgraph1 == -1 || agentSet.hasPriority(subgraph2, subgraph1)) {
                return false;
            } else if (subgraph2 == -1 || agentSet.hasPriority(subgraph1, subgraph2)) {
                return true;
            }
        }
        return id1 < id2;
    };

    std::set<int, decltype(comparator)> notFinished(comparator);
    std::unordered_set<int> finished;
    std::unordered_set<Node> qPathNodes, finishedPositions;
    std::vector<Node> qPath;

    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        notFinished.insert(i);
    }

    bool isPolygon = true;
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (!map.CellIsObstacle(i, j) && map.getCellDegree(i, j) != 2) {
                isPolygon = false;
                break;
            }
        }
        if (!isPolygon) {
            break;
        }
    }

    int curAgentId = -1;
    Agent curAgent;
    while (!notFinished.empty()) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            return false;
        }

        if (curAgentId == -1) {
            curAgent = agentSet.getAgent(*notFinished.begin());
        } else {
            curAgent = agentSet.getAgent(curAgentId);
        }
        notFinished.erase(curAgent.getId());

        SearchResult searchResult = search->startSearch(map, agentSet,
                                                        curAgent.getCur_i(), curAgent.getCur_j(),
                                                        curAgent.getGoal_i(), curAgent.getGoal_j(),
                                                        nullptr, true, true, -1,
                                                        isPolygon ? finishedPositions : std::unordered_set<Node>());

        if (!searchResult.pathfound) {
            return false;
        }

        auto path = *searchResult.lppath;
        qPath.push_back(*path.begin());
        qPathNodes.insert(*path.begin());
        for (auto it = path.begin(); it != std::prev(path.end()); ++it) {
            if (qPathNodes.find(*std::next(it)) != qPathNodes.end()) {
                int cycleBeg;
                for (cycleBeg = qPath.size() - 1; cycleBeg >= 0 && qPath[cycleBeg] != *std::next(it); --cycleBeg);
                rotate(map, agentSet, qPath, cycleBeg);

                bool toErase = false;
                while(qPath.size() != cycleBeg) {
                    Node lastNode = qPath.back();
                    if (agentSet.isOccupied(lastNode.i, lastNode.j) &&
                            finished.find(agentSet.getAgentId(lastNode.i, lastNode.j)) != finished.end()) {
                        if (!toErase) {
                            finishedPositions.insert(lastNode);
                            toErase = true;
                        }
                    } else {
                        if (toErase) {
                            finishedPositions.erase(lastNode);
                            toErase = false;
                        }
                    }
                    qPathNodes.erase(lastNode);
                    qPath.pop_back();
                }
            } else if (!push(map, agentSet, *it, *std::next(it), finishedPositions)) {
                swap(map, agentSet, *it, *std::next(it));
                if (finished.find(agentSet.getAgentId(it->i, it->j)) != finished.end()) {
                    finishedPositions.erase(*std::next(it));
                    finishedPositions.insert(*it);
                }
            }
            qPath.push_back(*std::next(it));
            qPathNodes.insert(*std::next(it));
        }
        finished.insert(curAgent.getId());
        finishedPositions.insert(curAgent.getGoalPosition());

        curAgentId = -1;
        while (!qPath.empty()) {
            Node lastNode = qPath.back();
            if (agentSet.isOccupied(lastNode.i, lastNode.j)) {
                Agent curAgent = agentSet.getAgent(agentSet.getAgentId(lastNode.i, lastNode.j));
                Node goal = Node(curAgent.getGoal_i(), curAgent.getGoal_j());
                if (notFinished.find(curAgent.getId()) == notFinished.end() && lastNode != goal) {
                    if (!agentSet.isOccupied(goal.i, goal.j)) {
                        agentSet.moveAgent(lastNode, goal, agentsMoves);
                        finishedPositions.erase(lastNode);
                        finishedPositions.insert(goal);
                    } else {
                        curAgentId = agentSet.getAgentId(goal.i, goal.j);
                        break;
                    }
                }
            }
            qPathNodes.erase(lastNode);
            qPath.pop_back();
        }
    }
    return true;
}

void PushAndRotate::getComponent(AgentSet &agentSet, std::pair<Node, Node> &startEdge,
                                 std::vector<std::pair<Node, Node>> &edgeStack,
                                 std::vector<std::unordered_set<Node>>& components) {
    std::unordered_set<Node> component;
    std::pair<Node, Node> curEdge;
    do {
        curEdge = edgeStack.back();
        component.insert(curEdge.first);
        component.insert(curEdge.second);
        edgeStack.pop_back();
    } while (curEdge != startEdge);
    if (component.size() <= 2) {
        return;
    }
    for (auto node : component) {
        agentSet.setNodeSubgraph(node.i, node.j, components.size());
    }
    components.push_back(component);

}

void PushAndRotate::combineNodeSubgraphs(AgentSet &agentSet, std::vector<std::unordered_set<Node>>& components,
                                         Node &subgraphNode, int subgraphNum) {
    std::vector<int> subgraphs = agentSet.getSubgraphs(subgraphNode.i, subgraphNode.j);
    for (int j = 0; j < subgraphs.size(); ++j) {
        if (subgraphs[j] != subgraphNum) {
            for (auto node : components[subgraphs[j]]) {
                agentSet.removeSubgraphs(node.i, node.j);
                agentSet.setNodeSubgraph(node.i, node.j, subgraphNum);
            }
            components[subgraphs[j]].clear();
        }
    }
}

void PushAndRotate::getSubgraphs(const Map &map, AgentSet &agentSet) {
    std::unordered_set<Node> close;
    std::vector<std::unordered_set<Node>> components;
    std::unordered_set<Node> joinNodes;
    int connectedComponentNum = 0;
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (!map.CellIsObstacle(i, j)) {
                Node curNode = Node(i, j);
                if (close.find(curNode) == close.end()) {
                    int oldSize = close.size();
                    std::vector<std::pair<Node, Node>> edgeStack;
                    std::unordered_map<Node, int> in, up;
                    std::vector<std::tuple<Node, int, int>> stack = {std::make_tuple(curNode, -1, 0)};

                    while (!stack.empty()) {
                        std::tuple<Node, int, int> state = stack.back();
                        Node cur = std::get<0>(state);
                        int lastInd = std::get<1>(state);
                        int depth = std::get<2>(state);

                        std::list<Node> successors = search->findSuccessors(cur, map);
                        std::list<Node>::iterator it = successors.begin();
                        for (int i = 0; i < lastInd; ++i, ++it) {}
                        if (lastInd == -1) {
                            close.insert(cur);
                            agentSet.setConnectedComponent(cur.i, cur.j, connectedComponentNum);
                            in[cur] = depth;
                            up[cur] = depth;
                        } else {
                            if ((depth != 0 && up[*it] >= in[cur]) || depth == 0) {
                                std::pair<Node, Node> curEdge = std::make_pair(cur, *it);
                                getComponent(agentSet, curEdge, edgeStack, components);
                                if (depth != 0) {
                                    joinNodes.insert(cur);
                                }
                            }
                            up[cur] = std::min(up[*it], up[cur]);
                            it = std::next(it);
                        }
                        for (it, lastInd = lastInd + 1; it != successors.end(); ++it, ++lastInd) {
                            if (close.find(*it) != close.end()) {
                                up[cur] = std::min(in[*it], up[cur]);
                            } else {
                                std::pair<Node, Node> curEdge = std::make_pair(cur, *it);
                                edgeStack.push_back(curEdge);
                                std::get<1>(stack.back()) = lastInd;
                                stack.push_back(std::make_tuple(*it, -1, depth + 1));
                                break;
                            }
                        }
                        if (it == successors.end()) {
                            stack.pop_back();
                        }
                    }

                    agentSet.addComponentSize(close.size() - oldSize);
                    ++connectedComponentNum;
                }
            }
        }
    }

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (!map.CellIsObstacle(i, j) && map.getCellDegree(i, j) >= 3 && agentSet.getSubgraphs(i, j).empty()) {
                agentSet.setNodeSubgraph(i, j, components.size());
                components.push_back({Node(i, j)});
                joinNodes.insert(Node(i, j));
            }
        }
    }

    int m = map.getEmptyCellCount() - agentSet.getAgentCount();
    auto isGoal = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
        std::vector<int> startSubgraphs = agentSet.getSubgraphs(start.i, start.j);
        std::vector<int> curSubgraphs = agentSet.getSubgraphs(cur.i, cur.j);
        return curSubgraphs.size() > 1 || curSubgraphs.size() == 1 && curSubgraphs[0] != startSubgraphs[0];
    };

    std::vector<int> order;
    for (int i = 0; i < components.size(); ++i) {
        order.push_back(i);
    }
    std::sort(order.begin(), order.end(),
              [&components](const int a, const int b) {return components[a].size() > components[b].size();});

    for (int i : order) {
        for (auto start : components[i]) {
            if (joinNodes.find(start) != joinNodes.end()) {
                combineNodeSubgraphs(agentSet, components, start, i);
                Dijkstra dijkstraSearch;
                SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, start.i, start.j, 0, 0,
                                                                       isGoal, true, true, m - 2, components[i]);
                while (searchResult.pathfound) {
                    auto path = *searchResult.lppath;
                    for (auto it = std::next(path.begin()); std::next(it) != path.end(); ++it) {
                        if (agentSet.getSubgraphs(it->i, it->j).empty()) {
                            agentSet.setNodeSubgraph(it->i, it->j, i);
                        }
                    }
                    combineNodeSubgraphs(agentSet, components, path.back(), i);
                    searchResult = dijkstraSearch.startSearch(map, agentSet, start.i, start.j, 0, 0,
                                                                           isGoal, false, m - 2);
                }
            }
        }
    }
}

int PushAndRotate::getReachableNodesCount(const Map &map, AgentSet &agentSet, Node &start,
                                         bool (*condition)(const Node&, const Node&, const Map&, const AgentSet&),
                                         const std::unordered_set<Node> &occupiedNodes) {
    int res = 0;
    Dijkstra dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, start.i, start.j, 0, 0,
                                                           condition, true, false, -1, occupiedNodes);
    while (searchResult.pathfound) {
        ++res;
        searchResult = dijkstraSearch.startSearch(map, agentSet, start.i, start.j, 0, 0,
                                                               condition, false, false, -1, occupiedNodes);
    }
    return res;
}

void PushAndRotate::assignToSubgraphs(const Map &map, AgentSet &agentSet) {
    auto isUnoccupied = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
        return !agentSet.isOccupied(cur.i, cur.j);
    };

    std::vector<int> agentsInConnectedComponents(agentSet.getConnectedComponentsCount());
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Agent agent = agentSet.getAgent(i);
        ++agentsInConnectedComponents[agentSet.getConnectedComponent(agent.getCur_i(), agent.getCur_j())];
    }

    int m = map.getEmptyCellCount() - agentSet.getAgentCount();
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (map.CellIsObstacle(i, j)) {
                continue;
            }
            Node pos(i, j);
            auto subgraphs = agentSet.getSubgraphs(pos.i, pos.j);
            if (subgraphs.empty()) {
                continue;
            }
            int subgraph = subgraphs[0];
            auto successors = search->findSuccessors(pos, map);
            int totalCount = agentSet.getComponentSize(pos.i, pos.j) -
                    agentsInConnectedComponents[agentSet.getConnectedComponent(pos.i, pos.j)];
            int throughPos = 0;
            bool hasSuccessorsInOtherSubgraph = false;
            for (auto neigh : successors) {
                auto neighSubgraphs = agentSet.getSubgraphs(neigh.i, neigh.j);
                if (neighSubgraphs.empty() || neighSubgraphs[0] != subgraph) {
                    hasSuccessorsInOtherSubgraph = true;
                    int throughNeigh = getReachableNodesCount(map, agentSet, neigh, isUnoccupied, {pos});
                    int m1 = totalCount - throughNeigh;
                    if (m1 >= 1 && m1 < m && agentSet.isOccupied(i, j)) {
                        agentSet.setAgentSubgraph(agentSet.getAgentId(pos.i, pos.j), subgraph);
                    }
                    auto isGoal = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
                        return map.getCellDegree(cur.i, cur.j) == 1 || !agentSet.getSubgraphs(cur.i, cur.j).empty();
                    };
                    Dijkstra dijkstraSearch;
                    SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, neigh.i, neigh.j,
                                                                           0, 0, isGoal, true, true, -1, {pos});
                    auto path = *searchResult.lppath;
                    int agentCount = 0;
                    for (auto node : path) {
                        if (agentSet.isOccupied(node.i, node.j)) {
                            if (agentCount >= m1 - 1) {
                                break;
                            }
                            agentSet.setAgentSubgraph(agentSet.getAgentId(node.i, node.j), subgraph);
                            ++agentCount;
                        }
                    }
                    throughPos += throughNeigh;
                }
            }
            if (agentSet.isOccupied(i, j) && (!hasSuccessorsInOtherSubgraph || totalCount - throughPos >= 1)) {
                agentSet.setAgentSubgraph(agentSet.getAgentId(pos.i, pos.j), subgraph);
            }
        }
    }
}

void PushAndRotate::getPriorities(const Map &map, AgentSet &agentSet) {
    std::unordered_map<Node, int> goalPositions;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        goalPositions[agentSet.getAgent(i).getGoalPosition()] = i;
    }

    auto isGoal = [](const Node &start, const Node &cur, const Map &map, const AgentSet &agentSet) {
        return !agentSet.getSubgraphs(cur.i, cur.j).empty();
    };
    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (map.CellIsObstacle(i, j)) {
                continue;
            }

            auto subgraphs = agentSet.getSubgraphs(i, j);
            if (subgraphs.empty()) {
                continue;
            }
            int subgraph = subgraphs[0];
            auto successors = search->findSuccessors(Node(i, j), map);
            for (auto neigh : successors) {
                auto neighSubgraphs = agentSet.getSubgraphs(neigh.i, neigh.j);
                if (neighSubgraphs.empty() || neighSubgraphs[0] != subgraph) {
                    Dijkstra dijkstraSearch;
                    SearchResult searchResult = dijkstraSearch.startSearch(map, agentSet, neigh.i, neigh.j,
                                                                           0, 0, isGoal, true, true, -1, {Node(i, j)});
                    if (!searchResult.pathfound) {
                        continue;
                    }
                    auto path = *searchResult.lppath;
                    path.push_front(Node(i, j));
                    for (auto node : path) {
                        auto it = goalPositions.find(node);
                        if (it == goalPositions.end()) {
                            break;
                        }
                        Agent agent = agentSet.getAgent(it->second);
                        int agentSubgraph = agent.getSubgraph();
                        if (agentSubgraph != -1) {
                            if (agentSubgraph != subgraph) {
                                agentSet.setPriority(subgraph, agentSubgraph);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}


MultiagentSearchResult PushAndRotate::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    //std::cout << agentSet.getAgentCount() << std::endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    getSubgraphs(map, agentSet);

    AgentSet goalAgentSet = agentSet;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Node goal = agentSet.getAgent(i).getGoalPosition();
        goalAgentSet.setAgentPosition(i, goal);
    }
    assignToSubgraphs(map, agentSet);
    assignToSubgraphs(map, goalAgentSet);
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        if (agentSet.getAgent(i).getSubgraph() != goalAgentSet.getAgent(i).getSubgraph()) {
            result.pathfound = false;
            return result;
        }
    }
    getPriorities(map, agentSet);

    result.pathfound = solve(map, config, agentSet, begin);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    if (elapsedMilliseconds > config.maxTime) {
        result.pathfound = false;
    }
    if (result.pathfound) {
        if (config.parallelizePaths) {
            getParallelPaths(agentSet);
        } else {
            getPaths(agentSet);
        }
        result.agentsMoves = &agentsMoves;
        result.agentsPaths = &agentsPaths;
        result.time = static_cast<double>(elapsedMilliseconds) / 1000;
    }
    return result;
}
