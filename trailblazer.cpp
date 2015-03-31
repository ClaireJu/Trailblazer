#include "trailblazer.h"
#include "queue.h"
#include "stack.h"
#include "pqueue.h"

bool depthFirstSearchHelper(BasicGraph&, Vertex*, Vertex*, Vector<Vertex*>&);
void reconstrcuctpath(Vertex* v, Vector<Vertex*>& path);
void daHelper(BasicGraph&, Vertex*, Vertex*, Vector<Vertex*>&, int);
bool samecluster(Vertex* start, Vertex* finish, int& s, int& f, Vector<Set<Vertex*>>&);
void mergecluster(int&, int&, Vector<Set<Vertex*>>&);

using namespace std;


/* depthFirstSearch() function uses DFS to find a path between start position and end position.It has
 * three parameters, a graph called by reference, a pointer to starting vertex and a pointer to ending
 * vertex. It returns a vector of all vertexes along the path.
 */
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    depthFirstSearchHelper(graph, start, end, path);
    return path;
}


/* depthFirstSearchHelper() function uses DFS to find a path between start position and end position. Once
 * it finds a path, it stops. It has four parameters, a graph called by reference, a pointer to starting
 * vertex, a pointer to ending vertex and a vector to store vertexes that have been tested along the path .
 * It returns true if a path is found, otherwise false.
 */
bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path) {
    path.add(start);                        //choose
    start->visited = true;
    start->setColor(GREEN);
    if(start == end) {
        return true;
    }
    for(Vertex* neighbor : graph.getNeighbors(start)) {
        if(!neighbor->visited && depthFirstSearchHelper(graph, neighbor, end, path)) {
          return true;
        }
    }
    path[path.size()-1]->setColor(GRAY);
    path.remove(path.size()-1);             //un-choose
    return false;
}


/* breadthFirstSearch() function uses BFS to find a path between start position and end position.It
 * can find the shortest path. It has three parameters, a graph called by reference, a pointer to
 * starting vertex and a pointer to ending vertex. It returns a vector of all vertexes along the path.
 */
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    Queue<Vertex*> q;
    start->visited = true;
    start->setColor(YELLOW);
    q.enqueue(start);
    while(!q.isEmpty()) {
        Vertex* v = q.dequeue();
        v->setColor(GREEN);
        if(v == end) {
            reconstrcuctpath(v, path);
            break;
        }
        for(Vertex* neighbor : graph.getNeighbors(v)) {
            if(!neighbor->visited) {
                neighbor->visited = true;
                neighbor->setColor(YELLOW);
                neighbor->previous = v;
                q.enqueue(neighbor);
            }
        }
    }
    return path;
}


/* reconstructpath() function follows previous pointers to reconstruct path between start position and
 * end position. It has two parameters, a pointer to vertex and a vector of all vertexes along the path.
 */
void reconstrcuctpath(Vertex* v, Vector<Vertex*>& path) {
    Stack<Vertex*> s;
    Vertex* current = v;
    while(current != NULL) {
        s.add(current);                     //a stack of ending vertex first and the starting vertex last.
        current = current->previous;
    }
    while(!s.isEmpty()) {
        path.add(s.pop());                  //a vector of starting vertex first and the ending vertex last.
    }
}


/* dijkstrasAlgorithm() function uses dijkstra's algorithm to find the minimum-weight path between start
 * position and end position.It has three parameters, a graph called by reference, a pointer to starting
 * vertex and a pointer to ending vertex. It returns a vector of all vertexes along the path.
 */
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    daHelper(graph, start, end, path, 1);
    return path;
}


/* daHelper() function uses either dijkstra's algorithm or astar according to the argument command to find
 * the minimum-weight path between start position and end position.It has five parameters, a graph called
 * by reference, a pointer to starting vertex, a pointer to ending vertex, a vector of all vertexes along
 * the path and a int command.
 */
void daHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path, int command) {
    PriorityQueue<Vertex*> pq;
    for(Vertex* v : graph.getVertexSet()) {
        v->cost = POSITIVE_INFINITY;
    }
    start->setColor(YELLOW);
    start->cost = 0;
    if(command == 1){                       //dijkstra's algorithm
        pq.enqueue(start, start->cost);
    }else {                                 //astar
        pq.enqueue(start, heuristicFunction(start, end));
    }
    while(!pq.isEmpty()) {
        Vertex* v = pq.dequeue();
        v->visited = true;
        v->setColor(GREEN);
        if(v == end) {
            reconstrcuctpath(v, path);
            break;
        }
        for(Vertex* neighbor : graph.getNeighbors(v)) {
            if(!neighbor->visited) {
                double cost = v->cost + graph.getEdge(v, neighbor)->cost;
                if(cost <= neighbor->cost) {//new priority should be at least as urgent as existing priority in the queue
                    neighbor->cost = cost;
                    neighbor->previous = v;
                    if(neighbor->getColor() == YELLOW) {   //if vertex is already in the queue
                        if(command == 1){   //dijkstra's algorithm
                            pq.changePriority(neighbor, neighbor->cost);
                        }else {             //astar
                            pq.changePriority(neighbor, cost + heuristicFunction(neighbor, end));
                        }
                    }else {
                        neighbor->setColor(YELLOW);
                        if(command == 1){   //dijkstra's algorithm
                            pq.enqueue(neighbor, neighbor->cost);
                        }else {             //astar
                            pq.enqueue(neighbor, cost + heuristicFunction(neighbor, end));
                        }
                    }
                }
            }
        }
    }
}


/* aStar() function finds the minimum-weight path between start position and end position. It uses
 * heuristics to fine-tune the order of elements in its priority queue to explore more likely desirable
 * elements first. It has three parameters, a graph called by reference, a pointer to starting vertex
 * and a pointer to ending vertex. It returns a vector of all vertexes along the path.
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    daHelper(graph, start, end, path, 2);
    return path;
}


/* kruskal() accepts a graph by reference as a parameter, and returns a set of pointers to edges in
 * the graph such that those edges would connect the graph's vertexes into a minimum spanning tree.
 */
Set<Edge*> kruskal(BasicGraph& graph) {
    graph.resetData();
    Set<Edge*> mst;
    PriorityQueue<Edge*> pq;
    Vector<Set<Vertex*>> clustervector;
    for(Vertex* v : graph.getVertexSet()) { //place each vertex into its own cluster
        Set<Vertex*> s;
        s.add(v);
        clustervector.add(s);
    }
    for(Edge* e : graph.getEdgeSet()) {     //put all edges into a priority queue, using weights as priorities
        pq.enqueue(e, e->cost);
    }
    while(clustervector.size() >= 2) {
        Edge* e = pq.dequeue();
        int s = 0;
        int f = 0;
        if(!samecluster(e->start, e->finish, s, f, clustervector)) {//start and finish vertexes of e are not in the same cluster
            mergecluster(s, f, clustervector);
            mst.add(e);
        }
    }
    return mst;
}


/* samecluster() returns tree if two vertexes are in the same cluster, false otherwise. It has five
 * parameters, two pointers to the two vertexes, two indexes that indicate which cluster the two
 * vertexes belong to respectively, and a vector of set, containing pointers to vertex.
 */
bool samecluster(Vertex* start, Vertex* finish, int& s, int& f, Vector<Set<Vertex*>>& clustervector) {
    for(int i = 1; i < clustervector.size(); i++) {
        if(clustervector[i].contains(start)) {
            s = i;
        }
        if(clustervector[i].contains(finish)) {
            f = i;
        }
    }
    if(s == f) {
        return true;
    } else {
       return false;
    }
}


/* mergecluster() mergers two clusters containing different vertexes. It has three parameters, two indexes
 * of two clusters respectively, and a vector of set, containing pointers to vertex.
 */
void mergecluster(int& s, int& f, Vector<Set<Vertex*>>& clustervector) {
    for(Vertex* v : clustervector[f]) {
        clustervector[s].add(v);                  //merge into one cluster
    }
    clustervector.remove(f);
}
