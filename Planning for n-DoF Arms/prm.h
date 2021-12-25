#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <limits>
#include <queue>
#include <set>
#include <cmath>
#include <chrono>
#include <algorithm>

typedef std::vector<double> CSpaceVertex ; // Storing n DOF dimension points of C free


typedef std::vector<std::pair<float,CSpaceVertex>> adjList ; // Adjacency list for storing neighbouring vertexes in C free
// Adj List - Pair <cost,vertex>

void printCSpaceVertex(CSpaceVertex v)
{
    for(int i = 0; i < v.size();i++)
    {
        std::cout << v[i] << " " ;
    }
    std::cout << std::endl;
}


/* 

ConnectedComponenets function - Returns true if two vertices v1 and v2 are connected in the graph. Else returns False.
The function runs a breadth first search in the graph from source vertex v1, and checks if v2 is reachable.

*/

bool ConnectedComponents(CSpaceVertex v1, CSpaceVertex v2, std::map<CSpaceVertex,adjList>& graph) 
{


    if(v1 == v2) return true; // If already same, return

    std::set<CSpaceVertex> visited; // If vertex present in this set, it has been visited by DFS/BFS
 
    // Create a queue for BFS
    std::queue<CSpaceVertex> q;
 
    // Mark the current node as visited and enqueue it
    visited.insert(v1);
    q.push(v1);
 
    while(!q.empty())
    {
        // Dequeue a vertex from queue and print it
        CSpaceVertex temp =q.front();
        q.pop();
 
        // Get all adjacent vertices of the dequeued vertex s
        // If a adjacent has not been visited, then mark it
        // visited  and enqueue it   
        auto it = graph.find(temp);
        auto adj = it->second;  
        for(auto x:adj)
        {
 
            // If this adjacent node is the destination node,
            // then return true
            if(x.second == v2)
                return true;
 
            // Else, continue to do BFS           
            if(visited.find(x.second) == visited.end()) // If x not found in visited, it hasnt been visited yet
            {
                visited.insert(x.second);
                q.push(x.second);
            }
        }
    }
 
 // If BFS is complete without visiting d
    return false;
}


/*
 VertexDistance - Calculates the euclidean distance between two vertices(configurations) in CSpace
*/

float VertexDistance(CSpaceVertex v1, CSpaceVertex v2)
{
    float sum = 0;
    for(int i = 0; i < v1.size();i++)
    {
        sum += abs((v1[i] - v2[i])*(v1[i] - v2[i]));
        

    }
    return sqrt(sum);
}


/*
To add a edge between given configurations v1 and v2. Adds v1 to adjacency list of v2 and vice versa. 
*/

void addEdgetoPRM(CSpaceVertex v1, CSpaceVertex v2, std::map<CSpaceVertex,adjList>& graph) // Add v2 vertex to v1 and vice versa

{
    float dist = VertexDistance(v1,v2);

    // Add v2 to v1
    auto it = graph.find(v1);
    it->second.push_back(std::make_pair(dist,v2));

    // Add v1 to v2
    it = graph.find(v2);
    it->second.push_back(std::make_pair(dist,v1));
}


/*
NearestNeighboursPRM() - Takes as input a vertex, k and max_dist. Returns k nearest neighbours to the vertex within 
a distance of max_dist to the vertex.
The returned value is a map - With distance to the vertex as the key and the CSpace vertex as value, so its returned in ordered form
The nearest neighbours are returned in order of increasing distance. 
*/
std::map<float,CSpaceVertex> NearestNeighboursPRM(CSpaceVertex v1, int k, float max_dist,  std::map<CSpaceVertex,adjList>& graph) // Key is cost, so its sorted
{

    /* 
    Returns maximum k nearest neighbours
    Breaks as sson as k near neighn=bours are found, does not necessariliy iterate over whole graph
    */ 

    std::map<float,CSpaceVertex> nn;
    float dist; 
    
    for(auto const& x : graph)
    {

        dist = VertexDistance(v1,x.first);
        if(dist < max_dist)
        {
            nn.insert({dist,x.first});

        }
        if(nn.size() > k)
        {
            break;
        }
        
    }

    return nn;        
    

}

/*
Struct for Astar node 
Stores the f,g,h values and the parent of the given CSpace vertex
*/


struct AstarNode
{
  float f;
  float g;
  float h;
  CSpaceVertex parent = {-1,-1,-1,-1,-1};

  AstarNode() : f(0), g(0), h(0) {} ;
  AstarNode(float f_, float g_, float h_,CSpaceVertex v) : f(f_) , g(g_), h(h_),parent(v) {};
};

/* Similar to the Astar used for HW1.
Returns a path (vector of Cspace vertices)
*/

std::vector<CSpaceVertex> Astar(CSpaceVertex start, CSpaceVertex goal, std::map<CSpaceVertex,adjList>& graph) // Input graph too?
{

    std::vector<CSpaceVertex> path;



    std::set<std::pair<float,CSpaceVertex>> open_list ; // lsorted list of pair<fcost,vertex> for open list
    std::map<CSpaceVertex,bool> closed_list ; 
    std::map<CSpaceVertex,AstarNode> nodes; // Vertex with current f , g ,h costs and parent

    open_list.insert(std::make_pair(VertexDistance(start,goal),start));
    nodes.insert({start,AstarNode()});
    // std::cout << "Initializing open list and nodes list " << std::endl;
    // std::cout << "Open list size is (should be one) " << open_list.size() << std::endl;
    // std::cout << "Nodes list size is (should be one) " << nodes.size() << std::endl;
    while(!open_list.empty() && closed_list.find(goal) == closed_list.end())
    {

      auto current_pair = *open_list.begin();
      CSpaceVertex current_vertex = current_pair.second;
      float current_cost = current_pair.first;
      // std::cout << "Node being expanded is (first element in priority queue) ";
      // printCSpaceVertex(current_vertex);

      open_list.erase(open_list.begin()); // Remove first element from priority queue
      closed_list.insert({current_vertex,true}); // Add expanded element to closed list
      // std::cout << "Vertex inserted into close list is ";
      // printCSpaceVertex(current_vertex);
      // std::cout << "Size of closed list is " << closed_list.size()<< std::endl;

      auto it = graph.find(current_vertex);
      auto neighbours = it->second; // Adjacency list containing neighbours
      // std::cout << "Numbers of neighbours to search is " << neighbours.size() << std::endl;

      for(auto const &x : neighbours)
      {

        auto iter = nodes.find(x.second); // find address of neighbour node
        if(!closed_list.empty() && closed_list.find(x.second) != closed_list.end()) // If neighbour is in close list
        {

          // std::cout << "Neighbour in closed list " << std::endl;
          // std::cout <<"*********************************************************" << std::endl;
          continue; // Go to next neighbour 

        } 

        else if(nodes.find(x.second) != nodes.end()) // If neighbour present in nodes list
        {
          // std::cout << "Neighbour is present in Nodes list " << std::endl;
          // std::cout <<"*********************************************************" << std::endl;
          float newg = nodes.find(current_vertex)->second.g + x.first ; // Parent g + edge cost
          float h = VertexDistance(x.second, goal);
          float newf = newg + h;

          if(newf < iter->second.f) // If new f cost is lesser than current f cost, update values and parent
          {
            // std::cout << "Lower cost path found, Update values in lists " << std::endl;
            // std::cout <<"*********************************************************" << std::endl;
            // Update f value in open_list 
            // To do this, we remove the element from set then insert new one with updated f value (set is immutable?)
            auto temp = open_list.find(std::make_pair(iter->second.f,x.second));
            open_list.erase(temp);
            open_list.insert(std::make_pair(newf,x.second));

            // Update values of node in opened_nodes list
            iter->second.g = newg;
            iter->second.f = newf;
            iter->second.parent = current_vertex;
          }
        }

        else
        {
          // Create new node, and add to nodes list  and open_list

          // std::cout << "Neighbour doesnt exist. Creat new node and add to lists " << std::endl;
          float newg = nodes.find(current_vertex)->second.g + x.first ; // Parent g + edge cost g' = g + c(s,s')
          float h = VertexDistance(x.second, goal);
          float newf = newg + h;

          // Add to open list
          open_list.insert(std::make_pair(newf,x.second));

          //Add to nodes list 
          nodes.insert({x.second, AstarNode(newf,newg,h,current_vertex)});



        }
      }

    }

    // Backtracking code to get path

    std::cout << " Start Backtracking to get path " << std::endl;
    if(nodes.find(goal) == nodes.end())
    {
      std::cout << " Goal not present in NODES list " << std::endl;
    }

    CSpaceVertex parent_index = {-1,-1,-1,-1,-1} ;
    // std::cout << "Size of NODES list is " << nodes.size() << std::endl;
    //   std::cout << " Goal vertex (Goal Init) at beginning of backtrack loop is  " ;
    // printCSpaceVertex(goal);
    // std::cout << "Parent of goal is ";
    auto itr = nodes.find(goal);
    CSpaceVertex par = itr->second.parent;
    // printCSpaceVertex(par);
    CSpaceVertex path_vertex = goal;

    while(path_vertex != parent_index)
    {
      // std::cout << "Astar Backtrack loop " << std::endl;
      path.push_back(path_vertex);
      auto it = nodes.find(path_vertex);
      path_vertex = it->second.parent;
      // std::cout << " Parent vertex is ";
      // printCSpaceVertex(path_vertex);
      if(path_vertex == goal)
      {
        std::cout << "Parent of goal is goal. Infinite loop. Break it idiot." << std::endl;
        break;
      }
    }

    // Reverse path to get path from start to goal

    std::reverse(path.begin(),path.end());

    return path;  
}