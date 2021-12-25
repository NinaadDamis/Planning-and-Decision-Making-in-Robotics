/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include "prm.h"
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

# define M_PI 3.14159


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}

/******** MY CODE ****************/

typedef std::vector<double> CSpaceVertex ; // Storing n DOF dimension points of C free

struct RRTNode
{

  int index;
  int parent_index;
  double cost;
  CSpaceVertex vertex;

};

typedef std::vector<RRTNode*> Tree ;


/****************************************************************************/

/*

GENERAL UTILITY FUNCTIONS USED BY MULTIPLE PLANNERS

*/

void PrintVertex(CSpaceVertex q)
{
  std::cout << "Vertex is " ;

  for(int i = 0; i < q.size(); i++)
  {
    std::cout << q[i] << ",";

  }
  std::cout <<std::endl;
}


/*

sampleRandomVertex() : This function is used to general random samples uniformly from the configuration space

*/

CSpaceVertex sampleRandomVertex(int numDof, double*	map, int x_size, int y_size)
{
    
    while(1)
    {
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        // std::nextafter(2.0*M_PI, std::numeric_limits<RealType>::max()
        std::uniform_real_distribution<> dis(0,2.0*M_PI ); 
        CSpaceVertex sampleVertex ;
        for (int n = 0; n < numDof; n++) 
        {
            // Use dis to transform the random unsigned int generated by gen into a 
            // double in [1, 2). Each call to dis(gen) generates a new random double

            const double a = dis(gen);
            sampleVertex.push_back(a);
            
        }
        double arr[12]; // Change - 
        // COnverting vector into array to input to isValidArm
        for(int i = 0; i<numDof; i++)
        {

          arr[i] = sampleVertex[i];

        }
        if (IsValidArmConfiguration(arr,numDof,map, x_size,y_size))
        {

            return sampleVertex;
        }
        else
        {
          //std::cout << "Sample generated is not valid. Generate new sample" << std::endl;
        }
    }

}


/*

RRTNearestNeighbour() : This function does a linear O(n) search through the tree, and finds the vertex in the tree
                        with the minimum distance to q_rand. Returns the index of the nearest neighbour in th tree.

*/


int RRTNearestNeighbour(Tree &tree, CSpaceVertex q_rand) //, int numofDOFs, double*	map,int x_size, int y_size
{

  int size = tree.size();
  double min_dist = 1000; // Initialize very high value
  double dist;
  int nn_index; // Index of nearest neighbour
  for(int i = 0; i < size; i++)
  {
    dist = VertexDistance(q_rand,tree[i]->vertex);
    if(dist<min_dist)
    {
      min_dist = dist;
      nn_index = i;

    }
  }

  return nn_index;

}


/*

LocalPlanner() - GIven two vertices v1 and v2, this function outputs whether there exists an obstacle free path from v1 to v2.
                 This is achieved by using interpolation. Using input parameter num_steps, we decide the number of intermediate
                 configurations, and we check if each of those configurations is Valid. 

*/

bool LocalPlanner(int numofDOFs, double*	map, int x_size, int y_size, CSpaceVertex v1, CSpaceVertex v2 , int num_steps)
{
    // From v1 to v2
    int size = v1.size();
    CSpaceVertex increment;
    for(int i = 0; i < size; i++)
    {
        increment.push_back((v2[i] - v1[i]) / num_steps);
    }

    for(int i = 0; i < num_steps; i++)
    {
        // CSpaceVertex temp;
        for(int j = 0; j < size ; j++)
        {
            v1[j] = v1[j] + increment[j];
        }
        //Making array from vector
        double arr[12]; // Change
        for(int f = 0; f<numofDOFs ; f++)
        {
          arr[f] = v1[f];
        }
        if(IsValidArmConfiguration(arr,numofDOFs,map,x_size,y_size)) continue;
        else return false;
    }

    return true;
}


/*

AltRRTGenerateNewVertex() : This function basically is the Extend function for the RRT variants.
                            Given q_rand and the nearest neighbour in the tree, this function tries to generate q_new 
                            towards q_rand such that it is within distance epsilon ( max_step parameter).
                            An empty vector CSPaceVertex is returned if no q_new is abled to be generated.

*/

CSpaceVertex AltRRTGenerateNewVertex(Tree &tree,CSpaceVertex q_rand, int q_near_index, double max_step, int numofDOFs, double*	map,
                              int x_size, int y_size, int num_steps)
{
  
  CSpaceVertex q_new; // Initialize q_new
 

  double dist = VertexDistance(q_rand,tree[q_near_index]->vertex);

  if(dist<max_step)
  {
    return q_rand; // The random vertex is itself the new vertex
  }



  else
  {
    double resolution = 0.01745;
    dist = 0;
    
    CSpaceVertex q_near = tree[q_near_index]->vertex;
    CSpaceVertex q_new_int = q_near;
    CSpaceVertex increment;

    for(int i = 0; i < q_near.size(); i++)
    {
        if(q_rand[i]>q_near[i]) increment.push_back(1);
        else increment.push_back(-1);
    }

    while(dist<max_step)
    {

      for(int i = 0; i < q_rand.size();i++) // Increase each joint config linearly by value ofresolution
      {
        if(increment[i] == 1)
        {
          q_new_int[i] = q_new_int[i] + increment[i]*resolution;
          if(q_new_int[i]>q_rand[i]) q_new_int[i] = q_rand[i];
        }

        if(increment[i] == -1)
        {
          q_new_int[i] = q_new_int[i] + increment[i]*resolution;
          if(q_new_int[i]<q_rand[i]) q_new_int[i] = q_rand[i];
        }
      }

      double arr[12]; // Change

      for(int i = 0; i < q_rand.size();i++) 
      {
        arr[i] = q_new_int[i];
      }

      if(IsValidArmConfiguration(arr,numofDOFs,map,x_size,y_size) && VertexDistance(q_near,q_new_int) < max_step) 
      {
        //std::cout << "Potential new node is valid and within max_step" << std::endl;
        if(LocalPlanner(numofDOFs,map,x_size,y_size,q_near,q_new_int ,num_steps))
        {
          //std::cout << "Local Planner can find valid path " << std::endl;
          dist = VertexDistance(q_near,q_new_int);
          q_new = q_new_int;
        }

        else break;


      }
      else break;


    }

  }

  return q_new; // If q_new returned is empty, then no new node was created, generate random node again 

}


/*

RRTAddVertex : Adds q_new to the tree with its parent and cost.

*/
void RRTAddVertex(Tree &tree, CSpaceVertex q_new, int nn_index, int& count)
{

  RRTNode* temp = new RRTNode;
  temp->index = count ;
  temp->parent_index = nn_index;
  temp->vertex = q_new;
  temp->cost = tree[nn_index]->cost + VertexDistance(q_new,tree[nn_index]->vertex); // Is it correct?

  tree.push_back(temp);
  count++;



}

/***********************************************************************************/



/* RRT-CONNECT --> CONNECT FUNCTION */

std::vector<int> RRTConnectExtend(Tree &tree,CSpaceVertex q,int q_near_index, double max_step, int numofDOFs, double*	map,
                              int x_size, int y_size, int num_steps, int& count)
{

  int trapped = 0;
  int state = 0;

  CSpaceVertex q_connect;

  while(!trapped)
  {
    q_connect = AltRRTGenerateNewVertex(tree,q,q_near_index, max_step, numofDOFs,	map,x_size,y_size,num_steps);
    if(q_connect.empty()) 
    { state = 1; // Trapped
      break; // No new vertex was able to be generated
    }
    else if(q_connect == q)
    {
      state = 2; // Reached
      break; 
    }
    else
    {
      RRTAddVertex(tree,q_connect,q_near_index,count);
      q_near_index = count -1;
    }

  }


  std::vector<int>state_vector = {state,q_near_index};

  return state_vector; // If q_new returned is empty, then no new node was created, generate random node again 


}

/**********************************************************************************************************/


/* ********************************** RRT STAR SPECIFIC FUNCTIONS *********************************** */


/* 

RRTStarNear() - This function takes a tree, a vertex q_new , and a distance parameter r as input.
                   It returns as output the set of vertices in the tree within distance 'r' from q_new.  

*/


std::vector<int> RRTStarNear(Tree &tree,CSpaceVertex q_new, double r)
{
  double dist ;
  std::vector<int> near_set;
  for(int i = 0; i < tree.size();i++)
  {
    auto current_node = tree[i]; // Pointer to node
    dist = VertexDistance(q_new,current_node->vertex);
    if(dist<r)
    {
      near_set.push_back(i) ; // Add index of the the node to the set of near neighbopurs
    }
  }
  return near_set;
}



/*

RRTStarAddQNew() - This function adds the vertex q_new to the tree such that the cost of q_new is minimum.
                   It does this by iterating through the set near_set (set of vertices in tree within a given distance to q_new)
                   and connecting to the vertex which produces the least cost for q_new.

*/


void RRTStarAddQNew(Tree &tree, CSpaceVertex q_new, int q_nearest_index, std::vector<int> near_set,int numofDOFs,
                   double *map, int x_size, int y_size, int num_steps, int &count )
{
  // Initialize xmin and cmin
  
  int xmin = q_nearest_index; // Index of xmin vertex
  double cmin; // Min cost of path from xmin->xnew
  cmin = tree[q_nearest_index]->cost + VertexDistance(q_new,tree[q_nearest_index]->vertex); // Cost + c(q_new,q_nearest)

  for(auto x : near_set)
  {
    auto q_near = tree[x]->vertex;
    auto cost = tree[x]->cost + VertexDistance(q_new,q_near);
    if(LocalPlanner(numofDOFs,map,x_size,y_size,q_new,q_near,num_steps) && cost < cmin )
    {
      xmin = x;
      cmin = cost;
    }
  }
  RRTAddVertex(tree,q_new,xmin,count);

  
}


/*

RRTStarRewireTree() - This function is used to rewire the edges in the tree.
                      New edges are created from x_new to the vertices in X near(the near_set) if the path through x_new has lesser
                      cost than the path through the current parent.

*/

void RRTSTarRewireTree(Tree &tree, int q_new_index,std::vector<int> near_set,int numofDOFs,
                        double *map, int x_size, int y_size, int num_steps)

{
  CSpaceVertex q_new = tree[q_new_index]->vertex;
  double cost_q_new = tree[q_new_index]->cost;
  for(auto x : near_set)
  {
    
    CSpaceVertex q_near = tree[x]->vertex;
    double cost_q_near = tree[x]->cost;

    if(LocalPlanner(numofDOFs,map,x_size,y_size,q_new,q_near,num_steps) && cost_q_new + VertexDistance(q_new,q_near) < cost_q_near)
    {
      // Rewire edges, ie, change parent of q_near to q_new
      tree[x]->parent_index = q_new_index;
      tree[x]->cost = cost_q_new + VertexDistance(q_new,q_near);
    }

  }

}

/**************************************************************************************************/





/*********** PRM FUNCTIONS ****************/


std::map<CSpaceVertex,adjList> ConstructRoadmapPRM(int dof, double*	map,
		                        int x_size, int y_size,int max_num_vertex,int k,int max_distance,int local_planner_steps)
{

  std::map<CSpaceVertex,adjList> graphVE ; // Initialize empty graph

  int i = 0;
  while(i < max_num_vertex)
  {
    CSpaceVertex q = sampleRandomVertex(dof,map,x_size,y_size); // Generate random sample in C free
    adjList temp;
    auto nn = NearestNeighboursPRM(q,k,max_distance,graphVE); // Get nearest Neighbours
    graphVE.insert({q,temp}); // Insert vertex into graph with empty adjacency list
    for(auto const&x : nn)
    {
      if(!ConnectedComponents(q,x.second,graphVE) && LocalPlanner(dof,map,x_size,y_size,q,x.second,local_planner_steps))
      {

        addEdgetoPRM(q,x.second,graphVE);

      }
    }

  i++;

  }


  return graphVE;

}

std::map<CSpaceVertex,adjList> graphVE; // Global variable declared for PRM


/*******************************************************************************/





static void plannerPRM(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{

  auto start = std::chrono::high_resolution_clock::now();
	//no plan by default
	*plan = NULL;
	*planlength = 0;

  CSpaceVertex start_pos;
  CSpaceVertex goal_pos;

  CSpaceVertex start_init;
  CSpaceVertex goal_init;

  std::vector<CSpaceVertex> path;
  int n = 3000; // Number of vertices for roadmap
  int k = 40 ; // Initialize k number of nearest neighbours
  float max_dist = 3; // ?? Define max distance for nearest neighbours
  int num_steps = 100; // Number of steps for interpolation for local planner

  int query_success = 1;
  int start_init_success = 0;
  int goal_init_success = 0;

  for(int i = 0; i < numofDOFs; i++)
  {
    start_pos.push_back(armstart_anglesV_rad[i]);
  }


  

  for(int i = 0; i < numofDOFs; i++)
  {
    goal_pos.push_back(armgoal_anglesV_rad[i]);
  }



  if(graphVE.empty()) 
  {
    graphVE = ConstructRoadmapPRM(numofDOFs,map,x_size,y_size,n,k,max_dist,num_steps);
    std::cout <<"Roadmap graph was empty. Running first time " << std::endl;
  }

  else 
  {
      if(graphVE.begin()->first.size() != numofDOFs)
      {
      graphVE = ConstructRoadmapPRM(numofDOFs,map,x_size,y_size,n,k,max_dist,num_steps);
      std::cout <<"existing roadmap was for different arm.Constructing new map" << std::endl;
      }

        else
  {
    std::cout<< "Roadmap graph is not empty. Only query phase required " << std::endl;
  }

  }


  std::cout << "Size of Roadmap graph is " << graphVE.size() << std::endl;

  /* 

  Connecting start and goal positions to the roadmap.
  As trying to connect the start or goal to only its nearest neighbour may fail
  I have taken a set of 100 nearest neighbours, and try to connect to them until 
  we are able to connect.

  The vertices we are able to connect to are called start_init and goal_init

  */

  auto start_init_nn = NearestNeighboursPRM(start_pos,100,max_dist,graphVE);
  for(auto const &x : start_init_nn)
  {
    CSpaceVertex temp = x.second;
    if(LocalPlanner(numofDOFs,map,x_size,y_size,start_pos,temp,num_steps))
    {
      std::cout << " Successfully connected start position to roadmap " << std::endl;
      start_init = temp;
      start_init_success = 1;
      break;
    }
    else 
    {
      std::cout << " Coudnt connect start position to roadmap " << std::endl;
      // query_success = 0;
    }
    
  }

  std::cout << " *********************************************************************************************** " << std::endl;
  auto goal_init_nn = NearestNeighboursPRM(goal_pos,100,max_dist,graphVE);
  for(auto const &x : goal_init_nn)
  {
    CSpaceVertex temp = x.second;
    if(LocalPlanner(numofDOFs,map,x_size,y_size,goal_pos,temp,num_steps))
    {
      std::cout << " Successfully connected goal position to roadmap " << std::endl;
      goal_init = temp;
      goal_init_success = 1;
      break;
    }
    else
    {
      std::cout << " Coudnt connect goal position to roadmap " << std::endl;
      // query_success = 0;
    }
     
  }

  std::cout << "***************************************************************************************************" << std::endl;
  std::cout << "Check if start init and goal init are connected - " ;
  std::cout << ConnectedComponents(start_init,goal_init,graphVE) << std::endl;


  /*

  We check if vertices start_init and goal_init are connected.
  If they aren't connected, we are not able to find a path

  */

  if(!ConnectedComponents(start_init,goal_init,graphVE))
  {
    std::cout << "Start init and goal init are not connected " << std::endl;
    query_success = 0;
  }


  std::cout << "Calling ASTAR Function" << std::endl;
  auto a_star_start = std::chrono::high_resolution_clock::now();

  /*

  If the query phase is successful we call the A* search algorithm to find a path in the graph.

  */


  if(query_success)
  {
    auto astar_path = Astar(start_init,goal_init,graphVE);

    path.push_back(start_pos);
    path.insert(std::end(path),std::begin(astar_path),std::end(astar_path));
    path.push_back(goal_pos);
  }

  else
  {
    std::cout << "No path found !!!!!!" << std::endl;
  }
  auto a_star_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> a_star_elapsed_seconds = a_star_end - a_star_start;
  // std::cout << "Time taken for A star is " << a_star_elapsed_seconds.count() << std::endl;

  int numofsamples = path.size();
  if(numofsamples < 2){
      printf("the arm is already at the goal\n");
      return;
  }
  *plan = (double**) malloc(numofsamples*sizeof(double*));
  int firstinvalidconf = 1;
  for (int i = 0; i < numofsamples; i++){
      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
      // for(j = 0; j < numofDOFs; j++){
      //     (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
      }

  for(int j =0; j < numofsamples; j++)
  {
    for(int i = 0; i < numofDOFs; i ++)
    {
      (*plan)[j][i] = path[j][i];
    }
  }

  *planlength = numofsamples;

  std::cout << "PLan length is " << *planlength << std::endl;

  /*
  
   Calculate Cost of the path produced by PRM

  */

  double path_cost = 0;
  for(int i = 0 ; i < path.size() -1 ; i++)
  {
    path_cost += VertexDistance(path[i],path[i+1]);
  }
  std::cout <<"Cost of path is " << path_cost <<std::endl;

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "Time taken for planning is " << elapsed_seconds.count() << std::endl;  

  return;
}



static void plannerRRTStar(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default

  auto start = std::chrono::high_resolution_clock::now();
	*plan = NULL;
	*planlength = 0;


    Tree tree; // Initialize tree
    int count = 0; // Index in the tree

    int max_iter = 80000 ; // Number of itrations to run
    double max_step = 0.2; // Define epsilon for max_step distance
    double r = 0.3; // Distance Parameter for creating set of near neighbours
    int num_steps = 20; // Define num_steps for local planner
    int goal_index;
    int goal_found = 0;
    int iters_after_first_sol = 0;
    std::vector<CSpaceVertex> path;

    CSpaceVertex start_pos;
    CSpaceVertex goal_pos;
    CSpaceVertex q_rand;
    CSpaceVertex q_near;
    CSpaceVertex q_new;
    int nn_index; // Nearest neighbour index in tree
    int q_new_index;


    for(int i = 0; i < numofDOFs; i++)
    {
      start_pos.push_back(armstart_anglesV_rad[i]);
    }


    

    for(int i = 0; i < numofDOFs; i++)
    {
      goal_pos.push_back(armgoal_anglesV_rad[i]);
    }

     RRTNode* start_node = new  RRTNode;

    start_node->index = count;
    start_node->parent_index = -1; // -1 as it is root of the tree
    start_node->vertex = start_pos;
    start_node->cost = 0; // Sourc vertex has zero cost

    tree.push_back(start_node); // Add root node to tree
    count++;


    /*MAIN FOR LOOP - RUNNING FOR MAX_ITER ITERATIONS */

    for(int i = 0; i < max_iter; i++)
    {

      /* Once the initial solution is found, we may not want to run the algorithm 
       till max_iter = 80,000 , as it would take too much time (> 10 minutes ).
       Thus, once the initial solution is found, we run only 5000 more iterations before
       breaking from the loop.
       We can increase this number to obtain paths with better cost, but at the expense of the algorithm
       running longer. 
       */

      if(iters_after_first_sol > 5000) break;
      if(goal_found ==1) iters_after_first_sol+=1;

      /* Generate Random Configuration */

      // Once goal is found, we want to stop goal biasing and focus on just rewiring the tree. Hence we use goal_found.
      if(goal_found ==0 )
      {
        if(i%20 ==0 && i !=0)
        {
          q_rand = goal_pos; // Goal Bias - P = 0.05 --> Selects goal once every 20 times (as suggested in class by Max)
        }
        else
        {
          q_rand = sampleRandomVertex(numofDOFs,map,x_size,y_size);
        }

      }
      else
      {
        //std::cout << "Goal found ->So goal biasing stopped" << std::endl;
        q_rand = sampleRandomVertex(numofDOFs,map,x_size,y_size);
      }

      if(i%10000 ==0 && i !=0) 
      {
        
        std::cout << "Number of iterations is " << i << std::endl;
        std::cout <<"Tree size is " << tree.size() << std::endl;
      }
      /* Get nearest neighbour */

      nn_index = RRTNearestNeighbour(tree,q_rand);
      q_new = AltRRTGenerateNewVertex(tree,q_rand,nn_index,max_step,numofDOFs,map,x_size,y_size, num_steps);
      q_new_index = count;
      if(q_new.empty())
      {
        //std::cout << "New vertex wasnt able to be Generated. Break" << std::endl;
        continue; // Go to next iterations
      }
      

      if(q_new == goal_pos)
      {
        std::cout << "Goal reached " << std::endl;
        std::cout << "***************************************************************************************** " << std::endl;
        std::cout << "Number of iterations is " << i << std::endl;
        goal_index = count;
        goal_found = 1;
        auto first_sol = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> first_sol_elapsed_seconds = first_sol - start;
        std::cout << "Time taken for finding first solution is " << first_sol_elapsed_seconds.count() << std::endl;
        // break; // Break outta for loopx
      }

      std::vector<int> near_set = RRTStarNear(tree,q_new,r);
      RRTStarAddQNew(tree,q_new,nn_index,near_set,numofDOFs,map,x_size,y_size,num_steps,count);
      RRTSTarRewireTree(tree,q_new_index,near_set,numofDOFs,map,x_size,y_size,num_steps);

    }
    /* BAcktracking to get path */

     std::cout << "Final size of tree is " << tree.size() << std::endl;

    if(goal_found == 1)
    {
    std::cout << "Starting Backtracking ***********" << std::endl;
    int parent_index= goal_index;
    while(parent_index!=-1)
    {

      CSpaceVertex temp = tree[parent_index]->vertex;
      path.push_back(temp);
      parent_index = tree[parent_index]->parent_index;
    }
    }

    int numofsamples = path.size();
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (int i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        
       
        }

    for(int j =0; j < numofsamples; j++)
    {
      for(int i = 0; i < numofDOFs; i ++)
      {
        (*plan)[j][i] = path[numofsamples - 1 - j][i];
      }
    }

    *planlength = numofsamples;

    std::cout << "PLan length is " << *planlength << std::endl;

    std::cout << "Cost of Goal Vertex is " << tree[goal_index]->cost << std::endl;


    //std::cout << "Deleting Trees" << std::endl;

    for(int i = 0; i < tree.size();i++)
    {

      delete tree[i];
      tree[i] = nullptr;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Time taken for planning is " << elapsed_seconds.count() << std::endl;


    
    return;
}


static void plannerRRTConnect(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default

  auto start = std::chrono::high_resolution_clock::now();
	*plan = NULL;
	*planlength = 0;


    Tree tree1; // Initialize tree1
    Tree tree2; // Initialize tree1

    int count1 = 0; // Index in the tree
    int count2 = 0;

    int max_iter = 80000 ; // Number of itrations to run
    double max_step = 0.2; // Define epsilon for max_step distance
    int num_steps = 20; // Define num_steps for local planner
    int goal_index;
    int goal_reached = 0;
    int tree1_connect_index;
    int tree2_connect_index;
    // int state;
    std::vector<CSpaceVertex> path;
    std::vector<CSpaceVertex> path1;
    std::vector<CSpaceVertex> path2;

    CSpaceVertex start_pos;
    CSpaceVertex goal_pos;
    CSpaceVertex q_rand;
    CSpaceVertex q_near;
    CSpaceVertex q_new;
    CSpaceVertex q_connect;
    CSpaceVertex placeholder = {-1,-1,-1,-1,-1};
    int nn_index; // Nearest neighbour index in tree
    int nn_index_extend; // Nearest neighbour in other tree 


    for(int i = 0; i < numofDOFs; i++)
    {
      start_pos.push_back(armstart_anglesV_rad[i]);
    }


    

    for(int i = 0; i < numofDOFs; i++)
    {
      goal_pos.push_back(armgoal_anglesV_rad[i]);
    }

  /*

    Initializing start and goal node and adding them to tree1 and tree2

  */

    RRTNode* start_node = new RRTNode;

    start_node->index = count1;
    start_node->parent_index = -1; // -1 as it is root of the tree
    start_node->vertex = start_pos;
    start_node->cost = 0;

    tree1.push_back(start_node); // Add root node to tree
    count1++;


    RRTNode* goal_node = new RRTNode;

    goal_node->index = count2;
    goal_node->parent_index = -1; // -1 as it is root of the tree
    goal_node->vertex = goal_pos;
    goal_node->cost = 0;

    tree2.push_back(goal_node); // Add root node to tree
    count2++;


    // We use start_tree variable to basically implement the swapping of the trees.

    bool start_tree = true;

    /*MAIN FOR LOOP - RUNNING FOR MAX_ITER ITERATIONS */

    for(int i = 0; i < max_iter; i++)
    {

      if(start_tree)
      {

        if(i %10000 == 0 && i !=0) std::cout << "Number of iterations is " << i << std::endl;

        /* Generate Random Configuration */

        // Sample random vertex - No goal biasing in RRT connect

        q_rand = sampleRandomVertex(numofDOFs,map,x_size,y_size);


        /* Get nearest neighbour */

        nn_index = RRTNearestNeighbour(tree1,q_rand);
        q_new = AltRRTGenerateNewVertex(tree1,q_rand,nn_index,max_step,numofDOFs,map,x_size,y_size, num_steps);
        if(q_new.empty())
        {
          //std::cout << "New vertex wasnt able to be Generated. Break" << std::endl;
          continue; // Go to next iterations
        }
        
        RRTAddVertex(tree1, q_new, nn_index,count1);

        // Code for extend operation
        nn_index_extend = RRTNearestNeighbour(tree2,q_new); // Nearest neighbour with new node generated above
        std::vector<int> state = RRTConnectExtend(tree2,q_new,nn_index_extend,max_step,numofDOFs,map,x_size,y_size,num_steps,count2);
        if(state[0] ==2) 
        { 
          std::cout << "Trees were connected " << std::endl;
          tree1_connect_index = count1 - 1;
          tree2_connect_index = state[1];
          goal_reached = 1;
          std::cout << "Total vertices in both trees combined is " << tree1.size() + tree2.size() << std::endl;
          break;
        }

        start_tree = !start_tree;
      }

      else
      {

        /* Generate Random Configuration */

        // Sample random vertex - No goal biasing in RRT connect

        q_rand = sampleRandomVertex(numofDOFs,map,x_size,y_size);


        /* Get nearest neighbour */

        nn_index = RRTNearestNeighbour(tree2,q_rand);
        q_new = AltRRTGenerateNewVertex(tree2,q_rand,nn_index,max_step,numofDOFs,map,x_size,y_size, num_steps);
        if(q_new.empty())
        {
          //std::cout << "New vertex wasnt able to be Generated. Break" << std::endl;
          continue; // Go to next iterations
        }
        
        RRTAddVertex(tree2, q_new, nn_index,count2);

        // Code for extend operation

        nn_index_extend = RRTNearestNeighbour(tree1,q_new); // Nearest neighbour with new node generated above

        std::vector<int> state = RRTConnectExtend(tree1,q_new,nn_index_extend,max_step,numofDOFs,map,x_size,y_size,num_steps,count1);
        if(state[0] ==2) 
        { 
          std::cout << "Trees were connected " << std::endl;
          tree1_connect_index =state[1];
          tree2_connect_index = count2-1 ;
          goal_reached = 1;
          std::cout << "Total vertices in both trees combined is " << tree1.size() + tree2.size() << std::endl;
          break;
        }

        start_tree = !start_tree;

      }

    }
    /* BAcktracking to get path */

    if(goal_reached ==1 )
    {
    int parent_index1= tree1_connect_index;
    while(parent_index1!=-1)
    {

      CSpaceVertex temp = tree1[parent_index1]->vertex;
      path1.push_back(temp);
      parent_index1 = tree1[parent_index1]->parent_index;
    }
    int parent_index2= tree2_connect_index;
    while(parent_index2!=-1)
    {

      CSpaceVertex temp = tree2[parent_index2]->vertex;
      path2.push_back(temp);
      parent_index2 = tree2[parent_index2]->parent_index;
    }


    // Creating the final path by reversing path1 and appending path 2.

    std::reverse(path1.begin(),path1.end());
    path1.insert(std::end(path1), std::begin(path2), std::end(path2));
    path = path1;

    }

    else{ std::cout << "PATH WASNT FOUND !!!!!!!!!!!!!!!!!!!" << std::endl;}

    int numofsamples = path.size();
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (int i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        
        }

    for(int j =0; j < numofsamples; j++)
    {
      for(int i = 0; i < numofDOFs; i ++)
      {
        (*plan)[j][i] = path[j][i];
      }
    }

    *planlength = numofsamples;


    double total_cost = tree1[tree1_connect_index]->cost + tree2[tree2_connect_index]->cost 
                        + VertexDistance(tree1[tree1_connect_index]->vertex, tree2[tree2_connect_index]->vertex); 

    std::cout << "Cost of path is " << total_cost <<std::endl;

    //std::cout << "Deleting Trees" << std::endl;

    for(int i = 0; i < tree1.size();i++)
    {

      delete tree1[i];
      tree1[i] = nullptr;
    }

    for(int i = 0; i < tree2.size();i++)
    {

      delete tree2[i];
      tree2[i] = nullptr;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Time taken for planning is " << elapsed_seconds.count() << std::endl;


    
    return;
}

static void plannerRRT(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default

  auto start = std::chrono::high_resolution_clock::now();
	*plan = NULL;
	*planlength = 0;


    Tree tree; // Initialize tree
    int count = 0; // Index in the tree




    int max_iter = 80000 ; // Number of itrations to run
    std::cout << " Initializing tree. Running for max_iter =  " << max_iter  << std::endl;

    double max_step = 0.2; // Define epsilon for max_step distance
    int num_steps = 20; // Define num_steps for local planner
    int goal_index = 0;
    std::vector<CSpaceVertex> path;

    CSpaceVertex start_pos;
    CSpaceVertex goal_pos;
    CSpaceVertex q_rand;
    CSpaceVertex q_near;
    CSpaceVertex q_new;
    int nn_index; // Nearest neighbour index in tree


    for(int i = 0; i < numofDOFs; i++)
    {
      start_pos.push_back(armstart_anglesV_rad[i]);
    }


    

    for(int i = 0; i < numofDOFs; i++)
    {
      goal_pos.push_back(armgoal_anglesV_rad[i]);
    }

    RRTNode* start_node = new RRTNode;

    start_node->index = count;
    start_node->parent_index = -1; // -1 as it is root of the tree
    start_node->vertex = start_pos;
    start_node->cost = 0;

    tree.push_back(start_node); // Add root node to tree
    count++;

    /*MAIN FOR LOOP - RUNNING FOR MAX_ITER ITERATIONS */

    for(int i = 0; i < max_iter; i++)
    {

      /* Generate Random Configuration */
      if(i%10000 == 0 && i != 0) 
      {
        std::cout << "Number of iterations is " << i << std::endl;
        std::cout <<"Size of tree is " << tree.size() << std::endl;
      }

      if(i%20 ==0 && i !=0)
      {
        q_rand = goal_pos; // Goal Bias - P = 0.05 --> Selects goal once every 20 times ( as suggested by Max)
      }
      else
      {
        q_rand = sampleRandomVertex(numofDOFs,map,x_size,y_size);
      }


      /* Get nearest neighbour */

      nn_index = RRTNearestNeighbour(tree,q_rand);
      q_new = AltRRTGenerateNewVertex(tree,q_rand,nn_index,max_step,numofDOFs,map,x_size,y_size, num_steps);
      if(q_new.empty())
      {
        //std::cout << "New vertex wasnt able to be Generated. Break" << std::endl;
        continue; // Go to next iterations
      }
      
      RRTAddVertex(tree, q_new, nn_index,count);

      if(q_new == goal_pos)
      {
        goal_index = count - 1;
        break; // Break outta for loopx
      }

      
    }
    /* BAcktracking to get path */

     std::cout << "Final size of tree is " << tree.size() << std::endl;


    std::cout << "Starting Backtracking ***********" << std::endl;
    if(goal_index == 0) std::cout << "Path wasnt found !!!!!!!!!!!!!!!!!" << std::endl;
    int parent_index= goal_index;
    while(parent_index!=-1)
    {
      CSpaceVertex temp = tree[parent_index]->vertex;
      path.push_back(temp);
      parent_index = tree[parent_index]->parent_index;
    }


    int numofsamples = path.size();
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (int i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        }

    for(int j =0; j < numofsamples; j++)
    {
      for(int i = 0; i < numofDOFs; i ++)
      {
        (*plan)[j][i] = path[numofsamples - 1 - j][i];
      }
    }

    *planlength = numofsamples;

    std::cout << "PLan length is " << *planlength << std::endl;

    // Delete dynamic memory allocated to tree
    std::cout << "Cost of Goal Vertex is " << tree[goal_index]->cost << std::endl;

    for(int i = 0; i < tree.size();i++)
    {

      delete tree[i];
      tree[i] = nullptr;
    }

    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Time taken for planning is " << elapsed_seconds.count() << std::endl;


    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    if (planner_id == RRT)
    {
       plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    if (planner_id == RRTSTAR)
    {
       plannerRRTStar(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }    
    
    if (planner_id == RRTCONNECT)
    {
       plannerRRTConnect(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }

    if (planner_id == PRM)
    {
       plannerPRM(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    
    //dummy planner which only computes interpolated path
    // plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





