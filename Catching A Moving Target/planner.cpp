#include <math.h>
#include "planner.h"
#include <mex.h>
#include <chrono>
#include <thread>
// #include <stack>
#include <stdlib.h>


/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

float getHeuristic(int &x, int &y, int &goalx, int &goaly)
{
    // return sqrt((x - goalx)*(x-goalx) + (y - goaly)*(y-goaly)); // Euclidean
    // Octile/Chebyshev Distance
    int D = 1;
    int D2 = 1;
    int dx = abs(x - goalx);
    int dy = abs(y - goaly);

    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy) ;

}

int global_count = 0; // Define global variable

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    if(curr_time == 0){global_count = 0;}
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int weight = collision_thresh/2; // Weight for weighted A star

    int goalposeX = (int) target_traj[target_steps-1-global_count];
    int goalposeY = (int) target_traj[target_steps-1+target_steps-global_count];


    /* Code to backtrack on target trajectory after reaching its final trajectory pose
    Once the final trajectory position of the robot is reached, WA* is not run at every iteration.
    We just assign the previous position in the trajectory list.

    */
    if(goalposeX == robotposeX && goalposeY == robotposeY)
    {
        global_count+=1;


        int dowx = abs(targetposeX - (int) target_traj[target_steps-1-global_count]);
        int dowy = abs(targetposeY - (int) target_traj[target_steps-1+target_steps-global_count] );
        int sumxy = dowx + dowy ;
        if(sumxy <=2)
        {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            global_count -=1;
            return;
        }
        else
        {
            action_ptr[0] = (int) target_traj[target_steps-1-global_count];//robotposeX;
            action_ptr[1] = (int) target_traj[target_steps-1+target_steps-global_count];//robotposeY;
            return;
        }

    }
    node start;


    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision


    /* MY CODE */

    // Initialize open and closed lists

    std::set<PPI> open_list; // List of nodes in open list in sorted order
    open_list.insert(std::make_pair(0.0,std::make_pair(robotposeX,robotposeY))); // Start node with zero cost
    std::map<PI,node> nodes_created ; // List of nodes initialized
    nodes_created.insert({std::make_pair(robotposeX,robotposeY),start}); // Add start to nodes_created

    bool closed_list[x_size][y_size]; // Whether node has been closed; initialzed as zeros
    
    for(int i = 0;i < x_size; i ++){
        for(int j = 0;j<x_size ; j++){
            closed_list[i][j] = false ;
        }
    }

    while(!open_list.empty() && closed_list[goalposeX][goalposeY] == false)    
    {
        try
        {


            auto current_xy = *open_list.begin(); // Get first element in set

            // Get attributes of current_xy
            int cx = current_xy.second.first;
            int cy = current_xy.second.second;
            float cf = current_xy.first;
            closed_list[cx][cy] = true; // Add above elemnt to closed list

            open_list.erase(open_list.begin()); // Remove first element from open_list


            auto current_node = nodes_created.find(std::make_pair(cx,cy)); // Get iterator to current node 


            for(int i = 0; i < NUMOFDIRS; i++)
            {
                int newX = cx + dX[i];
                int newY = cy + dY[i];
                auto iter = nodes_created.find(std::make_pair(newX,newY));

                if (newX >= 1 && newX <= x_size && newY >= 1 && newY <=y_size) // Check if new x,y values are valid
                {
                    //Check if new x,y is not a obstacle
                    if (((int)map[GETMAPINDEX(newX,newY,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newX,newY,x_size,y_size)] < collision_thresh))
                    {
                        // If new x,y already in closed list, then go on to next neighbour
                        if(closed_list[newX][newY] == true)
                        {
                            continue ; 
                        }

                        // Get iterator if new x,y is found in nodes_created

                        else if (nodes_created.find(std::make_pair(newX,newY)) != nodes_created.end())
                        {
                            float newG = current_node->second._gval + (int)map[GETMAPINDEX(newX,newY,x_size,y_size)]; // g' = g + c(s,s')
                            float h = getHeuristic(newX,newY,goalposeX,goalposeY);
                            float newF = newG + weight*h;

                            float iter_fval = iter->second._fval;
                            
                            if(newF< iter_fval)
                            {
                                // Update f value in open_list 
                                // To do this, we remove the element from set then insert new one with updated f value (set is immutable?)
                                // break;
                                auto temp = open_list.find(std::make_pair(iter->second._fval,std::make_pair(newX,newY)));
                                if(temp!=open_list.end())
                                {
                                    open_list.erase(temp);
                                    open_list.insert(std::make_pair(newF,std::make_pair(newX,newY)));

                                    // Update values of node in nodes_created list
                                    iter->second._gval = newG;
                                    iter->second._fval = newF;
                                    iter->second._parentx = cx;
                                    iter->second._parenty = cy;
                                }


                                


                            }
                        }

                        else
                        {
                            // Create new node and PPI, and add to nodes_created and open_list

                            float newG = current_node->second._gval + (int)map[GETMAPINDEX(newX,newY,x_size,y_size)]; // g' = g + c(s,s')
                            float h = getHeuristic(newX,newY,goalposeX,goalposeY);
                            float newF = newG + weight*h;
                            open_list.insert(std::make_pair(newF,std::make_pair(newX,newY)));
                            nodes_created.insert({std::make_pair(newX,newY), node(cx,cy,newG,newF)});
                            auto temp = nodes_created.find(std::make_pair(newX,newY));

                        }

                    }
                }

                
            }

        }
        catch(const std::out_of_range&)
        {

            break;
        }

    }



    /* Backtracking to find best action. Path is stored in a stack.*/ 



    std::stack<PI> path; // Initialize path stack

    int x_b = goalposeX;
    int y_b = goalposeY;


    while(x_b >=0 && y_b >=0 && (!(x_b == robotposeX && y_b == robotposeY)))
    {
        path.push(std::make_pair(x_b,y_b)); // Push goal to stack


        auto temp1 = nodes_created.find(std::make_pair(x_b,y_b));
        x_b = temp1->second._parentx;
        y_b = temp1->second._parenty;


    }
    
    auto pop_action = path.top();


    robotposeX = pop_action.first;

    robotposeY = pop_action.second;

    if(action_ptr == nullptr)
    {
        action_ptr = new double[2];
    }
    if(action_ptr!=nullptr)
    {
        action_ptr[1] = robotposeY;
        action_ptr[0] = robotposeX;

    }



    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}

// int main(){
//     return 1;
// }