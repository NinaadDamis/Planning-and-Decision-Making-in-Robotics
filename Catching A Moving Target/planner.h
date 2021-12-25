#include <iostream>
#include <set>
#include <algorithm>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <stack>

// using namespace std;

typedef std::pair<int,int> PI; // Stores x,y coordinates
typedef std::pair<float,std::pair<int,int>> PPI; // Stores x,y coordinate and f value

struct node{

    int _parentx;
    int _parenty;
    float _gval;
    float _fval;
    // bool _isClosed;
    node() : _parentx(-1) , _parenty(-1), _gval(0.0), _fval(0.0) {};
    node(int parentx,int parenty,float gval,float fval) : _parentx(parentx) , _parenty(parenty), _gval(gval), _fval(fval) {};

};

// extern int global_count;
// extern std::stack<PI>path;