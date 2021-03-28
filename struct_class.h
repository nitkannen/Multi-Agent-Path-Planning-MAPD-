
#include <bits/stdc++.h>

using namespace std;

#define ll long long int
#define FF first
#define SS second
#define sd(x) scanf("%d", &x)
#define slld(x) scanf("%lld", &x)
#define pd(x) printf("%d\n", x)
#define plld(x) printf("%lld\n", x)
#define mod 1000000007
#define INF INT_MAX
#define eps 0.00001
#define debug(n1) cout << n1 << endl
#define FOR(i, n) for (ll i = 0; i < (n); i++)
#define MOD(a) (((a)>0) ? (a):(-(a)))


///////////////////////////////  require structure,class and global variables ///////////////
// data structure to store a point
struct pos{      
    int x,y;
    pos(int x = 0,int y = 0){
        this->x = x;
        this->y = y;
    }
};
bool operator==(pos &a, pos &b){
    return (a.x==b.x) && (a.y==b.y);
}
// each robot having initial and final position
struct robot{     
    int idx = -1;
    pos initP = pos(0,0);
    pos finalP = pos(0,0);
    pos currP = pos(0,0);
    int task_idx = -1;
    robot(int id = 0,int sx=0, int sy=0, int fx=0, int fy=0){
        idx = id;
        initP = pos(sx,sy);
        finalP = pos(fx,fy);
    }
};
//same as robot contains initial and final position
struct task{
    int idx = -1;
    pos initP = pos(0,0);
    pos finalP = pos(0,0);
    int bot_idx = -1;
    task(int id = 0,int sx=0, int sy=0, int fx=0, int fy=0){
        idx = id;
        initP = pos(sx,sy);
        finalP = pos(fx,fy);
    }
};

//note that idx of robot and task not used anywhere 
//to use it we have to first fill it in readfile() function

struct state{
    vector<pos> botPos;   //contains current position os each bot
    vector<int> task_status; //0:not picked,1:picked,2:delivered
    vector<pos> taskPos;
    vector<int> botTask;   //task assigned currecnt to prticular bot
    state(int bt = 0,int tsk = 0){
        this->botPos.resize(bt);
        this->task_status.resize(tsk);
        this->taskPos.resize(tsk);
        this->botTask.resize(bt,-1);
    }
};

struct path{
  int l,srt,stp;
  vector<int> ar;
};

struct star_node
{
    int id;
    int label; // 1: to pickup,  2:to drop , 0:to final pos
    pos loc; // node {x, y}
    int time; // time synonymous with distance
    int h; // heuristic from Floyd Warshall
    int parentId;

    star_node(int id=-1,int label = 1, pos loc = pos(-1,-1),  int time = 0,int h = INT_MAX,int p_id=-1){
        //printf("id:%d,ln:%d,loc:(%d,%d),h:%d,f:%d\n",id,label,loc.x,loc.y,h,h+time);
        this->id = id;
        this->label = label;
        this->time = time;
        this->loc = loc;
        this->h = h;
        this->parentId = p_id;
        //this->h = multi_label_Heuristic(this->label, this->loc)
    }  
};


//////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////   input & output data structure  ////////////////////

class Dataset
{
public:
   //num of rows,cols,bot,task,storage, obstacles
   int N,M,botN,taskN,storeN,obsN; 
   vector<vector<int>> matrix; //0:normal, 1:storage, 2: obstacles, 1:bot init pos
   vector<robot> botArr;    //robot array
   vector<task> taskArr;    //task array
   vector<vector<int>> minDistTable;
   Dataset(int n,int m,int b,int t,int s,int o){
    this->N = n; this->M = m;
    this->matrix.resize(n,vector<int>(m,0));

    this->botN = b; taskN = t;
    this->botArr.resize(b,robot()) ;
    this->taskArr.resize(t,task());
    this->minDistTable.resize(n*m, vector<int>(n*m,1e6)); //idx=i*m+j(row major)
   }
};

class Solution
{
public:
      //Final solution containing all the states in series.
    vector<state> frames;
    vector<vector<pos>> all_paths;
    int totalTime;
};


  

