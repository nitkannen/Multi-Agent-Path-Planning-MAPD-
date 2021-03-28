
#include <bits/stdc++.h>
#include "struct_class.h"

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
#define sm(a) cout << #a << ": \n"; for (auto i : a) { for(auto j : i) cout<<j<<' '; cout<<'\n'; }




/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SOME UTILITY FUNCTION /////////////////////////////////////////////
/* To check whether (i, j) lies in the matrix of size n x m */
bool check(int i, int j, int n, int m) {
    if (!(i >= 0 and i < n)) return false;
    if (!(j >= 0 and j < m)) return false;
    return true;
}

int dist(pos s,pos f,Dataset* data) //calculate floydWarshall distance between two points
{
  //return MOD((s.x-f.x)) + MOD((s.y-f.y));
  int srt = s.x * data->M + s.y;
  int dst = f.x * data->M + f.y;
  return data->minDistTable[srt][dst];
}

int h_dist(pos botP,int botIdx, int taskIdx, Dataset* data, int ln = 1) //calculate optimal distance between two points
{ //ln = 1:from current position to pickup, 2:from current to pickup and then to delivery
  //      3: from current to pickup to delivery to final pos no correction
  //     4:curr to pickup to delivery to final position with correction
  //      0: curr pos to final position only
  int d = 0;
    if(ln==0) return dist(botP,data->botArr[botIdx].finalP,data);
    if(ln==2) return dist(data->taskArr[taskIdx].initP,data->taskArr[taskIdx].finalP,data);
    if(ln==1){
      d += dist(botP,data->taskArr[taskIdx].initP,data);
      return d + dist(data->taskArr[taskIdx].initP,data->taskArr[taskIdx].finalP,data);
    }
    if(ln==4) {
      d += -1*dist(botP,data->botArr[botIdx].finalP,data);
      d += dist(botP,data->taskArr[taskIdx].initP,data);
      d += dist(data->taskArr[taskIdx].initP,data->taskArr[taskIdx].finalP,data);
      d += dist(data->taskArr[taskIdx].finalP,data->botArr[botIdx].finalP,data);
    }
    //cout<<"from h_dist value returned:"<<d<<endl;
    return d;

}


/* 2 - blocked, 0 - only one bot allowed, 1 - multiple robots allowed */
/* TODO: add parameters */ 
void floydWarshall(Dataset* data) {
    int n=data->N,m=data->M,x,y;
    for(int i=0; i<n; i++){
        for(int j=0; j<m; j++){
            data->minDistTable[i*m+j][i*m+j] = 0; // distance from u to u (same cell)
            if(data->matrix[i][j]<2){
              x = i-1; y = j;
              if(check(x,y,n,m) and data->matrix[x][y]<2) {
                  data->minDistTable[i*m+j][x*m+y] = 1;
                  data->minDistTable[x*m+y][i*m+j] = 1;
              }
              x = i, y = j-1;
              if(check(x,y,n,m) and data->matrix[x][y]<2) {
                  data->minDistTable[i*m+j][x*m+y] = 1;
                  data->minDistTable[x*m+y][i*m+j] = 1;
              }
            }
        }
    }

    for(int k=0; k<n*m; k++){
        for(int i=0; i<n*m; i++){
            for(int j=0; j<n*m; j++){
                if(data->minDistTable[i][j] > data->minDistTable[i][k] + data->minDistTable[k][j])
                    data->minDistTable[i][j] = data->minDistTable[i][k] + data->minDistTable[k][j];
            }
        }
    }
}



////////////////////////////SCHEDULER APPLIED//////////////////////////////////////////


int DFS(vector<pair<int,int>>curr_sch,vector<pair<int,int>>&final_sch,Dataset* data,vector<int>&botTime,vector<int>botT,vector<pos>botPos,ll* t_max,bool &isGoal){
  if(isGoal) return *t_max;
  vector<bool> task_done(data->taskN,false);
  FOR(i,curr_sch.size()) task_done[curr_sch[i].SS] = true;
  FOR(i,data->taskN) {if(task_done[i]) cout<<"T "; else cout<<"F ";}cout<<endl;
  bool flg = true;
  FOR(i,data->taskN) flg = flg & task_done[i];
  if(flg){
    cout<<"flg is T"<<endl;
    final_sch = curr_sch;
    botTime = botT;
    isGoal = true;
    return (*t_max);
  }
  int next_t = INT_MAX,curr_t;
  FOR(i,data->taskN){
    if(task_done[i]) continue;
    FOR(j,data->botN){
      ll hDist = h_dist(botPos[j],j,i,data,4);
      //cout<<botT[j]+hDist<<" ";
      if(botT[j]+hDist<=(*t_max)){
        pair<int,int> p = {j,i}; //agent task pair
        vector<pair<int,int>> sch_ = curr_sch;
        sch_.push_back(p);
        vector<int> botT_ = botT;
        botT_[j] += hDist;
        vector<pos> botPos_ = botPos;
        botPos_[j] = data->taskArr[i].finalP; //botPos updated after h calculation
        curr_t = DFS(sch_,final_sch,data,botTime,botT_,botPos_,t_max,isGoal); 
      }
      else{
        if(botT[j]+hDist < next_t) next_t  = botT[j]+hDist;
      }
    }
  }
  return next_t;
}

vector<pair<int,int>> scheduler(Dataset* data,vector<int>&botTime,ll* t_max){
  *t_max = 0;
  vector<pair<int,int>> final_sch;
  bool isGoal = false;
  do{
    vector<pair<int,int>> curr_sch;
    vector<int> botT(data->botN,0);
    vector<pos> botPos(data->botN);
    FOR(i,data->botN) {
      botPos[i] = data->botArr[i].initP;
      botT[i] = dist(data->botArr[i].initP,data->botArr[i].finalP,data);
    }
    isGoal = false;
    *t_max = DFS(curr_sch,final_sch,data,botTime,botT,botPos,t_max,isGoal);
    cout<<*t_max<<" "<<endl;
  }while(!isGoal);
  //(*t_max) = time;
  cout<<"Max time taken is:"<<(*t_max)<<endl;
  //cout<<"Time taken by each bot is: ";
  //FOR(i,data->botN) cout<<botT[i]<<" "; cout<<endl;
  return final_sch;
}

//converting final_sch to a favourable form
//[{1,3}, {3, 4}, {1, 2}] -> [{3, 2}, , {4}]
vector<vector<int>> convert_schedule(Dataset* data, vector<pair<int,int>> sch ){
  vector<vector<int>> final_sch(data->botN, vector<int>() );
  for(auto at_pair: sch){
      final_sch[at_pair.FF].push_back(at_pair.SS) ;  //data->botArray[at_pair.FF]
  }
  return final_sch;
}
 



////////////////////A STAR ALGORITHM APPLIED ///////////////////////////////////////////
// define the 4 valid moves
bool isValid(star_node &nd, Dataset* data,vector<vector<vector<bool>>> timeLocOccupied){
  bool flg = true;
  flg = flg & check(nd.loc.x,nd.loc.y,data->N,data->M); //if inside box
  flg = flg & (data->matrix[nd.loc.x][nd.loc.y] != 2);  //if blocked cell
  flg = flg & (!timeLocOccupied[nd.time][nd.loc.x][nd.loc.y]); //if already occupied
  return flg;
}

vector<pos> extract_path(vector<star_node> CLOSED, map<int,int> id2idx,vector<vector<vector<bool>>>& timeLocOccupied,Dataset* data){
  cout<<"extract path is called"<<endl;
  star_node nd = CLOSED.back();
  vector<pos> path;
  while(nd.parentId != -1){
    printf("id:%d,pid:%d,loc:(%d,%d)\n",nd.id,nd.parentId,nd.loc.x,nd.loc.y);
    if(data->matrix[nd.loc.x][nd.loc.y] != 2)
      timeLocOccupied[nd.time][nd.loc.x][nd.loc.y] = true;
    path.insert(path.begin(),nd.loc);
    ll next_idx = id2idx[nd.parentId];
    nd = CLOSED[next_idx];
  }
  path.insert(path.begin(),nd.loc);
  printf("path extracting is done...\n");
  return path;
}
struct node_cmp {
    bool operator()(star_node &a, star_node &b)
    {
        if((a.h + a.time) == (b.h + b.time))
          return a.h > b.h;
        else return (a.h + a.time) > (b.h + b.time); //f = g + h ;compare
    }
};

int dx[5] = {0, 0, 1, 0, -1};
int dy[5] = {0, 1, 0, -1, 0};

vector<pos> A_star(int botIdx, vector<int> t_idx, Dataset* data,vector<vector<vector<bool>>>& timeLocOccupied){

    priority_queue < star_node,vector<star_node>,node_cmp > pq;
    ll id = 0;

    if(t_idx.size()>0)
      pq.push(star_node(0,1, data->botArr[botIdx].initP, 0, h_dist(data->botArr[botIdx].initP,botIdx,t_idx[0],data, 1),-1));
    else
      pq.push(star_node(0,0, data->botArr[botIdx].initP, 0, h_dist(data->botArr[botIdx].initP,botIdx,-1,data, 0),-1));
    id++;
    int time = 0;
    vector<star_node> CLOSED;
    map<int,int> id2idx;
    //cout<<pq.size()<<" ";
    map<int,bool> visited;
    while(!pq.empty()){
        //cout<<"reached";
        star_node cur = pq.top();
        printf("id:%d,ln:%d,loc:(%d,%d)\n",cur.id,cur.label,cur.loc.x,cur.loc.y);
        id2idx[cur.id] = CLOSED.size();
        CLOSED.push_back(cur);
        visited[cur.loc.x*data->M+cur.loc.y] = true;
        pq.pop();

        if(t_idx.size() && cur.label == 1 && (cur.loc == data->taskArr[t_idx[0]].initP)   ){
            pq = priority_queue < star_node,vector<star_node>,node_cmp > ();
            pq.push( star_node(cur.id,2, cur.loc, cur.time, h_dist(cur.loc,botIdx,t_idx[0],data, 2),cur.parentId)) ;
            visited.clear();
            //CLOSED.pop_back();
        }

        else if(t_idx.size() && cur.label == 2 && (cur.loc == data->taskArr[t_idx[0]].finalP)){
            if(t_idx.size()>1){
                t_idx.erase(t_idx.begin());
                pq = priority_queue < star_node,vector<star_node>,node_cmp > ();
                pq.push(star_node(cur.id,1, cur.loc,cur.time,h_dist(cur.loc,botIdx, t_idx[0], data,1),cur.parentId));
                visited.clear();
                //CLOSED.pop_back();
            } 
            else if(t_idx.size()==1){
                pq = priority_queue < star_node,vector<star_node>,node_cmp > ();
                pq.push( star_node(cur.id,0, cur.loc, cur.time,h_dist(cur.loc, botIdx,t_idx[0], data,0),cur.parentId));
                t_idx.erase(t_idx.begin());
                visited.clear();
                //CLOSED.pop_back();
                //t_idx.erase(t_idx.begin());
            }
        }
        else if(cur.label == 0 && (cur.loc== data->botArr[botIdx].finalP ) )
        {
            vector<pos> path = extract_path(CLOSED,id2idx,timeLocOccupied,data);
            return path; //ideally it never comes here
        }
        else{
            
            FOR(i,5){
                pos new_p = pos(cur.loc.x + dx[i], cur.loc.y + dy[i] );
                if(!check(new_p.x,new_p.y, data->N, data->M)) continue;
                int t_id = -1;
                if(t_idx.size())t_id = t_idx[0];
                star_node new_curr = star_node(id,cur.label,new_p, cur.time + 1, h_dist(new_p, botIdx,t_id,data,cur.label),cur.id);
                if(isValid(new_curr,data,timeLocOccupied) && !visited[new_p.x*(data->M)+new_p.y]){
                  new_curr.id = (++id);
                    /*if(new_curr.label==0 && new_curr.loc==data->botArr[botIdx].finalP){
                        vector<pos> path = extract_path(CLOSED,id2idx,timeLocOccupied);
                        return path; //path found and return
                    }*/
                    pq.push(new_curr);
                }

            }
        }
    }

}


////////////////////////////MLA* APPLIED FOR MIN DISTANCE//////////////////////////////////////

Solution *solver(Dataset *data)
{
    Solution *sol = new Solution();
    // call scheduler
    vector<int> botTime;
    ll t_max = 0;
    vector<pair<int,int>> sch =  scheduler(data,botTime,&t_max);
    //t_max = 0;
    FOR(i,data->botN) {
      cout<<botTime[i]<<" "; 
      //if(t_max<botTime[i]) botTime[i] = t_max;
    }
    cout<<endl;
    for(auto p:sch){
      cout<<"("<<p.FF<<","<<p.SS<<") ";
    }
    vector<vector<int>> final_sch = convert_schedule( data,  sch );
    sm(final_sch);
    /*FOR(i,data->botN){
      ll t = 0;
      pos p = data->botArr[i].initP;
      for(auto j:final_sch[i]){
        t += h_dist(p,i,j,data,1);
        p = data->taskArr[j].finalP;
        cout<<p.x<<" "<<p.y<<" - "<<t<<endl;
      botTime[i] = t + h_dist(p,i,-1,data,0);
    }
    FOR(i,data->botN) {
      cout<<botTime[i]<<" "; 
      //if(t_max<botTime[i]) botTime[i] = t_max;
    }*/
    cout<<"Scheduling done..."<<endl;


    vector<vector<pos>> all_paths(data->botN, vector<pos>());
    vector<bool> botUsed(data->botN,false);
    vector<vector<vector<bool>>> timeLocOccupied(t_max,vector<vector<bool>>(data->N,vector<bool>(data->M,false)));
    FOR(i,data->botN){
      int maxTimeIdx = -1;
      int maxTime = 0;
      FOR(j,data->botN){
        if(!botUsed[j] && botTime[j]>maxTime){
          maxTimeIdx = j;
          maxTime = botTime[j];
        }
      }
      botUsed[maxTimeIdx] = true;
      vector<pos> agent_path = A_star(maxTimeIdx , final_sch[maxTimeIdx] , data,timeLocOccupied);
      all_paths[maxTimeIdx] = agent_path;
    }

    /*FOR(i, data->botN){
        vector<pos> agent_path = A_star(i , final_sch[i] , data);
        all_paths[i] = agent_path;
    } */
    for(auto a:all_paths){
      cout<<"bot:";
      for (auto b:a){
        cout<<"->("<<b.x<<","<<b.y<<")";
      }
      cout<<endl;
    }

    vector<pos> botPosLocal(data->botN);//will contain position of bot
    vector<pos> taskPosLocal(data->taskN);//will contain position of task
    vector<int> bot2Task(data->botN,-1);  //bot assigned task idx
    vector<int> bot_status(data->botN,1); //1:to pick, 2:to deliver, 3:to final pos 4:reached
    FOR(i,data->botN) bot2Task[i] = (final_sch[i].size()>0?final_sch[i][0]:-1);
    FOR(i,data->taskN) taskPosLocal[i] = data->taskArr[i].initP;
    FOR(i,data->botN) botPosLocal[i] = data->botArr[i].initP;
    FOR(i,data->botN) bot_status[i] = (final_sch[i].size()>0?1:3);
    sol->all_paths = all_paths;

    ll time = 0;
    //vector<int> botPosIdx(data->botN,0);
    while(1){
      state st(data->botN,data->taskN);
      //vector<vector<pair<bool,int>>> isOccupied(data->N,vector<pair<bool,int>>(data->M,{false,-1}));
      FOR(i,data->botN){
        if(all_paths[i].size()<= time) {
          continue;
        }
        botPosLocal[i] = all_paths[i][time];
        //cout<<all_paths[i][time].x<<" "<<all_paths[i][time].y<<endl;
      }//after for loop all bot is in their next position

      FOR(i,data->botN){
        if(bot_status[i]==2){
          taskPosLocal[bot2Task[i]] = botPosLocal[i];
          st.botTask[i] = bot2Task[i];
        }
        if(bot2Task[i] == -1) continue; //skip if bot has no task assigned
        if(botPosLocal[i] == data->taskArr[bot2Task[i]].initP){
          cout<<"condition satified"<<endl;
          bot_status[i] = 2; //task is with bot
        }
        else if(botPosLocal[i] == data->taskArr[bot2Task[i]].finalP){
          if(final_sch[i].size()<=1){
            bot_status[i] = 3; //delivered and to final
            final_sch[i].erase(final_sch[i].begin());
            bot2Task[i] = -1;
          }
          else{
            bot_status[i] = 1; //delivered and next task
            final_sch[i].erase(final_sch[i].begin());
            bot2Task[i] = final_sch[i][0];
          }
        }
      } 
      for(auto p:bot2Task) cout<<p<<" "; cout<<endl;
      for(auto p:bot_status) cout<<p<<" "; cout<<endl;
      for(auto p:botPosLocal) printf("(%d,%d) ",p.x,p.y); cout<<endl;
      for(auto p:taskPosLocal) printf("(%d,%d) ",p.x,p.y); cout<<endl;
      st.taskPos.assign(taskPosLocal.begin(), taskPosLocal.end());
      //copy(vect1.begin(), vect1.end(), back_inserter(vect2));
      st.botPos.assign(botPosLocal.begin(), botPosLocal.end());
      cout<<"time:"<<time<<" botPosLocal updated correctly...\n";
      sol->frames.push_back(st);
      time++;
      if(time>=t_max) break;
    }
    cout<<"SOLVED..."<<endl;
    sol->totalTime = time;
    printf("sol->frames.size()=%d\n",sol->frames.size());
    return sol;
}




/*
6 10 3 5 2 3    //rows, cols, n_robots, n_tasks, n_storage, n_obstacle
1 1 5 5          // next n_robots line init pos x,y, final pos x ,y
3 4 0 9          
3 3 5 5          
2 4 5 9          // next n_task line init pos x,y and final pos x,y
4 5 0 1
5 8 3 9
3 7 2 8
1 9 4 6
2 7             // temp storage
1 9
4 9             // obstacles
5 7
3 6
*/             

 /*
TO DO:
1) check_invalid(state) or check_goal(state) return 0 or 1
2) state_transitions(state, ID) // states 2^n returns  returns next state
3) how to store DP?
4) Abrar comment the code: Role of each variable, role of each function
 */


  

