#include <math.h>
#include <bits/stdc++.h>
#include <iostream>
//#include "catalan.h"
using namespace std;

#define forn(i,e) for(ll i = 0; i < e; i++)
#define forsn(i,s,e) for(ll i = s; i < e; i++)
#define rforn(i,s) for(ll i = s; i >= 0; i--)
#define rforsn(i,s,e) for(ll i = s; i >= e; i--)
#define ll long long
#define INF 2e18
#define fastio ios_base::sync_with_stdio(false), cin.tie(NULL), cout.tie(NULL)
#define F first
#define S second


// The problem is to find the cost of triangulation with the minimum cost.
// The cost of a triangulation is sum of the weights of its component triangles.

// The idea is to divide the polygon into three parts: a single triangle, 
// the sub-polygon to the left, and the sub-polygon to the right. We try all possible 
// divisions like this and find the one that minimizes the cost of the 
// triangle plus the cost of the triangulation of the two sub-polygons.

struct vertex{
    int x; //x coardinate
    int y; //y coardinate
};

double dist(vertex p1, vertex p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) +
                (p1.y - p2.y)*(p1.y - p2.y));
}
 
// A utility function to find cost of a triangle. The cost is considered
// as perimeter (sum of lengths of all edges) of the triangle
double cost(vector<vertex> polygon, int i, int j, int k)
{
    vertex p1 = polygon[i], p2 = polygon[j], p3 = polygon[k];
    return dist(p1, p2) + dist(p2, p3) + dist(p3, p1);
}
vector<pair<int, pair<int, int>>> tris;
int states[100][100][100];
double dp[110][110];
double MinTriangulate(vector<vertex> polygon, int i, int j){

    if(j < i + 2)
        return 0; // no triangle is possible
    // if(dp[i][j] != -1.0){
    //     return dp[i][j];
    // } 

    double ans = 1e9;
    int a, b, c;
    for(int k = i+1; k<j ; k++){
        if(( MinTriangulate(polygon, i, k) + MinTriangulate(polygon, k, j) + cost(polygon, i, j, k) ) < ans){
            ans = ( MinTriangulate(polygon, i, k) + MinTriangulate(polygon, k, j) + cost(polygon, i, j, k) );
            a = i; b = k; c= j;
        }      
    }
    tris.push_back({a, {b, b}});
    //cout<<ans;
    return  dp[i][j] = ans;

}
 

int main()
{   
    int n;
    cout<<"Enter Number of Vertices:\n";
    cin>>n;
    vector<vertex> polygon;
    cout<<"Enter The Vertices\n";
    forn(i,n){
        int a, b;
        vertex v;
        cin>>a>>b;
        v.x = a;
        v.y = b;
        polygon.push_back(v);
    }
    //cout<<65;
    for(int i = 1; i<= 100; i++) for(int j = 1; j<=100; j++) dp[i][j] = -1.0;
    //cout<<dp[1][2];

    cout<<MinTriangulate(polygon, 0, 4);

    return 0;
}


///// Next Segment

#include <math.h>
#include <bits/stdc++.h>
#include <iostream>
using namespace std;
 
#define forn(i,e) for(ll i = 0; i < e; i++)
#define forsn(i,s,e) for(ll i = s; i < e; i++)
#define rforn(i,s) for(ll i = s; i >= 0; i--)
#define rforsn(i,s,e) for(ll i = s; i >= e; i--)
#define ll long long
#define INF 2e18
#define fastio ios_base::sync_with_stdio(false), cin.tie(NULL), cout.tie(NULL)

ll catalan[100010];
void catalanOn2(){
    // """
    // utility function to calculate the catalan number using Dynamic Programming in O(n);
    // """
    catalan[0] = catalan[1] = 1;

    for(int i = 2; i< 100000; i++){
        for(int j = 0; j<i; j++){
            catalan[i] += catalan[j]*catalan[i - j - 1];
        }
    }

}
ll BinomialCoeff(int n, int k){
    //returns nck
    if(k>n-k)
        k = n-k;
    
    ll res = 1;   // required value n(n-1)(n -2)...(n-k+1)/ (k)*(k-1)*(k-2)..1
    for(int i = 0; i<k; i++){
        res *= (n - i);
        res /= (i + 1);
    }
    return res;
}

void catalanOn(){
    // """
    // utility function to calculate the catalan number using Dynamic Programming in O(n);
    // """
    catalan[0] = catalan[1] = 1;

    for(int i = 2; i< 10000; i++){
        ll c = BinomialCoeff(2*i, i);
        catalan[i] = c/(i + 1);
    }

}

int main()
{   
    
    catalanOn();
    cout<<"Enter Required C(n) value: ";
    int n;
    cin>>n;
    cout<<"Catalan("<<n<<')'<<" = "<<catalan[n];

    return 0;
}