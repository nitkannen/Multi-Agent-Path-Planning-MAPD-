#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
//#include "struct_class.h"
#include "algorithm.h"
#include "visual.h"

#define FOR(i,n) for(int i=0; i<(n); i++)
#define sm(a) cout << #a << ": \n"; for (auto i : a) { for(auto j : i) cout<<j<<' '; cout<<'\n'; }

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////  data input & output function   /////////////////////////


Dataset *readfile(string filename = "./input/input.txt")
{

    fstream inFile;
    inFile.open(filename, ios::in);
    if (!inFile) {
      cout << "File not created!"<<endl;
    }
    else {
      cout << "File opened successfully!\n";
    }
    cout<<"reached";
    int n,m,bt,tsk,st,obs;
    inFile>>n>>m>>bt>>tsk>>st>>obs;
    Dataset *data = new Dataset(n,m,bt,tsk,st,obs);

    FOR(i,bt){
        int sx, sy, fx, fy;
        inFile>>sx>>sy>>fx>>fy;
        data->botArr[i] = robot(i,sx,sy,fx,fy);
        data->matrix[sx][sy] = 1;
    }
    FOR(i,tsk){
        int sx, sy, fx, fy;
        inFile>>sx>>sy>>fx>>fy;
        data->taskArr[i] = task(i,sx,sy,fx,fy);
    }
    FOR(i,st){
        int px, py;
        inFile>>px>>py;
        data->matrix[px][py] = 2;
    }
    FOR(i,obs){
        int px, py;
        inFile>>px>>py;
        data->matrix[px][py] = 1;
    }

    floydWarshall(data);
    cout<<"reading done"<<endl;
    inFile.close();

    return data;
}

//bot1: (1,1,_)->(1,2,_)->(1,3,T1)->(1,4,T1)->(1,4,T1)
void writefile(string outputfileName , Solution *sol, Dataset* data)
{
 
   fstream outputfile;
   outputfile.open(outputfileName, ios::out);
   if(outputfile)
    cout<<"output File created correctly"<<endl;
   else cout<<"output File not opened"<<endl;

   if (outputfile.is_open())
   {    outputfile<<"Total time taken is: "<<sol->totalTime<<endl;
        int _botN = data->botN;
        for (int r = 0; r < _botN; r++) {
            outputfile << "Bot #" << r << ": ";
            for (auto st : sol->frames) {
                outputfile<<"-> ( " << (st.botPos[r].x) << ", " << (st.botPos[r].y) << ", ";
                if (st.botTask[r] == -1)
                    outputfile << "_ ) ";
                else
                    outputfile << "T"<<st.botTask[r] << " ) ";
            }
            outputfile<<endl;
        }
    }
   else
       cout << "outputfile open error!!" << endl;
    outputfile.close();
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// MAIN PROGRAM ///////////////////////////////////
const char* keys =
"{help h usage ? | | Usage examples: \n\t\t./AIFA --input=./input/input.txt --input=./input/input.txt --s}"
"{input in       |<none>| input file name   }"
"{output out   |<none>| output file name }"
"{simulate s    |<none>| whether simulate or not}"
;



int main(int argc, char **argv)
{   

    CommandLineParser parser(argc, argv, keys);
    parser.about("Use this script to run lane ,car and glare detection");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    string inputFile = "./input/input.txt"; //path to input file
    string outputFile = "./output/t1.txt";       //path to output file
    bool simulate = true;

    if (parser.has("input")) {
        inputFile = parser.get<String>("input");
        cout<<"input file: "<<inputFile<<endl;
    }

    if (parser.has("output")) {
        outputFile = parser.get<String>("output");
        cout<<"output file: "<<outputFile<<endl;
    }

    if(parser.has("simulate")){
        simulate=true;
    }
    else {
        simulate=false;
    }

    Dataset* data = readfile(inputFile);
    sm(data->matrix);cout<<endl;
    sm(data->minDistTable);
    Solution* sol = solver(data);
    writefile(outputFile,sol,data);
    if(simulate){
      FOR(i,sol->frames.size()){
        Mat img = frame2image(sol->frames[i],data);
        imshow("Simulation",img);
        waitKey(0);
        string txt = "./output/frame"+to_string(i+1)+".jpg";
        imwrite(txt,img);
        printf("frame no:%d\n",i+1);
      }
      cout<<"!!! Simulation Complete !!!"<<endl;
    }

  return 0;
}