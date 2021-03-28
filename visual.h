#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

#define FOR(i,n) for(int i=0; i<(n); i++)

using namespace cv;
using namespace std;


Mat frame2image(state st,Dataset* data)
{
  vector<vector<vector<int>>> arr(3,vector<vector<int>>(data->N,vector<int>(data->M,0)));
  FOR(i,data->N){
    FOR(j,data->M){
      arr[0][i][j] = data->matrix[i][j];
    }
  }
  FOR(i,st.taskPos.size()){
    pos p = st.taskPos[i];
    arr[1][p.x][p.y] = i+1;
  }
  FOR(i,st.botPos.size()){
    pos p = st.botPos[i];
    arr[2][p.x][p.y] = i+1;
  }

  Mat img(100*data->N,100*data->M,CV_8UC3,Scalar(0,0,0));
  FOR(i,data->N){
    FOR(j,data->M){
      cv::Rect rect1(j*100,i*100, 100, 100);
      cv::Rect rect2(j*100+15,i*100+15, 70, 70);
      cv::Point cirCenter(j*100+50,i*100+50);
      //TODO: arr to be redefinedS
      FOR(k,3){
        if(k==0){
          if(arr[k][i][j]==0) 
            cv::rectangle(img, rect1, cv::Scalar(255, 255, 255),FILLED);
          else if(arr[k][i][j]==2)
            cv::rectangle(img, rect1, cv::Scalar(0, 0, 0),FILLED);
          else
            cv::rectangle(img, rect1, cv::Scalar(102, 255, 255),FILLED);
        }
        else if(k==1 && arr[k][i][j]!=0){
          string txt = "T"+to_string(arr[k][i][j]);
          cv::rectangle(img, rect2, cv::Scalar(0, 255, 0),FILLED);
          putText(img,txt,cirCenter-Point(20,20),FONT_HERSHEY_PLAIN,1,Scalar(255,255,255),2,false);
        }
        else if(k==2 && arr[k][i][j]!=0){
          string txt = "R"+to_string(arr[k][i][j]);
          cv::circle(img, cirCenter,30, cv::Scalar(0, 0, 255),FILLED);
          putText(img,txt,cirCenter,FONT_HERSHEY_PLAIN,1,Scalar(0,0,0),2,false);
        }
      }
    }
  }

  Mat dst(720,1080,CV_8UC3);
  cv::resize(img,dst,dst.size());
  return dst;
}