//
//  main.cpp
//  draw
//
//  Created by Shane on 2020/06/15.
//  Copyright © 2020 Shane. All rights reserved.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
 
using namespace std;
using namespace cv;
#define PI 3.141592
#define MAPSIZE 90
 
int omap[MAPSIZE][MAPSIZE];
IplImage *vimg;
double r_x;
double r_y;
double r_r[4];
double l_x[4];
double l_y[4];

void get_angle(double d[2]){
    r_r[0]=atan(d[1]/d[0]);
    r_r[1]=atan(d[1]/d[0])-45*PI/180;
    r_r[2]=atan(d[1]/d[0])-45*PI/180;
    r_r[3]=atan(d[1]/d[0])-90*PI/180;
}

void get_pose(double l[2]){
    r_x=1+l[0]*10;
    r_y=1+l[1]*10;
}

void get_data(double l[4]){
    for (int i=0;i<4;i++){
        l_x[i]=10*l[i]/sqrt(1+pow(tan(r_r[i]),2))+r_x;
        l_y[i]=tan(r_r[i])*10*l[i]/sqrt(1+pow(tan(r_r[i]),2))+r_y;
    }
}

void draw_robot(IplImage *Img){
    CvPoint p1,p2,p3;
    p1.x=r_x;
    p1.y=r_y;
    p2.x=r_x-1*cos(1/3-r_r[1]);
    p2.y=r_y+1*sin(1/3-r_r[1]);
    p3.x=r_x-1*sin(1/3-r_r[1]);
    p3.y=r_y-1*cos(1/3-r_r[1]);
    cvLine(Img, p1, p2, Scalar(0,0,0));
    cvLine(Img, p1, p3, Scalar(0,0,0));
}

void map_value(int omap[MAPSIZE][MAPSIZE]){
    for (int l=0;l<4;l++)
    {
        int c=ceil(r_x);
        printf("r_x=%lf,\n",r_x);
        printf("r_r=%lf,\n",r_r[l]);
        int d=floor(l_x[l]);
        for(int x=c;x<d;x++)
        {
            printf("c=%d,\n",c);
            printf("l_x=%lf,\n",l_x[l]);
            printf("x=%d,\n",x);
            //int x=i;
            int y=r_y+(x-r_x)*tan(r_r[l]);
            printf("y=%d,\n",y);
            //if (y-x*tan(r_r[l])>0.5)
            //y=y-1;
            //if(omap[x][y]!=1)
            omap[x][y]=1;
            //printf("r_x=%lf,\n",r_x);
        }
        int a=0;
        int b=0;
        a=floor(l_x[l]);
        b=floor(l_y[l]);
        omap[a][b]=2;
    }
    
    for(int i=0;i<MAPSIZE;i++)
    {
    for(int j=0;j<MAPSIZE;j++)
    {
        printf("map_value=%d,\n",omap[i][j]);
    }
    }
     
}

void draw_map(IplImage *img,int omap[MAPSIZE][MAPSIZE]){
    CvPoint  p1,p2;
    for(int i=0;i<MAPSIZE;i++) {
        for(int j=0;j<MAPSIZE;j++){
            p1.x=i;
            p1.y=j;
            p2.x=i+1;
            p2.y=j+1;
            int omap_value;
            omap_value=omap[i][j];
            if(omap_value==1){
                cvRectangle(img, p1, p2,Scalar(255,255,255),CV_FILLED, 8,0);}
            else if(omap_value==2)
                cvRectangle(img, p1, p2,Scalar (0,0,0),CV_FILLED, 8,0);
        }
    }
}

int main()
{
    
    // 设置窗口
    //Mat img = Mat::zeros(Size(MAPSIZE, MAPSIZE), CV_8UC3);
    vimg = cvCreateImage(cvSize(MAPSIZE, MAPSIZE), IPL_DEPTH_8U, 3);
    //img.setTo(125);              // 设置屏幕为
    //draw edge
    
    CvPoint p6,p7;
    p6.x = 0;
    p6.y = 0;
    p7.x = MAPSIZE;
    p7.y = MAPSIZE;
    cvRectangle(vimg,p6,p7, CV_RGB(125,125,125), CV_FILLED, 8,0);
    
    Point p1(1,1);
    //Point p2(81,1);
    Point p3(81,61);
    //Point p4(1,61);
    //cvLine(vimg, p1, p2, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p1, p4, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p2, p3, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p3, p4, CV_RGB(255, 255, 255), 3, 8, 0);
    cvRectangle(vimg,p1,p3, CV_RGB(0,0,0), 1, 8,0);
    for(int i=0;i<MAPSIZE;i++){
        for(int j=0;j<MAPSIZE;j++){
            omap[i][j]=0;
        }
    };
    //int RTC::current_pose_out[2];
    //int RTC::current_velocity_out=;
    //int RTC::current_laser_out;
    double a[2];
    double b[2];
    double c[4];
    if (m_target_velocityIn.isNew())
    {
        m_target_velocityIn.read();
        a[0] = m_target_velocity.data.vx;
        a[1] = m_target_velocity.data.vy;
    }
    if (m_target_positionIn.isNew())
    {
        m_target_positionIn.read();
        b[0] = m_target_position.data.position.x;
        b[1] = m_target_position.data.position.y;
    }
    if (m_ir_sensor_metre_outIn.isNew())
    {
        m_ir_sensor_metre_outIn.read();
        c[0] = m_ir_sensor_metre_out.data[0];
        c[1] = m_ir_sensor_metre_out.data[1];
        c[2] = m_ir_sensor_metre_out.data[2];
        c[3] = m_ir_sensor_metre_out.data[3];
    }
    
    //input robot location
    get_pose(a);
    //printf("pose=%lf,\n",r_x);
    //printf("pose=%lf,\n",r_y);
    get_angle(b);
    //printf("angle=%lf,\n",r_r[0]);
    //printf("angle=%lf,\n",r_r[1]);
    //input laser data
    get_data(c);
    printf("angle=%lf,\n",l_x[0]);
    printf("angle=%lf,\n",l_y[0]);
    
    map_value(omap);
    //IplImage Img = IplImage(img);
    draw_robot(vimg);
    draw_map(vimg,omap);
 /*
    for(int i=0;i<MAPSIZE;i++){
    for(int j=0;j<MAPSIZE;j++){
        printf("map_value=%d,\n",omap[i][j]);}}
  */
    /*
    Point p1(100, 100);          // 点p1
    Point p2(758, 50);           // 点p2
 
    // 画直线函数
    line(img, p1, p2, Scalar(0, 0, 255), 2);   // 红色
    line(img, Point(300, 300), Point(758, 400), Scalar(0, 255, 255), 3);
 
    Point p(20, 20);//初始化点坐标为(20,20)
    circle(img, p, 1, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
 
    Point p4;
    p4.x = 300;
    p4.y = 300;
    circle(img, p4, 100, Scalar(120, 120, 120), -1);
 
    int thickness = 3;
    int lineType = 8;
    double angle = 30;  //椭圆旋转角度
    ellipse(img, Point(100, 100), Size(90, 60), angle, 0, 360, Scalar(255, 255, 0), thickness, lineType);
 
 
    // 画矩形
    Rect r(250, 250, 120, 200);
    rectangle(img, r, Scalar(0, 255, 255), 3);
 */
    //imshow("draw", img);
    cvShowImage("draw", vimg);
    waitKey();
    int map[80][60][3];
    for (int i = 0; i < 80; i++) {
        for (int j = 0; j < 60; j++) {
            for (int k = 0; k < 3; k++) {
                int v;
                v = omap[i + 1][j + 1];
                m_map.data[j * 80 + i * 3 + k] = map[i][60 - j][v];
            }
        }
    }


}
/*
void initialize(int map[MAPSIZE][MAPSIZE]){
    for(int i=0;i<MAPSIZE;i++) {
        for(int j=0;j<MAPSIZE;j++){
            map[i][j]=0;
        }
    }
}
*/
/*
void get_angle(double d[2]){
    for(int i=0;i<4;i++){
    if(i==1|i==2){
        r_r[i]=atan(d[1]/d[0]);
    }
    else
        r_r[i]=atan(d[1]/d[0])+45/180*PI;
    }
}

void get_pose(double l[2]){
    r_x=10+l[0]*100;
    r_y=10+l[1]*100;
}

void get_data(double l[4]){
    for (int i=0;i<4;i++){
        l_x[0]=l[0]/sqrt(1+pow(tan(r_r[i]),2))+r_x;
        l_y[0]=tan(r_r[i])*l[0]/sqrt(1+pow(tan(r_r[i]),2))+r_y;
    }
}

void draw_robot(IplImage *Img){
    CvPoint p1,p2,p3;
    p1.x=r_x;
    p1.y=r_y;
    p2.x=r_x-100*cos(1/3-r_r[1]);
    p2.y=r_y+100*sin(1/3-r_r[1]);
    p3.x=r_x-100*sin(1/3-r_r[1]);
    p3.y=r_y-100*cos(1/3-r_r[1]);
    cvLine(Img, p1, p2, Scalar(255,255,255));
    cvLine(Img, p1, p3, Scalar(255,255,255));
}

void map_value(int map[MAPSIZE][MAPSIZE]){
    int x,y=0;
    for (int l=0;l<4;l++){
        for(int i=ceil(r_x);i<l_x[i];i++){
        x=i;
        y=ceil(x*tan(r_r[l]));
        if (y-x*tan(r_r[l])>0.5)
            y--;
        map[x][y]=1;
        map[x-1][y-1]=1;
        }
        int a=0;
        int b=0;
        a=floor(l_x[l]);
        b=floor(l_y[l]);
        map[a][b]=2;
    }
}

void draw_map(IplImage *Img,int map[MAPSIZE][MAPSIZE]){
    CvPoint  p1,p2;
    for(int i=0;i<MAPSIZE;i++) {
        for(int j=0;j<MAPSIZE;j++){
            p1.x=i;
            p1.y=j;
            p2.x=i+1;
            p2.y=j+1;
            int map_value;
            map_value=map[i][j];
            if(map_value==1)
                cvRectangle(Img, p1, p2,Scalar(255,255,255),-1);
            else if(map_value==2)
                cvRectangle(Img, p1, p2,Scalar (0,0,0),-1);
            
        }
    }
}
*/

/*
void draw_node(struct robot_position *robot, IplImage *draw_Image, int R, int G, int B)
{
    int i, j;
    for (i = 0; i < robot->node_n; i++)
    {
        //robot->node_x = (int)robot->node[i][0]*1000;
        //robot->node_y = (int)robot->node[i][1]*1000;
        robot->node_x = CENTER - (int)robot->node[i][0] * 1000 / MAPRATE;
        robot->node_y = CENTER - (int)robot->node[i][1] * 1000 / MAPRATE;
        
        
        cvCircle(draw_Image, cvPoint(robot->node_x, robot->node_y), 4, CV_RGB(B, G, R), -1, 8, 0);
        cvCircle(draw_Image, cvPoint(robot->node_x, robot->node_y), 4, CV_RGB(B, G, R), -1, 8, 0);
        for (j = 0; j<robot->node_n; j++)
        {
            if (i != j)
            {
                //robot->node_x2 = (int)robot->node[j][0]*1000;
                //robot->node_y2 = (int)robot->node[j][1]*1000;
                robot->node_x2 = CENTER - (int)robot->node[j][0] * 1000 / MAPRATE;
                robot->node_y2 = CENTER - (int)robot->node[j][1] * 1000 / MAPRATE;
                
                
                if (robot->edge[i][j] == 1 && robot->edge[j][i] == 1)
                {
                    cvLine(draw_Image, cvPoint(robot->node_x, robot->node_y), cvPoint(robot->node_x2, robot->node_y2), CV_RGB(255, 0, 255), 3, 8, 0);
                }
            }
        }
    }
    
}

void draw_robot(struct robot_position *robot, double robot_size, IplImage *draw_Image, int R, int G, int B)
{
    CvPoint rpt, rpt2, rpt3;
    rpt.x = robot->map_x;
    rpt.y = robot->map_y;
    rpt2.x = (int)(robot->map_x + (-robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) - (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
    rpt2.y = (int)(robot->map_y + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) + (-robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
    rpt3.x = (int)(robot->map_x + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) - (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
    rpt3.y = (int)(robot->map_y + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) + (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
    cvLine(draw_Image, rpt, rpt2, CV_RGB(B, G, R), 3, 8, 0);
    cvLine(draw_Image, rpt, rpt3, CV_RGB(B, G, R), 3, 8, 0);
}


void draw_LRFdata(struct robot_position *robot, long *lrf_data, int center_id, int num_lrf, IplImage *drawImage, int R, int G, int B)
{
    int i;
    double draw_x, draw_y;
    double s1, c1;
    double x1, y1;
    CvPoint pt1;
    
    for (i = center_id - num_lrf; i<center_id + num_lrf; i++) {
        if (i >= 94 && i <= 674) {
            if (lrf_data[i] > 100 && lrf_data[i] < 3900) {
                x1 = lrf_data[i] * sin((((double)i - 384.0)*(360.0 / 1024.0)*M_PI) / 180.0);
                y1 = lrf_data[i] * cos((((double)i - 384.0)*(360.0 / 1024.0)*M_PI) / 180.0);
                
                s1 = sin(robot->rangle*M_PI / 180.0);
                c1 = cos(robot->rangle*M_PI / 180.0);
                
                draw_x = x1*c1 - y1*s1 + robot->real_x;
                draw_y = x1*s1 + y1*c1 + robot->real_y;
                
                pt1.x = CENTER - (int)(draw_x / MAPRATE);
                pt1.y = CENTER - (int)(draw_y / MAPRATE);
                cvLine(drawImage, cvPoint(robot->map_x, robot->map_y), pt1, CV_RGB(B, G, R), 1, 8, 0);
            }
        }
    }
}

void draw_globalMap(int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], IplImage *omapImage, IplImage *amapImage, IplImage *mmapImage[])
{
    int i, j, l;
    
    for (i = 0; i<MAPSIZE; i++)
        for (j = 0; j<MAPSIZE; j++) {
            if (omap[i][j] < -1) {
                omapImage->imageData[omapImage->widthStep*j + i * 3] = 255;
                omapImage->imageData[omapImage->widthStep*j + i * 3 + 1] = 255;
                omapImage->imageData[omapImage->widthStep*j + i * 3 + 2] = 255;
                
                if (mmapImage[0] != NULL)
                    for (l = 0; l<10; l++) {
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3] = 255;
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 1] = 255;
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 2] = 255;
                    }
            }
            else if (omap[i][j] > 1){//} && omap[i][j]) {
                omapImage->imageData[omapImage->widthStep*j + i * 3] = 0;//255;//0;
                omapImage->imageData[omapImage->widthStep*j + i * 3 + 1] = 0;//255;//0
                omapImage->imageData[omapImage->widthStep*j + i * 3 + 2] = 0;//255;//0
                
                if (mmapImage[0] != NULL)
                    for (l = 0; l<10; l++) {
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3] = 0;
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 1] = 0;
                        mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 2] = 0;
                    }
            }
            
            if (amap[i][j] > 0 && amap != NULL) {
                amapImage->imageData[amapImage->widthStep*j + i * 3] = 0;
                amapImage->imageData[amapImage->widthStep*j + i * 3 + 1] = 0;
                amapImage->imageData[amapImage->widthStep*j + i * 3 + 2] = 0;
            }
        }
}

double cal_probability(int k)
{
    double e;
    e = tanh((double)k / 10.0);
    return e;
}

void draw_map(int k, int l, int map_value, IplImage *omap_Image)
{
    CvPoint pt;
    double p;
    
    if (map_value > 0) {
        p = cal_probability(map_value);
        if (p >= 0.8) {
            pt.x = k;
            pt.y = l;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 0;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 0;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 0;
        }
        else {
            pt.x = k;
            pt.y = l;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 125 - (int)(125 * p);
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 125 - (int)(125 * p);
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 125 - (int)(125 * p);
        }
    }
    else {
        
        p = cal_probability(fabs(map_value));
        if (p >= 0.8) {
            pt.x = k;
            pt.y = l;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 255;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 255;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 255;
        }
        else {
            pt.x = k;
            pt.y = l;
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 125 + (int)(125 * p);
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 125 + (int)(125 * p);
            omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 125 + (int)(125 * p);
        }
    }
}

void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], IplImage *omap_Image)
{
    if (sflag == 1) {
        if (k >= pt_x)
            omap[k][l] += 2;
        else
            omap[k][l]--;
    }
    else if (sflag == 2) {
        if (k <= pt_x)
            omap[k][l] += 2;
        else
            omap[k][l]--;
    }
    else if (sflag == 3) {
        if (l >= pt_y)
            omap[k][l] += 2;
        else
            omap[k][l]--;
    }
    else {
        if (l <= pt_y)
            omap[k][l] += 2;
        else
            omap[k][l]--;
    }
    
    if (cal_probability(amap[k][l]) > 0.85)
        if (omap[k][l] < 0 && amap[k][l] + omap[k][l] > 0)
            omap[k][l] += amap[k][l];
    
    draw_map(k, l, omap[k][l], omap_Image);
    
}

void map_building2(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], IplImage *omap_Image, IplImage *amap_Image)
{
    
    int pt_x, pt_y, ptc_x, ptc_y;
    double x_3, y_3;
    CvPoint pt, pt2;
    int dx, dy;
    int j, e, k, l;
    double s1, c1;
    s1 = sin((double)robot->dangle*PI / 180.0);
    c1 = cos((double)robot->dangle*PI / 180.0);
    ptc_x = CENTER - (int)robot->real_x / MAPRATE;
    ptc_y = CENTER - (int)robot->real_y / MAPRATE;
    
    for (j = 0; j<1080; j++) {
        if (x[j] != ERR && y[j] != ERR) {
            x_3 = (double)x[j]*c1 - (double)y[j]*s1 + (double)robot->real_x;
            y_3 = (double)x[j]*s1 + (double)y[j]*c1 + (double)robot->real_y;
            pt_x = CENTER - (int)x_3 / MAPRATE;
            pt_y = CENTER - (int)y_3 / MAPRATE;
            if(pt_x < 0) continue;
            if (pt_y < 0) continue;
            amap[pt_x][pt_y]++;
            dx = (pt_x - ptc_x);
            dy = (pt_y - ptc_y);
            
            e = 0;
            if (dx >= 0 && dy >= 0) {
                if (dx>dy) {
                    for (l = ptc_y, k = ptc_x; k <= pt_x; k++) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l++;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap, omap_Image);
                    }
                }
                else {
                    for (l = ptc_y, k = ptc_x; l <= pt_y; l++) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k++;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap, omap_Image);
                    }
                }
            }
            else if (dx >= 0 && dy <= 0) {
                dy *= -1;
                if (dx>dy) {
                    for (l = ptc_y, k = ptc_x; k <= pt_x; k++) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l--;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap, omap_Image);
                    }
                }
                else {
                    for (l = ptc_y, k = ptc_x; l >= pt_y; l--) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k++;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap, omap_Image);
                    }
                }
            }
            else if (dx <= 0 && dy >= 0) {
                dx *= -1;
                if (dx>dy) {
                    for (l = ptc_y, k = ptc_x; k >= pt_x; k--) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l++;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap, omap_Image);
                    }
                }
                else {
                    for (l = ptc_y, k = ptc_x; l <= pt_y; l++) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k--;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap, omap_Image);
                    }
                }
            }
            else {
                dx *= -1;
                dy *= -1;
                if (dx>dy) {
                    for (l = ptc_y, k = ptc_x; k >= pt_x; k--) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l--;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap, omap_Image);
                    }
                }
                else {
                    for (l = ptc_y, k = ptc_x; l >= pt_y; l--) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k--;
                        }
                        
                        calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap, omap_Image);
                    }
                }
            }
            
            if (amap[pt_x][pt_y]>0 && amap_Image != NULL) {
                pt.x = CENTER - (int)x_3 / MAPRATE;
                pt.y = CENTER - (int)y_3 / MAPRATE;
                pt2.x = CENTER - (int)x_3 / MAPRATE;
                pt2.y = CENTER - (int)y_3 / MAPRATE;
                amap_Image->imageData[amap_Image->widthStep*pt.y + pt.x * 3] = 0;
                amap_Image->imageData[amap_Image->widthStep*pt.y + pt.x * 3 + 1] = 0;
                amap_Image->imageData[amap_Image->widthStep*pt.y + pt.x * 3 + 2] = 0;
            }
            
            draw_map(pt_x, pt_y, omap[pt_x][pt_y], omap_Image);
        }
    }
}
*/
