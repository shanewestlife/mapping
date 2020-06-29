// -*- C++ -*-
/*!
 * @file  mapping.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "mapping.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
#define PI 3.141592
#define MAPSIZE 90

int omap[MAPSIZE][MAPSIZE];
IplImage* vimg;
double r_x;
double r_y;
double r_r[4];
double l_x[4];
double l_y[4];

// Module specification
// <rtc-template block="module_spec">
static const char* mapping_spec[] =
  {
    "implementation_id", "mapping",
    "type_name",         "mapping",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "shane",
    "category",          "mapping",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
mapping::mapping(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_target_positionIn("target_position", m_target_position),
    m_target_velocityIn("target_velocity", m_target_velocity),
    m_ir_sensor_metre_outIn("ir_sensor_metre_out", m_ir_sensor_metre_out),
    m_mapOut("map", m_map)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
mapping::~mapping()
{
}



RTC::ReturnCode_t mapping::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("target_position", m_target_positionIn);
  addInPort("target_velocity", m_target_velocityIn);
  addInPort("ir_sensor_metre_out", m_ir_sensor_metre_outIn);
  
  // Set OutPort buffer
  addOutPort("map", m_mapOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t mapping::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t mapping::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t mapping::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

void get_angle(double d[2]) {
    r_r[0] = atan(d[1] / d[0]);
    r_r[1] = atan(d[1] / d[0]) - 45 * PI / 180;
    r_r[2] = atan(d[1] / d[0]) - 45 * PI / 180;
    r_r[3] = atan(d[1] / d[0]) - 90 * PI / 180;
}

void get_pose(double l[2]) {
    r_x = 1 + l[0] * 10;
    r_y = 1 + l[1] * 10;
}

void get_data(double l[4]) {
    for (int i = 0; i < 4; i++) {
        l_x[i] = 10 * l[i] / sqrt(1 + pow(tan(r_r[i]), 2)) + r_x;
        l_y[i] = tan(r_r[i]) * 10 * l[i] / sqrt(1 + pow(tan(r_r[i]), 2)) + r_y;
    }
}

void draw_robot(IplImage* Img) {
    CvPoint p1, p2, p3;
    p1.x = r_x;
    p1.y = r_y;
    p2.x = r_x - 1 * cos(1 / 3 - r_r[1]);
    p2.y = r_y + 1 * sin(1 / 3 - r_r[1]);
    p3.x = r_x - 1 * sin(1 / 3 - r_r[1]);
    p3.y = r_y - 1 * cos(1 / 3 - r_r[1]);
    cvLine(Img, p1, p2, Scalar(0, 0, 0));
    cvLine(Img, p1, p3, Scalar(0, 0, 0));
}

void map_value(int omap[MAPSIZE][MAPSIZE]) {
    for (int l = 0; l < 4; l++)
    {
        int c = ceil(r_x);
        printf("r_x=%lf,\n", r_x);
        printf("r_r=%lf,\n", r_r[l]);
        int d = floor(l_x[l]);
        for (int x = c; x < d; x++)
        {
            //printf("c=%d,\n", c);
            //printf("l_x=%lf,\n", l_x[l]);
            //printf("x=%d,\n", x);
            //int x=i;
            int y = r_y + (x - r_x) * tan(r_r[l]);
            printf("y=%d,\n", y);
            //if (y-x*tan(r_r[l])>0.5)
            //y=y-1;
            //if(omap[x][y]!=1)
            omap[x][y] = 1;
            //printf("r_x=%lf,\n",r_x);
        }
        int a = 0;
        int b = 0;
        a = floor(l_x[l]);
        b = floor(l_y[l]);
        omap[a][b] = 2;
    }
    /*
    for (int i = 0; i < MAPSIZE; i++)
    {
        for (int j = 0; j < MAPSIZE; j++)
        {
            printf("map_value=%d,\n", omap[i][j]);
        }
    }
    */
}

void draw_map(IplImage* img, int omap[MAPSIZE][MAPSIZE]) {
    CvPoint  p1, p2;
    for (int i = 0; i < MAPSIZE; i++) {
        for (int j = 0; j < MAPSIZE; j++) {
            p1.x = i;
            p1.y = j;
            p2.x = i + 1;
            p2.y = j + 1;
            int omap_value;
            omap_value = omap[i][j];
            if (omap_value == 1) {
                cvRectangle(img, p1, p2, Scalar(255, 255, 255), CV_FILLED, 8, 0);
            }
            else if (omap_value == 2)
                cvRectangle(img, p1, p2, Scalar(0, 0, 0), CV_FILLED, 8, 0);
        }
    }
}

RTC::ReturnCode_t mapping::onExecute(RTC::UniqueId ec_id)
{
    vimg = cvCreateImage(cvSize(MAPSIZE, MAPSIZE), IPL_DEPTH_8U, 3);
    //img.setTo(125);              // 设置屏幕为
    //draw edge

    CvPoint p6, p7;
    p6.x = 0;
    p6.y = 0;
    p7.x = MAPSIZE;
    p7.y = MAPSIZE;
    cvRectangle(vimg, p6, p7, CV_RGB(125, 125, 125), CV_FILLED, 8, 0);

    Point p1(1, 1);
    //Point p2(81,1);
    Point p3(81, 61);
    //Point p4(1,61);
    //cvLine(vimg, p1, p2, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p1, p4, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p2, p3, CV_RGB(255, 255, 255), 3, 8, 0);
    //cvLine(vimg, p3, p4, CV_RGB(255, 255, 255), 3, 8, 0);
    cvRectangle(vimg, p1, p3, CV_RGB(0, 0, 0), 1, 8, 0);
    for (int i = 0; i < MAPSIZE; i++) {
        for (int j = 0; j < MAPSIZE; j++) {
            omap[i][j] = 0;
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
    printf("angle=%lf,\n", l_x[0]);
    printf("angle=%lf,\n", l_y[0]);

    map_value(omap);
    //IplImage Img = IplImage(img);
    draw_robot(vimg);
    draw_map(vimg, omap);
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
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t mapping::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t mapping::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void mappingInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapping_spec);
    manager->registerFactory(profile,
                             RTC::Create<mapping>,
                             RTC::Delete<mapping>);
  }
  
};


