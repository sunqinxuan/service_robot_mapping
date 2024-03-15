#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <iostream>
#include <ostream>
#include <fstream>
#include <GL/freeglut.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ANN/ANN.h>
#include <vector>
#include <string.h>




using namespace std;
using namespace cv;

#ifndef _ICP_H_
#define _ICP_H_
#define M_PI       3.14159265358979323846
#define uchar unsigned char



typedef struct Point3D {
	float x;
	float y;
	float z;
	uchar r;
	uchar g;
	uchar b;
} Point3D;

typedef struct Point3DSet {
	Point3D *point;
	int number;
	int invalid_point_number;
} Point3DSet;

typedef struct Rotation {
	float q0;
	float q1;
	float q2;
	float q3;
} Rotation;

typedef struct Translation {
	float t0;
	float t1;
	float t2;
} Translation;

typedef struct Transformation {
	Rotation R;
	Translation T;
} Transformation;


//////////////////////////////////new added/////////////////////
typedef struct Pose {
	float orgx;
	float orgy;
	float orgz;
	float x1;
	float x2;
	float x3;
	float y1;
	float y2;
	float y3;
	float z1;
	float z2;
	float z3;
} Pose;
////////////////////////////////////////////////////////////////




Point3DSet *GetPoint3DSet(unsigned int *dset, int number);
void ReleasePoint3DSet(Point3DSet *pset);
void icp(Point3DSet *data_set, Point3DSet *model_set,int QR_flag);

Point3D GetPoint3DSetMean(Point3DSet *pset);

Point3DSet *loadDataSet(void);
Point3DSet *loadModelSet(void);
void load3dDataToGL(void);
void loadTextureToGL(void);

void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void key(int key, int x, int y);
void reshape (int w, int h);
void renderScene1(void);
void loadData(char *points,char *points_pre,char *image,char *image_pre);
void TransformPoint3DSet(Transformation RT, Point3DSet *pset, Point3DSet *yset);
void Matching(Point3DSet *sel_data_set, Point3DSet *model_set, Point3DSet *yset);
Transformation GetOptimalRotation(Point3DSet *dset, Point3DSet *mset);



#endif /*_ICP_H_*/


#define SIGN(x) ( (x)<0 ? -1:((x)>0?1:0 ) )


//#define width 640
//#define height 480
#define icp_width 640
#define icp_height 480