#include <GL/freeglut.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <OpenNI.h>
#include <time.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tchar.h>
#include <strsafe.h>


using namespace std;
using namespace cv;
using namespace openni;

typedef struct point {
	float x;
	float y;
	float ang;
}point;


int capture(void);
int initial(void);
int close(void);