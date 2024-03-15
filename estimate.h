#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <map>
#include <iterator>
#include <string>
#include <time.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;


typedef struct Position {
	int px;
	int py;
	bool operator < (const Position &p) const
    {
		return px<=p.px;
    }
} Position;

int estimate(void);
