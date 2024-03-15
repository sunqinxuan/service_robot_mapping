#include "icp.h"
#include "estimate.h"
	extern Mat Rf;	
	extern Mat Tf;
	extern Transformation esti;

//---OpenGL ȫ�ֱ���


float xyzdata[480][640][3];
float xyzdata2[480][640][3];

float alpha=0,beta=0,gamma=0;

uchar texture[480][640][3];
uchar texture2[480][640][3];


vector<KeyPoint> keypoints1, keypoints2;
vector<DMatch> matches;
vector<int> index;
vector<Point3D> display;
vector<Transformation> transform;
////////////////////////////////////////////new added//////////////////////////////////
Pose pose;
vector<Pose> traj;
vector<Point3DSet> pointcloud;
///////////////////////////////////////////////////////////////////////////////////////
int zhen;

	Point3DSet *sel_data_set;
    Point3DSet *sel_model_set;
	Point3DSet *sel_data_set_k;

	Transformation trans={{1,0,0,0},{0,0,0}};
	extern CvMat Rqr,Tqr;
	Transformation transqr_tmp={{1,0,0,0},{0,0,0}},transqr={{1,0,0,0},{0,0,0}};
	extern IplImage pImg1,pImg2;
	extern cv::Mat img1,img2;
	Mat Rf_pre=Mat(3,3,CV_32FC1);
	Mat Tf_pre=Mat(3,1,CV_32FC1);

int glWinWidth = 640, glWinHeight = 480;
double eyex, eyey, eyez, atx, aty, atz;  // eye* - �����λ�ã�at* - ע�ӵ�λ��
double sx=1,sy=1,sz=1;

bool leftClickHold = false, rightClickHold = false;
bool wheelup = false, wheeldown = false;
int mx,my;          // ��갴��ʱ�� OpenGL ���ڵ�����
int ry = 90, rx = 90;    // ��������ע�ӵ�Ĺ۲�Ƕ�
double mindepth=200.0, maxdepth=2000.0;      // ������ݵļ�ֵ 
double radius = 3000.0;      // �������ע�ӵ�ľ���

// ��갴����Ӧ����
void mouse(int button, int state, int x, int y)
{
   if(button == GLUT_LEFT_BUTTON)
   {
      if(state == GLUT_DOWN)
      {
         leftClickHold=true;
      }
      else
      {
         leftClickHold=false;
      }
   }

   if (button== GLUT_RIGHT_BUTTON)
   {
      if(state == GLUT_DOWN)
      {
         rightClickHold=true;
      }
      else
      {
         rightClickHold=false;
      }
   }
}

void key(int key, int x, int y)
{
	switch(key)
	{
	case GLUT_KEY_UP:
		sx+=0.1;
		sy+=0.1;
		sz+=0.1;
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		sx-=0.1;
		sy-=0.1;
		sz-=0.1;
		glutPostRedisplay();
		break;
	default:
		sx=sx;
		sy=sy;
		sz=sz;
	}
}

// ����˶���Ӧ����
void motion(int x, int y)
{
   int rstep = 5; 
   if(leftClickHold==true)
   {
      if( abs(x-mx) > abs(y-my) )
      {
         rx += SIGN(x-mx)*rstep;    
      }
      else
      {
         ry -= SIGN(y-my)*rstep;    
      }
      
      mx=x;
      my=y;
      glutPostRedisplay();
   }

   if(rightClickHold==true)
   {
      radius += SIGN(y-my)*100.0;
      radius = std::max( radius, 100.0 );
      mx=x;
      my=y;
      glutPostRedisplay();
   }
}




void renderScene1(void) 
{
   // clear screen and depth buffer
   glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
   // Reset the coordinate system before modifying 
   glLoadIdentity();   
   // set the camera position
   atx = 0.0f;
   aty = 0.0f;
   atz = ( mindepth - maxdepth ) / 2.0f;
   eyex = atx + radius * sin( CV_PI * ry / 180.0f ) * cos( CV_PI * rx/ 180.0f ); 
   eyey = aty + radius * cos( CV_PI * ry/ 180.0f ); 
   eyez = atz + radius * sin( CV_PI * ry / 180.0f ) * sin( CV_PI * rx/ 180.0f );
   gluLookAt ( eyex, eyey, eyez, atx, aty, atz,0.0, -1.0, 0.0);
   //gluLookAt(0.0f,0.0f,6000.0f,0.0f,0.0f,1000.0f,0.0,1.0,0.0);
   glRotatef(0,0,1,0);
   glRotatef(-180,1,0,0);
   glScalef(sx,sy,sz);

   float x,y,z;
   // ����ͼ�����
   glPointSize(2.0); 
   glBegin(GL_POINTS);

   glColor3f(1,0,0);
   glVertex3f(0,0,0);
   //z:b
   glColor3f(0,0,1);
   glVertex3f(0,0,100);
   glColor3f(0,0,1);
   glVertex3f(0,0,500);
   glColor3f(0,0,1);
   glVertex3f(0,0,1000);
   //x:r
   glColor3f(1,0,0);
   glVertex3f(100,0,0);
   glColor3f(1,0,0);
   glVertex3f(500,0,0);
   glColor3f(1,0,0);
   glVertex3f(1000,0,0);
   //y:g
   glColor3f(0,1,0);
   glVertex3f(0,100,0);
   glColor3f(0,1,0);
   glVertex3f(0,500,0);
   glColor3f(0,1,0);
   glVertex3f(0,1000,0);
   //for(vector<Point3D>::iterator it=display.begin();it!=display.end();++it)
   //{
	  // glColor3f((*it).b/255,(*it).g/255,(*it).r/255);//(1,1,1);//
	  // x= (*it).x;
	  // y= (*it).y;
	  // z= (*it).z;
	  // glVertex3f(-x,y,z); 
   //}
   for(vector<Point3DSet>::iterator it=pointcloud.begin();it!=pointcloud.end();++it)
   {
	   for(int i=0;i<(*it).number;i++)
	   {
		   glColor3f((float)(*it).point[i].b/255,(float)(*it).point[i].g/255,(float)(*it).point[i].r/255);//(1,1,1);//
		   x= (*it).point[i].x;
		   y= (*it).point[i].y;
		   z= (*it).point[i].z;
		   glVertex3f(x,y,z); 
	   }
   }
   glEnd(); 

   /////////////////////////////////////////////////////new added//////////////////////////////////
   glBegin(GL_LINES);
   for(vector<Pose>::iterator it=traj.begin();it!=traj.end();++it)
   {
	   glColor3f(1,0,0);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).x1*200),(*it).orgy+(*it).x2*200,(*it).orgz+(*it).x3*200);
	   glColor3f(0,1,0);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).y1*200),(*it).orgy+(*it).y2*200,(*it).orgz+(*it).y3*200);
	   glColor3f(0,0,1);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).z1*200),(*it).orgy+(*it).z2*200,(*it).orgz+(*it).z3*200);
	   if(it!=traj.begin())
	   {
		   glColor3f(1,1,1);
		   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
		   glVertex3f((*(it-1)).orgx,(*(it-1)).orgy,(*(it-1)).orgz);
	   }
   }
   glEnd();
   /////////////////////////////////////////////////////////////////////////////////////////////////

   glFlush();
   glutSwapBuffers();
}


// ���ڱ仯ͼ���ع���Ӧ����
void reshape (int w, int h) 
{
   glWinWidth = w;
   glWinHeight = h;
   glViewport (0, 0, (GLsizei)w, (GLsizei)h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective (45, (GLfloat)w / (GLfloat)h, 1.0, 15000.0);   
   glMatrixMode (GL_MODELVIEW);
}


void loadData(char *points,char *points_pre,char *image,char *image_pre)
{
	
	CvScalar ss1,ss2;
	FILE *fp1,*fp2;

	img1=imread(image);
	img2=imread(image_pre);
	//pImg1= IplImage(img1);
	//pImg2= IplImage(img2);
	uchar *pt1,*pt2;
	
	//accessing the image pixels
	for (int i=0;i<icp_height;i++)
	{
		pt1=img1.ptr<uchar>(i);
		pt2=img2.ptr<uchar>(i);
		for (int j=0;j<icp_width;j++)
		{
			//ss1=cvGet2D(&pImg1,i,j);         // ss.val[0] = red, ss.val[1] = green, ss.val[2] = blue
			//ss2=cvGet2D(&pImg2,i,j);
			//texture[i][j][2] = ss1.val[0];
			//texture[i][j][1] = ss1.val[1];
			//texture[i][j][0] = ss1.val[2];
			//texture2[i][j][2] = ss2.val[0];
			//texture2[i][j][1] = ss2.val[1];
			//texture2[i][j][0] = ss2.val[2];
			texture[i][j][2]=pt1[j*3];
			texture[i][j][1]=pt1[j*3+1];
			texture[i][j][0]=pt1[j*3+2];
			texture2[i][j][2]=pt2[j*3];
			texture2[i][j][1]=pt2[j*3+1];
			texture2[i][j][0]=pt2[j*3+2];
		}
	}  
	fp1=fopen(points,"r");
	fp2=fopen(points_pre,"r");
	for (int i=0;i<icp_height;i++)
	{ 
		for (int j=0;j<icp_width;j++)
		{
			fscanf(fp1,"%f",&xyzdata[i][j][0]);
			fscanf(fp1,"%f",&xyzdata[i][j][1]);
			fscanf(fp1,"%f",&xyzdata[i][j][2]);
			fscanf(fp2,"%f",&xyzdata2[i][j][0]);
			fscanf(fp2,"%f",&xyzdata2[i][j][1]);
			fscanf(fp2,"%f",&xyzdata2[i][j][2]);
		}
	}


	fclose(fp1);
	fclose(fp2);
}



Point3DSet *loadDataSet(void)
{
	Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));	
	pset->point =(Point3D*) malloc(icp_width*icp_height*sizeof(Point3D));
	pset->invalid_point_number = 0;
	int num=0;
	//int i=200;
	for(int i=0;i<icp_height;i+=4)
	{
		for(int j=0;j<icp_width;j+=4)
		{	
			if(xyzdata[i][j][0]+xyzdata[i][j][1]+xyzdata[i][j][2]==0)//||xyzdata[i][j][2]>4000)
				continue;
			else
			{
				pset->point[num].x=-xyzdata[i][j][0];
				pset->point[num].y=xyzdata[i][j][1];
				pset->point[num].z=xyzdata[i][j][2];
				pset->point[num].b=texture[i][j][0];
				pset->point[num].g=texture[i][j][1];
				pset->point[num].r=texture[i][j][2];
				num++;
			}
		}
	}	
	pset->number = num;
	//FILE* fp;
	//fp=fopen("dataset.txt","wt");
	//for(int i=0;i<pset->number;i++)
	//	//fprintf(fp,"%f  %f\n",pset->point[i].x,pset->point[i].z);
	//	fprintf(fp,"%f %f %f\n",pset->point[i].r,pset->point[i].g,pset->point[i].b);
	//fprintf(fp,"%d\n",pset->number);
	//fclose(fp);
	return pset;
}

Point3DSet *loadModelSet(void)
{
	Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));	
	pset->point =(Point3D*) malloc(icp_width*icp_height*sizeof(Point3D));
	pset->invalid_point_number = 0;
	int num=0;
	//int i=200;
	for(int i=0;i<icp_height;i+=4)
	{
		for(int j=0;j<icp_width;j+=4)
		{	
			if(xyzdata2[i][j][0]+xyzdata2[i][j][1]+xyzdata2[i][j][2]==0)//||xyzdata2[i][j][2]>4000)
				continue;
			else
			{
				pset->point[num].x=-xyzdata2[i][j][0];
				pset->point[num].y=xyzdata2[i][j][1];
				pset->point[num].z=xyzdata2[i][j][2];
				pset->point[num].b=texture2[i][j][0];
				pset->point[num].g=texture2[i][j][1];
				pset->point[num].r=texture2[i][j][2];
				num++;
			}
		}
	}	
	pset->number = num;
	//FILE* fp;
	//fp=fopen("modelset.txt","wt");
	//for(int i=0;i<pset->number;i++)
	//	//fprintf(fp,"%f  %f\n",pset->point[i].x,pset->point[i].z);
	//	fprintf(fp,"%f %f %f\n",pset->point[i].r,pset->point[i].g,pset->point[i].b);
	//fprintf(fp,"%d\n",pset->number);
	//fclose(fp);
	return pset;
}

void ReleasePoint3DSet(Point3DSet *pset) {
	free(pset->point);
	free(pset);
}

void Selection(Point3DSet *data_set,Point3DSet *pset)
{
	//Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//pset->point =(Point3D*) malloc((height*width)*sizeof(Point3D));
	//pset->invalid_point_number = 0;

	for(int i=0;i<data_set->number;i++)
	{
		pset->point[i].x=data_set->point[i].x;
		pset->point[i].y=data_set->point[i].y;//0;//
		pset->point[i].z=data_set->point[i].z;
		pset->point[i].r=data_set->point[i].r;
		pset->point[i].g=data_set->point[i].g;
		pset->point[i].b=data_set->point[i].b;
	}
	pset->number=data_set->number;
	//return pset;
}

void Matching(Point3DSet *sel_data_set, Point3DSet *model_set, Point3DSet *yset) 
{
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;	
	queryPt = annAllocPt(3);					// allocate query point
	dataPts = annAllocPts(model_set->number, 3);			// allocate data points
	nnIdx = new ANNidx[1];						// allocate near neigh indices
	dists = new ANNdist[1];						// allocate near neighbor dists
	for(int i=0;i<model_set->number;i++)
	{
		dataPts[i][0]=model_set->point[i].x;
		dataPts[i][1]=model_set->point[i].y;
		dataPts[i][2]=model_set->point[i].z;
	}
	kdTree = new ANNkd_tree(dataPts,model_set->number,3);
					//dataPts,					// the data points
					//nPts,						// number of points
					//dim);						// dimension of space

	/* return closest point set with the same size as selected data_set */
	//Point3DSet *yset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//yset->number = sel_data_set->number;
	//yset->point =(Point3D*) malloc((sel_data_set->number)*sizeof(Point3D));
	//yset->invalid_point_number = 0;

	/* Find closest point set Y for data set */
	int i, j;
	int closest_index = 0;
	float distance = 0;
	//float minimum_distance = 0;
	int num=0;
	index.clear();
	//FILE* fp=fopen("distance.txt","wt");
	for (i=0; i < sel_data_set->number; i++) //sel_data_set��ÿһ��i
	{
		closest_index = 0;
		//minimum_distance = 1000000000;

		queryPt[0]=sel_data_set->point[i].x;
		queryPt[1]=sel_data_set->point[i].y;
		queryPt[2]=sel_data_set->point[i].z;
		kdTree->annkSearch(queryPt,1,nnIdx,dists,0);		

		//for (j=0; j < model_set->number; j++) //Ѱ�ҵ�i��model_set�е�����㣨��Ϊ����ͬһ������ϵ�£�
		//{
		//	if ((model_set->point[j].x == 0)&&(model_set->point[j].y == 0)) {continue;}
		//	distance = (sel_data_set->point[i].x - model_set->point[j].x)*(sel_data_set->point[i].x - model_set->point[j].x) +
		//				(sel_data_set->point[i].y - model_set->point[j].y)*(sel_data_set->point[i].y - model_set->point[j].y) +
		//				(sel_data_set->point[i].z - model_set->point[j].z)*(sel_data_set->point[i].z - model_set->point[j].z) ;
		//	if (distance < minimum_distance) {
		//		minimum_distance = distance;
		//		closest_index = j;
		//	}
		//}
		
		if(dists[0]<10000)
		{
			yset->point[num].x = model_set->point[nnIdx[0]].x;
			yset->point[num].y = model_set->point[nnIdx[0]].y;
			yset->point[num].z = model_set->point[nnIdx[0]].z;
			yset->point[num].r = model_set->point[nnIdx[0]].r;
			yset->point[num].g = model_set->point[nnIdx[0]].g;
			yset->point[num].b = model_set->point[nnIdx[0]].b;
			//sel_data_set->point[num].x=sel_data_set->point[i].x;
			//sel_data_set->point[num].y=sel_data_set->point[i].y;
			//sel_data_set->point[num].z=sel_data_set->point[i].z;
			//sel_data_set->point[num].r=sel_data_set->point[i].r;
			//sel_data_set->point[num].g=sel_data_set->point[i].g;
			//sel_data_set->point[num].b=sel_data_set->point[i].b;
			index.push_back(i);
			//fprintf(fp,"%f\n",dists[0]);
			//fprintf(fp,"%f,%f,%f\n",sel_data_set->point[num].x,sel_data_set->point[num].y,sel_data_set->point[num].z);
			//fprintf(fp,"%f,%f,%f\n",yset->point[num].x,yset->point[num].y,yset->point[num].z);
			num++;
		}
	}

	//FILE* idx;
	//idx=fopen("index.txt","wt");
	//for(vector<int>::iterator it=index.begin();it!=index.end();++it)
	//{
	//	fprintf(idx,"%d\n",(*it));
	//}
	//fclose(idx);

	yset->number=num;
	sel_data_set->number=num;
	delete nnIdx;
	delete dists;
	delete kdTree;
	//fprintf(fp,"%d\n",num);
	//fclose(fp);
	//return yset;//��model_set����sel_data_setÿһ�����Ӧ����������
}


Point3D GetMeanOfPointSet(Point3DSet *pset) {
	int i;
	Point3D p_bar;
	p_bar.x = 0;
	p_bar.y = 0;
	p_bar.z = 0;
	for (i=0; i < pset->number; i++) 
	{
		p_bar.x += pset->point[i].x;
		p_bar.y += pset->point[i].y;
		p_bar.z += pset->point[i].z;
	}
	p_bar.x /= pset->number;
	p_bar.y /= pset->number;
	p_bar.z /= pset->number;

	return p_bar;
}

void SetRotationMatrix(CvMat *Matrix, Rotation R) //����Ԫ��������ת����
{
	CvScalar s;//ͼ���������ݣ���ͼ����һͨ���ģ�val[0]�д洢����
				//RGBͼ��B:val[0],G:val[1],R:val[2],alphe:val[3]

	s.val[0] = R.q0*R.q0 + R.q1*R.q1 - R.q2*R.q2 - R.q3*R.q3;
	cvSet2D(Matrix, 0, 0, s);//��Matrix�ĵ㣨0,0����ֵs

	s.val[0] = 2*(R.q1*R.q2 - R.q0*R.q3);
	cvSet2D(Matrix, 0, 1, s);

	s.val[0] = 2*(R.q1*R.q3 + R.q0*R.q2);
	cvSet2D(Matrix, 0, 2, s);

	s.val[0] = 2*(R.q2*R.q1 + R.q0*R.q3);
	cvSet2D(Matrix, 1, 0, s);

	s.val[0] = R.q0*R.q0 - R.q1*R.q1 + R.q2*R.q2 - R.q3*R.q3;
	cvSet2D(Matrix, 1, 1, s);

	s.val[0] = 2*(R.q2*R.q3 - R.q0*R.q1);
	cvSet2D(Matrix, 1, 2, s);

	s.val[0] = 2*(R.q3*R.q1 - R.q0*R.q2);
	cvSet2D(Matrix, 2, 0, s);

	s.val[0] = 2*(R.q3*R.q2 + R.q0*R.q1);
	cvSet2D(Matrix, 2, 1, s);

	s.val[0] = R.q0*R.q0 - R.q1*R.q1 - R.q2*R.q2 + R.q3*R.q3;
	cvSet2D(Matrix, 2, 2, s);
}

Transformation GetOptimalRotation(Point3DSet *dset, Point3DSet *mset) 
{
	//#define GetOptimalRotation_DEBUG_MESSAGE
	Point3D dbar, mbar;
	//dbar = GetMeanOfPointSet(dset);//���Ƶ�ƽ��ֵ�����ģ���
	dbar.x=0;
	dbar.y=0;
	dbar.z=0;
	for (int i=0;i<mset->number;i++)
	{
		dbar.x+=dset->point[index[i]].x;
		dbar.y+=dset->point[index[i]].y;
		dbar.z+=dset->point[index[i]].z;
	}
	dbar.x /= mset->number;
	dbar.y /= mset->number;
	dbar.z /= mset->number;

	mbar = GetMeanOfPointSet(mset);
	//cout<<"dbar:"<<dbar.x<<","<<dbar.y<<","<<dbar.z<<endl;
	//cout<<"mbar:"<<mbar.x<<","<<mbar.y<<","<<mbar.z<<endl;
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintPoint3D(dbar);
	PrintPoint3D(mbar);
	#endif

	CvMat *P = cvCreateMat(4,4,CV_32FC1);
	float Array[]={0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0};
	cvSetData(P, Array, P->step);
	//PrintMatrix(P, P->rows, P->cols);

	float Sxx=0, Sxy=0, Sxz=0, Syx=0, Syy=0, Syz=0, Szx=0, Szy=0, Szz=0;
	float Mcx, Mcy, Mcz, Dcx, Dcy, Dcz;
	int i;
	//cout<<"index:"<<index.size()<<endl;
	for (i=0; i<mset->number; i++) {
		Mcx = mset->point[i].x - mbar.x;//sel_data_set��model�������ֱ��ȥ��ֵ
		Mcy = mset->point[i].y - mbar.y;
		Mcz = mset->point[i].z - mbar.z;
		Dcx = dset->point[index[i]].x - dbar.x;
		Dcy = dset->point[index[i]].y - dbar.y;
		Dcz = dset->point[index[i]].z - dbar.z;
		Sxx += (Dcx*Mcx);
		Sxy += (Dcx*Mcy);
		Sxz += (Dcx*Mcz);
		Syx += (Dcy*Mcx);
		Syy += (Dcy*Mcy);
		Syz += (Dcy*Mcz);
		Szx += (Dcz*Mcx);
		Szy += (Dcz*Mcy);
		Szz += (Dcz*Mcz);
	}
	//printf("Sxx = %f\n", Sxx);

	/* Set P */
	CvScalar s;

	s.val[0] = Sxx + Syy + Szz;//����P��ֵ��P�����ã���
	cvSet2D(P, 0, 0, s);
	s.val[0] = Syz - Szy;
	cvSet2D(P, 0, 1, s);
	s.val[0] = Szx - Sxz;
	cvSet2D(P, 0, 2, s);
	s.val[0] = Sxy - Syx;
	cvSet2D(P, 0, 3, s);
	s.val[0] = Syz - Szy;
	cvSet2D(P, 1, 0, s);
	s.val[0] = Sxx - Syy - Szz;
	cvSet2D(P, 1, 1, s);
	s.val[0] = Sxy + Syx;
	cvSet2D(P, 1, 2, s);
	s.val[0] = Szx + Sxz;
	cvSet2D(P, 1, 3, s);
	s.val[0] = Szx - Sxz;
	cvSet2D(P, 2, 0, s);
	s.val[0] = Sxy + Syx;
	cvSet2D(P, 2, 1, s);
	s.val[0] = Syy - Sxx - Szz;
	cvSet2D(P, 2, 2, s);
	s.val[0] = Syz + Szy;
	cvSet2D(P, 2, 3, s);
	s.val[0] = Sxy - Syx;
	cvSet2D(P, 3, 0, s);
	s.val[0] = Szx + Sxz;
	cvSet2D(P, 3, 1, s);
	s.val[0] = Syz + Szy;
	cvSet2D(P, 3, 2, s);
	s.val[0] = Szz - Sxx - Syy;
	cvSet2D(P, 3, 3, s);
	//PrintMatrix(P, P->rows, P->cols);

	CvMat *EigenValue_Row=cvCreateMat(4,1,CV_32FC1);//P������ֵ
	CvMat *EigenVector=cvCreateMat(4,4,CV_32FC1);//��Ӧ����������
	cvEigenVV(P, EigenVector, EigenValue_Row, DBL_EPSILON);
	//PrintMatrix(P, P->rows, P->cols);
	//PrintMatrix(EigenValue_Row, EigenValue_Row->rows, EigenValue_Row->cols);
	//PrintMatrix(EigenVector, EigenVector->rows, EigenVector->cols);

	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	cvSetData(R, Array, R->step);

	float max_eigenvalue = cvGet2D(EigenValue_Row,0,0).val[0];
	int max_eigenvalue_index = 0;
	for (i=0; i<4; i++)     
	{
		if (cvGet2D(EigenValue_Row,i,0).val[0] > max_eigenvalue) 
		{
			max_eigenvalue = cvGet2D(EigenValue_Row,i,0).val[0];
			max_eigenvalue_index = i;
		}
	}
	//printf("max_eigenvalue = %f\n", max_eigenvalue);
	//printf("max_eigenvalue_index = %d\n", max_eigenvalue_index);
	Transformation RT;		//RT.R��ֵ��P����������ֵ��Ӧ����������
	RT.R.q0 = cvGet2D(EigenVector,max_eigenvalue_index,0).val[0];
	RT.R.q1 = cvGet2D(EigenVector,max_eigenvalue_index,1).val[0];
	RT.R.q2 = cvGet2D(EigenVector,max_eigenvalue_index,2).val[0];
	RT.R.q3 = cvGet2D(EigenVector,max_eigenvalue_index,3).val[0];
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	printf("Optimal Rotation (%f,%f,%f,%f)\n", RT.R.q0, RT.R.q1, RT.R.q2, RT.R.q3);
	#endif
	SetRotationMatrix(R, RT.R);
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintMatrix(R, R->rows, R->cols);
	#endif
	CvMat *T = cvCreateMat(3,1,CV_32FC1);
	CvMat *mbarMatrix = cvCreateMat(3,1,CV_32FC1);
	s.val[0] = mbar.x;
	cvSet2D(mbarMatrix, 0, 0, s);
	s.val[0] = mbar.y;
	cvSet2D(mbarMatrix, 1, 0, s);
	s.val[0] = mbar.z;
	cvSet2D(mbarMatrix, 2, 0, s);
	CvMat *dbarMatrix = cvCreateMat(3,1,CV_32FC1);
	s.val[0] = dbar.x;
	cvSet2D(dbarMatrix, 0, 0, s);
	s.val[0] = dbar.y;
	cvSet2D(dbarMatrix, 1, 0, s);
	s.val[0] = dbar.z;
	cvSet2D(dbarMatrix, 2, 0, s);
	CvMat *Matrix1 = cvCreateMat(3,1,CV_32FC1);
	cvmMul(R, dbarMatrix, Matrix1);//Matrix1=R*dbarMatrix
	CvMat *Matrix2 = cvCreateMat(3,1,CV_32FC1);
	cvmSub(mbarMatrix, Matrix1, Matrix2);//Matrix2=R*dbarMatrix+mbarMatrix
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintMatrix(Matrix2, Matrix2->rows, Matrix2->cols);
	#endif
	RT.T.t0 = cvGet2D(Matrix2, 0, 0).val[0];//T <-- Matrix2=R*dbarMatrix+mbarMatrix
	RT.T.t1 = cvGet2D(Matrix2, 1, 0).val[0];
	RT.T.t2 = cvGet2D(Matrix2, 2, 0).val[0];

	cvReleaseMat(&Matrix2);
	cvReleaseMat(&Matrix1);
	cvReleaseMat(&mbarMatrix);
	cvReleaseMat(&dbarMatrix);
	cvReleaseMat(&EigenValue_Row);
	cvReleaseMat(&EigenVector);
	cvReleaseMat(&T);
	cvReleaseMat(&P);
	cvReleaseMat(&R);

	return RT;
}

void TransformPoint3DSet(Transformation RT, Point3DSet *pset, Point3DSet *yset) 
{
	//Point3DSet *yset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//yset->number = pset->number;
	//yset->point =(Point3D*) malloc((pset->number)*sizeof(Point3D));
	//yset->invalid_point_number = 0;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *T = cvCreateMat(3,1,CV_32FC1);

	SetRotationMatrix(R, RT.R);//������ת����R
	CvScalar s;
	s.val[0] = RT.T.t0;
	cvSet2D(T, 0, 0, s);
	s.val[0] = RT.T.t1;
	cvSet2D(T, 1, 0, s);
	s.val[0] = RT.T.t2;
	cvSet2D(T, 2, 0, s);//����ƽ������T
//	PrintMatrix(R, R->rows, R->cols);
//	PrintMatrix(T, T->rows, T->cols);

	CvMat *point = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix1 = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix2 = cvCreateMat(3,1,CV_32FC1);
	int i;

	for (i=0; i < pset->number; i++) 
	{
		s.val[0] = pset->point[i].x;
		cvSet2D(point, 0, 0, s);
		s.val[0] = pset->point[i].y;
		cvSet2D(point, 1, 0, s);
		s.val[0] = pset->point[i].z;
		cvSet2D(point, 2, 0, s);
		cvmMul(R, point, Matrix1);//����˷�:Matrix1=R*point
		cvmAdd(Matrix1, T, Matrix2);//����ӷ�:Matrix2=R*point+T,��point�����任��ĵ�
		yset->point[i].x = cvGet2D(Matrix2, 0, 0).val[0];
		yset->point[i].y = cvGet2D(Matrix2, 1, 0).val[0];
		yset->point[i].z = cvGet2D(Matrix2, 2, 0).val[0];//�任��ĵ����pset
		yset->point[i].r = pset->point[i].r;
		yset->point[i].g = pset->point[i].g;
		yset->point[i].b = pset->point[i].b;
	}
	yset->number=pset->number;

			float r31,r11,r21,r32,r33;
			r31=CV_MAT_ELEM(*R,float,2,0);
			r11=CV_MAT_ELEM(*R,float,0,0);
			r21=CV_MAT_ELEM(*R,float,1,0);	
			r32=CV_MAT_ELEM(*R,float,2,1);
			r33=CV_MAT_ELEM(*R,float,2,2);
			beta=atan2(-r31,sqrt(r11*r11+r21*r21));
			alpha=atan2(r21/cos(beta),r11/cos(beta));
			gamma=atan2(r32/cos(beta),r33/cos(beta));
	

	cvReleaseMat(&Matrix2);
	cvReleaseMat(&Matrix1);
	cvReleaseMat(&point);
	cvReleaseMat(&T);
	cvReleaseMat(&R);
	//return yset;
}

Transformation updatetransform(Transformation t1,Transformation t2)
{
	//transform=t1*t2���ٸ���t1
	Transformation transform;
	transform.R.q0=t1.R.q0*t2.R.q0-t1.R.q1*t2.R.q1-t1.R.q2*t2.R.q2-t1.R.q3*t2.R.q3;
	transform.R.q1=t1.R.q0*t2.R.q1+t1.R.q1*t2.R.q0+t1.R.q2*t2.R.q3-t1.R.q3*t2.R.q2;
	transform.R.q2=t1.R.q0*t2.R.q2-t1.R.q1*t2.R.q3+t1.R.q2*t2.R.q0+t1.R.q3*t2.R.q1;
	transform.R.q3=t1.R.q0*t2.R.q3+t1.R.q1*t2.R.q2-t1.R.q2*t2.R.q1+t1.R.q3*t2.R.q0;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	SetRotationMatrix(R,t1.R);
	transform.T.t0=t2.T.t0*CV_MAT_ELEM(*R,float,0,0)+t2.T.t1*CV_MAT_ELEM(*R,float,0,1)+t2.T.t2*CV_MAT_ELEM(*R,float,0,2)+t1.T.t0;
	transform.T.t1=t2.T.t0*CV_MAT_ELEM(*R,float,1,0)+t2.T.t1*CV_MAT_ELEM(*R,float,1,1)+t2.T.t2*CV_MAT_ELEM(*R,float,1,2)+t1.T.t1;
	transform.T.t2=t2.T.t0*CV_MAT_ELEM(*R,float,2,0)+t2.T.t1*CV_MAT_ELEM(*R,float,2,1)+t2.T.t2*CV_MAT_ELEM(*R,float,2,2)+t1.T.t2;
	//transform.T.t0=t1.T.t0+t2.T.t0;
	//transform.T.t1=t1.T.t1+t2.T.t1;
	//transform.T.t2=t1.T.t2+t2.T.t2;
	return transform;
}

Transformation updatetransform_qr(Transformation t1,Transformation t2)
{
	//transform=t1*t2���ٸ���t1
	Transformation transform;
	transform.R.q0=t1.R.q0*t2.R.q0-t1.R.q1*t2.R.q1-t1.R.q2*t2.R.q2-t1.R.q3*t2.R.q3;
	transform.R.q1=t1.R.q0*t2.R.q1+t1.R.q1*t2.R.q0+t1.R.q2*t2.R.q3-t1.R.q3*t2.R.q2;
	transform.R.q2=t1.R.q0*t2.R.q2-t1.R.q1*t2.R.q3+t1.R.q2*t2.R.q0+t1.R.q3*t2.R.q1;
	transform.R.q3=t1.R.q0*t2.R.q3+t1.R.q1*t2.R.q2-t1.R.q2*t2.R.q1+t1.R.q3*t2.R.q0;
	transform.T.t0=t1.T.t0+t2.T.t0;
	transform.T.t1=t1.T.t1+t2.T.t1;
	transform.T.t2=t1.T.t2+t2.T.t2;
	return transform;
}

void TransformUsingH(CvMat *R, CvMat *T, Point3DSet *pset) 
{
	CvMat *point = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix1 = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix2 = cvCreateMat(3,1,CV_32FC1);
	int i;
	CvScalar s;
	for (i=0; i < pset->number; i++) 
	{
		s.val[0] = pset->point[i].x;
		cvSet2D(point, 0, 0, s);
		s.val[0] = pset->point[i].y;
		cvSet2D(point, 1, 0, s);
		s.val[0] = pset->point[i].z;
		cvSet2D(point, 2, 0, s);
		cvmMul(R, point, Matrix1);//����˷�:Matrix1=R*point
		cvmAdd(Matrix1, T, Matrix2);//����ӷ�:Matrix2=R*point+T,��point�����任��ĵ�
		pset->point[i].x = cvGet2D(Matrix2, 0, 0).val[0];
		pset->point[i].y = cvGet2D(Matrix2, 1, 0).val[0];
		pset->point[i].z = cvGet2D(Matrix2, 2, 0).val[0];//�任��ĵ����pset
	}

	cvReleaseMat(&Matrix2);
	cvReleaseMat(&Matrix1);
	cvReleaseMat(&point);
}


Transformation Matrix2Quat(CvMat *R,CvMat *T)
{
	Transformation RT;
	float s;
	float tq[4];
	int i,j;
	tq[0]=1+cvGet2D(R, 0, 0).val[0]+cvGet2D(R, 1, 1).val[0]+cvGet2D(R, 2, 2).val[0];
	tq[1]=1+cvGet2D(R, 0, 0).val[0]-cvGet2D(R, 1, 1).val[0]-cvGet2D(R, 2, 2).val[0];
	tq[2]=1-cvGet2D(R, 0, 0).val[0]+cvGet2D(R, 1, 1).val[0]-cvGet2D(R, 2, 2).val[0];
	tq[3]=1-cvGet2D(R, 0, 0).val[0]-cvGet2D(R, 1, 1).val[0]+cvGet2D(R, 2, 2).val[0];
	j=0;
	for(i=1;i<4;i++)
		j=(tq[i]>tq[j])?i:j;
	if(j==0)
	{
		RT.R.q0=tq[0];
		RT.R.q1=-cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q2=-cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q3=-cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
	}
	else if(j==1)
	{
		RT.R.q0=-cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q1=tq[1];
		RT.R.q2=cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q3=cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
	}
	else if(j==2)
	{
		RT.R.q0=-cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q1=cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q2=tq[2];
		RT.R.q3=cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
	}
	else
	{
		RT.R.q0=-cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q1=cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q2=cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q3=tq[3];
	}
	s=sqrt(0.25/tq[j]);
	//cout<<"j: "<<j<<endl;
	RT.R.q0*=s;
	RT.R.q1*=s;
	RT.R.q2*=s;
	RT.R.q3*=s;
	RT.T.t0=cvGet2D(T, 0, 0).val[0];
	RT.T.t1=cvGet2D(T, 1, 0).val[0];
	RT.T.t2=cvGet2D(T, 2, 0).val[0];
	return RT;
}



void icp(Point3DSet *data_set, Point3DSet *model_set,int QR_flag) 
{
	//ofstream fp;
	//fp.open("trjectory.txt",ios::app);

	clock_t start,end;	
	Point3DSet *yset;
	Transformation RT,RT0;
	sel_data_set_k =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_data_set_k->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_data_set_k->invalid_point_number = 0;
	yset =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	yset->point =(Point3D*) malloc(19200*sizeof(Point3D));
	yset->invalid_point_number = 0;
	sel_data_set =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_data_set->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_data_set->invalid_point_number = 0;
	sel_model_set =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_model_set->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_model_set->invalid_point_number = 0;
	//cout<<"here"<<endl;
	//Selection(data_set,sel_data_set);
	//Selection(model_set,sel_model_set);
	
	int inliers;
	start=clock();
	inliers=estimate();
	end=clock();
	//cout<<"inliers:"<<inliers<<endl;
	//fp<<"estimate time:"<<end-start<<endl;
	//cout<<endl<<"estimate time:"<<end-start<<endl<<endl;

	CvMat *rot= cvCreateMat(3,3,CV_32FC1);
	CvMat *rott= cvCreateMat(3,3,CV_32FC1);
	int trancount = 0;
	float dms,dms_tmp,dms_avr=0,dms_avr_pre=10;
	int num;
	RT0.R.q0=1;
	RT0.R.q1=0;
	RT0.R.q2=0;
	RT0.R.q3=0;
	RT0.T.t0=0;
	RT0.T.t1=0;
	RT0.T.t2=0;
	CvMat R=cvMat(3,3,CV_32FC1);
	CvMat T=cvMat(3,1,CV_32FC1);
	CvMat RR=cvMat(3,3,CV_32FC1);
	CvMat TT=cvMat(3,1,CV_32FC1);




	start=clock();
	//if(flag_qr==1)
	//{
	//	transqr_tmp=Matrix2Quat(&Rqr,&Tqr);
	//	transqr=updatetransform_qr(transqr,transqr_tmp);
	//	trans=transqr;
	//	//fp<<"corrected:"<<endl;
	//	//fp<<"trans:R:"<<trans.R.q0<<","<<trans.R.q1<<","<<trans.R.q2<<","<<trans.R.q3<<",T:"<<trans.T.t0<<","<<trans.T.t1<<","<<trans.T.t2<<endl;
	//	TransformPoint3DSet(transqr,data_set,data_set);
	//	SetRotationMatrix(rott, transqr.R);
	//	pose.orgx=transqr.T.t0;
	//	pose.orgy=transqr.T.t1;
	//	pose.orgz=transqr.T.t2;
	//	pose.x1=CV_MAT_ELEM(*rott,float,0,0);
	//	pose.x2=CV_MAT_ELEM(*rott,float,1,0);
	//	pose.x3=CV_MAT_ELEM(*rott,float,2,0);
	//	pose.y1=CV_MAT_ELEM(*rott,float,0,1);
	//	pose.y2=CV_MAT_ELEM(*rott,float,1,1);
	//	pose.y3=CV_MAT_ELEM(*rott,float,2,1);
	//	pose.z1=CV_MAT_ELEM(*rott,float,0,2);
	//	pose.z2=CV_MAT_ELEM(*rott,float,1,2);
	//	pose.z3=CV_MAT_ELEM(*rott,float,2,2);
	//	//fp<<"rotaion matrix:"<<endl<<
	//	//	CV_MAT_ELEM(*rott,float,0,0)<<","<<CV_MAT_ELEM(*rott,float,1,0)<<","<<CV_MAT_ELEM(*rott,float,2,0)<<","<<endl<<
	//	//	CV_MAT_ELEM(*rott,float,0,1)<<","<<CV_MAT_ELEM(*rott,float,1,1)<<","<<CV_MAT_ELEM(*rott,float,2,1)<<","<<endl<<
	//	//	CV_MAT_ELEM(*rott,float,0,2)<<","<<CV_MAT_ELEM(*rott,float,1,2)<<","<<CV_MAT_ELEM(*rott,float,2,2)<<endl<<endl<<endl<<endl;
	//	traj.push_back(pose);
	//	pointcloud.push_back(*data_set);
	//	//cvmMul(&Rqr_pre,&Rqr,&RR);
	//	//cvmAdd(&Tqr_pre,&Tqr,&TT);
	//	//TransformUsingH(&RR,&TT,data_set);
	//	//pointcloud.push_back(*data_set);
	//}
	//else
	{
		/////////////////////using feature points//////////////////////////////////////////
		//if(inliers>300)
		//{
		//	R=Rf;
		//	T=Tf;
		//}
		//else
		//{
		//	fp<<"*******************************************"<<endl;
		//	R=Rf_pre;
		//	T=Tf_pre;
		//}
		//Rf_pre=Rf.clone();
		//Tf_pre=Tf.clone();
		//////////////////////////////////////////////////////////////////////////////////

		//if(QR_flag)
		{
			R=Rf;
			T=Tf;
		}
		//else
		//{
		//	R=Rqr;
		//	T=Tqr;
		//}


		RT0=Matrix2Quat(&R,&T);
		//TransformPoint3DSet(RT0,sel_data_set,sel_data_set);
		TransformPoint3DSet(RT0,data_set,sel_data_set);
		//fp<<"inliers:"<<inliers<<endl;
		//fp<<"RT_esti:R:"<<RT0.R.q0<<","<<RT0.R.q1<<","<<RT0.R.q2<<","<<RT0.R.q3<<",T:"<<RT0.T.t0<<","<<RT0.T.t1<<","<<RT0.T.t2<<endl;
			
	
		trans=updatetransform(trans,RT0);
		//fp<<"trans: R:"<<trans.R.q0<<","<<trans.R.q1<<","<<trans.R.q2<<","<<trans.R.q3<<",T:"<<trans.T.t0<<","<<trans.T.t1<<","<<trans.T.t2<<endl;
	
		//SetRotationMatrix(rot, trans.R);
		//fp<<"rotaion matrix:"<<endl<<
		//	CV_MAT_ELEM(*rot,float,0,0)<<","<<CV_MAT_ELEM(*rot,float,1,0)<<","<<CV_MAT_ELEM(*rot,float,2,0)<<","<<endl<<
		//	CV_MAT_ELEM(*rot,float,0,1)<<","<<CV_MAT_ELEM(*rot,float,1,1)<<","<<CV_MAT_ELEM(*rot,float,2,1)<<","<<endl<<
		//	CV_MAT_ELEM(*rot,float,0,2)<<","<<CV_MAT_ELEM(*rot,float,1,2)<<","<<CV_MAT_ELEM(*rot,float,2,2)<<endl<<endl;

		TransformPoint3DSet(RT0,data_set,sel_data_set_k);

		//FILE* distance;
		//distance=fopen("distance.txt","wt");
		while(fabs(dms_avr-dms_avr_pre)>1&&trancount<10)//(trancount<60)//
		{
			Matching(sel_data_set_k,model_set,yset);
			RT=GetOptimalRotation(sel_data_set,yset);
			cout<<"RT:R:"<<RT.R.q0<<","<<RT.R.q1<<","<<RT.R.q2<<","<<RT.R.q3<<",T:"<<RT.T.t0<<","<<RT.T.t1<<","<<RT.T.t2<<endl;
			TransformPoint3DSet(RT,sel_data_set,sel_data_set_k);
			dms=0;
			num=0;
			for(int i=0;i<yset->number;i++)
			{
				dms_tmp=sqrt((sel_data_set_k->point[index[i]].x-yset->point[i].x)*(sel_data_set_k->point[index[i]].x-yset->point[i].x)
							+(sel_data_set_k->point[index[i]].y-yset->point[i].y)*(sel_data_set_k->point[index[i]].y-yset->point[i].y)
							+(sel_data_set_k->point[index[i]].z-yset->point[i].z)*(sel_data_set_k->point[index[i]].z-yset->point[i].z));
							//+((sel_data_set_k->point[index[i]].x-yset->point[i].x)*(sel_data_set_k->point[index[i]].x-yset->point[i].x)+(sel_data_set_k->point[index[i]].y-yset->point[i].y)*(sel_data_set_k->point[index[i]].y-yset->point[i].y))*alpha
							//+((sel_data_set_k->point[index[i]].x-yset->point[i].x)*(sel_data_set_k->point[index[i]].x-yset->point[i].x)+(sel_data_set_k->point[index[i]].z-yset->point[i].z)*(sel_data_set_k->point[index[i]].z-yset->point[i].z))*beta
							//+((sel_data_set_k->point[index[i]].y-yset->point[i].y)*(sel_data_set_k->point[index[i]].y-yset->point[i].y)+(sel_data_set_k->point[index[i]].z-yset->point[i].z)*(sel_data_set_k->point[index[i]].z-yset->point[i].z))*gamma);
				dms+=dms_tmp;
				num++;
			}	
			dms_avr_pre=dms_avr;
			dms_avr=dms/num;
			//fprintf(distance,"%f\n\n",dms_avr);
			//cout<<"dms_avr:"<<dms_avr<<endl;
			trancount++;
			//ReleasePoint3DSet(yset);
			//ReleasePoint3DSet(sel_data_set_k);
		}
		//cout<<endl;
		//fclose(distance);

		//if(dms_avr<30)
		{
			//fp<<"icp"<<endl;
			trans=updatetransform(trans,RT);
			//fp<<"RT_ICP:R:"<<RT.R.q0<<","<<RT.R.q1<<","<<RT.R.q2<<","<<RT.R.q3<<",T:"<<RT.T.t0<<","<<RT.T.t1<<","<<RT.T.t2<<endl;
			//fp<<"trans:R:"<<trans.R.q0<<","<<trans.R.q1<<","<<trans.R.q2<<","<<trans.R.q3<<",T:"<<trans.T.t0<<","<<trans.T.t1<<","<<trans.T.t2<<endl;
			SetRotationMatrix(rot, trans.R);
			//fp<<"rotaion matrix:"<<endl<<
			//	CV_MAT_ELEM(*rot,float,0,0)<<","<<CV_MAT_ELEM(*rot,float,1,0)<<","<<CV_MAT_ELEM(*rot,float,2,0)<<","<<endl<<
			//	CV_MAT_ELEM(*rot,float,0,1)<<","<<CV_MAT_ELEM(*rot,float,1,1)<<","<<CV_MAT_ELEM(*rot,float,2,1)<<","<<endl<<
			//	CV_MAT_ELEM(*rot,float,0,2)<<","<<CV_MAT_ELEM(*rot,float,1,2)<<","<<CV_MAT_ELEM(*rot,float,2,2)<<endl;
		}


		TransformPoint3DSet(trans,data_set,data_set);
		pose.orgx=trans.T.t0;
		pose.orgy=trans.T.t1;
		pose.orgz=trans.T.t2;
		pose.x1=CV_MAT_ELEM(*rot,float,0,0);
		pose.x2=CV_MAT_ELEM(*rot,float,1,0);
		pose.x3=CV_MAT_ELEM(*rot,float,2,0);
		pose.y1=CV_MAT_ELEM(*rot,float,0,1);
		pose.y2=CV_MAT_ELEM(*rot,float,1,1);
		pose.y3=CV_MAT_ELEM(*rot,float,2,1);
		pose.z1=CV_MAT_ELEM(*rot,float,0,2);
		pose.z2=CV_MAT_ELEM(*rot,float,1,2);
		pose.z3=CV_MAT_ELEM(*rot,float,2,2);
		//cout<<"inliers:"<<inliers<<endl<<endl;
		//if(inliers>300)
		{
			traj.push_back(pose);
			pointcloud.push_back(*data_set);
		}
		
		
	}
	end=clock();
	//fp<<"icp time:"<<end-start<<endl<<endl<<endl<<endl;
	//cout<<endl<<"icp time:"<<end-start<<endl<<endl;
	//cout<<"traj:"<<traj.size()<<endl;
	//cout<<"pointcloud:"<<pointcloud.size()<<endl<<endl;

	//fp.close();


	glutDisplayFunc(renderScene1); 
	ReleasePoint3DSet(sel_data_set);
	ReleasePoint3DSet(sel_model_set);
	ReleasePoint3DSet(yset);
	ReleasePoint3DSet(sel_data_set_k);
	ReleasePoint3DSet(model_set);

}