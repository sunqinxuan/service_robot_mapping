#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>

// Link with ws2_32.lib
#pragma comment(lib, "ws2_32.lib")


#include "icp.h"
#include "estimate.h"
#include "capture.h"

IplImage pImg1,pImg2;
Mat img1,img2;
extern Mat cImageBGR;
extern float xyzdata[480][640][3];
extern float xyzdata2[480][640][3];
extern uchar texture[480][640][3];
extern uchar texture2[480][640][3];

int num;
cv::Mat R_qr,T_qr;
CvMat Rqr,Tqr;


bool flag_send=false;
int QR_flag = 0;
point curpos;
char recvbuf[12];
bool flag_endloop=false;



DWORD WINAPI sendpoint3d(LPVOID lpParam)
{
	unsigned char* xyz=(unsigned char*) lpParam;

	int iResult;
    WSADATA wsaData;
    SOCKET SendSocket = INVALID_SOCKET;
    SOCKADDR_IN RecvAddr;
    unsigned short Port = 8000;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) 
	{
        wprintf(L"WSAStartup failed with error: %d\n", iResult);
        return 1;
    }
    // Create a socket for sending data
    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (SendSocket == INVALID_SOCKET) 
	{
        wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_port = htons(Port);
    RecvAddr.sin_addr.s_addr = inet_addr("192.168.2.6");
	cout<<"socket initialized"<<endl;

	int buflen=1000;
    char SendBuf[1000];
	char RecvBuf[10];
	int RecvAddrSize=sizeof(RecvAddr);
	char *buf=SendBuf;
	//char *ptr;
	int len=sizeof(float);
	int numt=1;
	//char *ptr;
	while(true)
	{
		Sleep(1);
		if(flag_send)
		{
			cout<<"flag in the thread:"<<flag_send<<endl;
		}
		
		if(flag_send)
		{
			//cout<<numt<<":ready to send"<<endl;
			//sprintf(SendBuf,"No.%d:send sth,for test",numt);
			//buflen=sizeof(SendBuf);
			//iResult = sendto(SendSocket,SendBuf, buflen, 0, (SOCKADDR *) &RecvAddr, sizeof (RecvAddr));
			//if (iResult == SOCKET_ERROR) 
			//{
			//	wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
			//}
			//iResult = recvfrom(SendSocket,RecvBuf, 10, 0, (SOCKADDR *) & RecvAddr, &RecvAddrSize);
			//if (iResult == SOCKET_ERROR) 
			//{
			//	wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
			//}
			//cout<<numt<<":received:"<<RecvBuf<<endl;
			//ZeroMemory(RecvBuf,10);
			//numt++;
			//Sleep(1000);
			cout<<numt<<":ready to send"<<endl;
			clock_t t_start,t_end;  
			t_start = clock();
			//cout<<"here"<<endl;
			unsigned char *ptr=xyz;
			//cout<<"here before copy"<<endl;
			//for(int i=0;i<pPointSet->number;i++)
			//{
			//	memcpy(buf,ptr,len);
			//	memcpy(buf+len,ptr+len,len);
			//	memcpy(buf+len*2,ptr+len*2,len);
			//	memcpy(buf+len*3,ptr+len*3,1);
			//	memcpy(buf+len*3+1,ptr+len*2+1,1);
			//	memcpy(buf+len*3+2,ptr+len*2+2,1);
			//	buf+=len*3+3;
			//	ptr+=len*3+3;
			//}
			for(int i=0;i<180;i++)
			{
				for(int j=0;j<1000;j++)
				{
					buf[j]=ptr[j];
				}
				//cout<<i<<":before send"<<endl;
				iResult = sendto(SendSocket,SendBuf, buflen, 0, (SOCKADDR *) &RecvAddr, sizeof (RecvAddr));
				if (iResult == SOCKET_ERROR) 
				{
					wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
				}
				ptr+=1000;
				//cout<<i<<":after send"<<endl;
				iResult = recvfrom(SendSocket,RecvBuf, 10, 0, (SOCKADDR *) & RecvAddr, &RecvAddrSize);
				if (iResult == SOCKET_ERROR) 
				{
					wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
				}
				//cout<<i<<":received"<<endl;
			}
			cout<<"a frame of points sent"<<endl;
			flag_send=false;
			t_end = clock() ;
			printf( "a frame of points sent time: %f ms\n",(double)(t_end-t_start) );
		}		
	}

	iResult = closesocket(SendSocket);
    if (iResult == SOCKET_ERROR) 
	{
        wprintf(L"closesocket failed with error: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
	//---------------------------------------------
    // Clean up and quit.
    wprintf(L"Exiting.\n");
    WSACleanup();
    return 0;
}


DWORD WINAPI getpos(LPVOID lpParam)
{
	//char* xyz=(char*)lpParam;
	//Point* pPoint=(Point*) lpParam;
	int iResult = 0;
    WSADATA wsaData;

    SOCKET RecvSocket;
    sockaddr_in RecvAddr;

    unsigned short Port = 9006;


    sockaddr_in SenderAddr;
    int SenderAddrSize = sizeof (SenderAddr);

    //-----------------------------------------------
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) 
	{
        wprintf(L"WSAStartup failed with error %d\n", iResult);
        return 1;
    }
    //-----------------------------------------------
    // Create a receiver socket to receive datagrams
    RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (RecvSocket == INVALID_SOCKET) 
	{
        wprintf(L"socket failed with error %d\n", WSAGetLastError());
        return 1;
    }
    //-----------------------------------------------
    // Bind the socket to any address and the specified port.
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_port = htons(Port);
    RecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    iResult = bind(RecvSocket, (SOCKADDR *) & RecvAddr, sizeof (RecvAddr));
    if (iResult != 0) 
	{
        wprintf(L"bind failed with error %d\n", WSAGetLastError());
        return 1;
    }
	//float x,y,z; 
	//char* tempBuf=(char*)RecvBuf;
	//pPoint->count=0;

    int buflen = 12;
	int num=1;
	clock_t start,end;
	while(true)
	{
		//cout<<"ready to receive curpos"<<endl;
		//start=clock();
		iResult = recvfrom(RecvSocket,recvbuf, buflen, 0, (SOCKADDR *) & SenderAddr, &SenderAddrSize);
		//if (iResult == 1)
		//{
		//	QR_flag=1;
		//}
		if (iResult == SOCKET_ERROR) 
		{
			//QR_flag=1;
			wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
		}
		//end=clock();
		memcpy(&curpos.x,recvbuf,4);
		memcpy(&curpos.y,recvbuf+4,4);
		memcpy(&curpos.ang,recvbuf+8,4);
		curpos.x*=1000;
		curpos.y*=1000;
		//cout<<curpos.x<<","<<curpos.y<<","<<curpos.ang<<endl;
	}
 
    //-----------------------------------------------
    // Close the socket when finished receiving datagrams
    wprintf(L"Finished receiving. Closing socket.\n");
    iResult = closesocket(RecvSocket);
    if (iResult == SOCKET_ERROR) {
        wprintf(L"closesocket failed with error %d\n", WSAGetLastError());
        return 1;
    }

    //-----------------------------------------------
    // Clean up and exit.
    wprintf(L"Exiting.\n");
    WSACleanup();
	return 0;
}




int main(int argc, char *argv[])
{
    DWORD dwThreadId_point,dwThreadId_pos;
    HANDLE hThread_point,hThread_pos; 

	initial();
	Point3DSet *data_set, *model_set;
	//PointSet *pPointSet=new PointSet();
	//pPointSet->flag=false;
	unsigned char xyz[280000];
	//unsigned char bufcom[280000]={0};
	//unsigned long bufcomlen=sizeof(bufcom);
	//unsigned char dstbuf[280000]={0};
	//unsigned long dstbuflen=sizeof(dstbuf);

	hThread_point = CreateThread(NULL,0,sendpoint3d,xyz,0,&dwThreadId_point); 
	//hThread_pos = CreateThread(NULL,0,getpos,recvbuf,0,&dwThreadId_pos); 
	cout<<"thread ready"<<endl;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(10,320);
	glutInitWindowSize(640, 480);
	glutCreateWindow("3D image");

	//FILE* time;
	//time=fopen("time.txt","wt");
	clock_t start,end;
	int zhen;
	extern vector<Point3D> display;
	extern vector<Pose> traj;
	extern vector<Point3DSet> pointcloud;
	extern Transformation trans;
	extern Pose pose;

	//FILE *qr_fp;
	//qr_fp=fopen("qr_num.txt","r");
	//int qr_num[31];
	//for(int i=0;i<31;i++)
	//{
	//	fscanf(qr_fp,"%d",&qr_num[i]);
	//	//cout<<qr_num[i]<<",";
	//}
	////cout<<endl;
	//fclose(qr_fp);
	//qr_fp=fopen("qr.txt","r");
	//float qr[31][3];
	//for(int i=0;i<31;i++)
	//{
	//	for(int j=0;j<3;j++)
	//	{
	//		fscanf(qr_fp,"%f",&qr[i][j]);
	//		//cout<<qr[i][j]<<" ";
	//	}
	//	//cout<<endl;
	//}
	R_qr=Mat(3,3,CV_32FC1);
	T_qr=Mat(3,1,CV_32FC1);
	


	
	point prepos;
	double firstangle;
	//firstangle=curpos.ang;
	//prepos=curpos;

	//////////////////////////////////////////////////////初变换版本////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	zhen=1;
	while(zhen<500)		
	{
		//while(pPointSet->flag);
		cout<<endl<<"now in loop:"<<zhen<<endl;
		//while(flag_send);
		//cout<<"QR_flag:"<<QR_flag<<endl;

		//QR_flag=0;
		if(flag_endloop)
			break;
		if(zhen>1)
		{
			for(int i=0;i<icp_height;i++)
			{
				for(int j=0;j<icp_width;j++)
				{	
					xyzdata2[i][j][0]=xyzdata[i][j][0];
					xyzdata2[i][j][1]=xyzdata[i][j][1];
					xyzdata2[i][j][2]=xyzdata[i][j][2];
					texture2[i][j][0]=texture[i][j][0];
					texture2[i][j][1]=texture[i][j][1];
					texture2[i][j][2]=texture[i][j][2];
				}
			}
			img2=img1.clone();
			if(abs(curpos.x-prepos.x)>500||abs(curpos.y-prepos.y)>500||abs(curpos.ang-prepos.ang)>0.2)
			{
				R_qr.at<float>(0,0)=cos(curpos.ang-prepos.ang);
				R_qr.at<float>(0,1)=0;
				R_qr.at<float>(0,2)=-sin(curpos.ang-prepos.ang);
				R_qr.at<float>(1,0)=0;
				R_qr.at<float>(1,1)=1;
				R_qr.at<float>(1,2)=0;
				R_qr.at<float>(2,0)=sin(curpos.ang-prepos.ang);
				R_qr.at<float>(2,1)=0;
				R_qr.at<float>(2,2)=cos(curpos.ang-prepos.ang);
				T_qr.at<float>(0,0)=(curpos.x-prepos.x)*cos(firstangle-CV_PI/2)+(curpos.y-prepos.y)*sin(firstangle-CV_PI/2);//qr[i-1][0]-qr[i][0];//qr[i-1][1]-qr[i][1];//
				T_qr.at<float>(1,0)=0;
				T_qr.at<float>(2,0)=-(curpos.x-prepos.x)*sin(firstangle-CV_PI/2)+(curpos.y-prepos.y)*cos(firstangle-CV_PI/2);//qr[i-1][1]-qr[i][1];//qr[i][0]-qr[i-1][0];//
				Rqr=R_qr;
				Tqr=T_qr;
				prepos=curpos;
				QR_flag=1;
				//break;
			}

		}		
		capture();
		cout<<"capture done"<<endl;
		img1=cImageBGR.clone();
		
		if(zhen==1)
		{
			zhen++;
			firstangle=curpos.ang;
			prepos=curpos;
			continue;
		}
		//cout<<"before loading "<<endl;
		data_set=loadDataSet();//从数组中读入点集
		model_set=loadModelSet();
		cout<<"after loading: "<<data_set->number<<","<<model_set->number<<endl;
		start=clock();
		icp(data_set, model_set,QR_flag);
		//cout<<"data_set:"<<data_set->number<<endl;
		//Sleep(3000);
		end=clock();
		//fprintf(time,"%f\n\n",(double)(end-start));
		cout<<"icp done:"<<end-start<<endl;
		unsigned char *buf=xyz;
		int len=sizeof(float);
		short temp;
		memcpy(buf,&data_set->number,4);
		cout<<"data_set->number:"<<data_set->number<<endl;
		//int testnum;
		//memcpy(&testnum,buf,len);
		//cout<<"testnum"<<testnum<<endl;
		buf+=len;
		for(int i=0;i<data_set->number;i++)
		{
			temp=(short)data_set->point[i].x;
			memcpy(buf,&temp,2);
			temp=(short)data_set->point[i].y;
			memcpy(buf+2,&temp,2);
			temp=(short)data_set->point[i].z;
			memcpy(buf+4,&temp,2);
			memcpy(buf+6,&data_set->point[i].r,1);
			memcpy(buf+7,&data_set->point[i].g,1);
			memcpy(buf+8,&data_set->point[i].b,1);
			buf+=9;
		}
		flag_send=true;
		//ReleasePoint3DSet(data_set);

		//unsigned long buflen=sizeof(buf);
		//cout<<"buflen:"<<data_set->number*15+len<<endl;
		////unsigned char bufcom[280000]={0};
		////unsigned long bufcomlen=sizeof(bufcom);
		//memset(bufcom,0,280000);
		//start=clock();
		//compress2(bufcom,&bufcomlen,xyz,data_set->number*15+len,Z_DEFAULT_COMPRESSION);
		//end=clock();
		//cout<<"compress time:"<<end-start<<endl;
		//cout<<"bufcomlen:"<<bufcomlen<<endl;
		////unsigned char dstbuf[280000]={0};
		////unsigned long dstbuflen=sizeof(dstbuf);
		//memset(dstbuf,0,280000);
		//uncompress(dstbuf,&dstbuflen,bufcom,bufcomlen);
		//cout<<"dstbuflen:"<<dstbuflen<<endl;

		//flag_send=true;
		//cout<<"is flag changed?"<<flag_send<<endl;
		zhen++;
			glutReshapeFunc (reshape);         // 窗口变化时重绘图像
			glutMouseFunc(mouse);            // 鼠标按键响应
			glutSpecialFunc(key);
			glutMotionFunc(motion);            // 鼠标移动响应
			glutPostRedisplay();                  // 刷新画面
			glutMainLoopEvent();
	}
	//fclose(time);

	FILE *pFile;
	char filename[10];	
	num=0;
	cout<<pointcloud.size()<<endl;
	short temp;
	for(vector<Point3DSet>::iterator iter=pointcloud.begin();iter!=pointcloud.end();++iter)
	{
		sprintf(filename,"%d",num);
		num++;
		pFile=fopen(filename,"wb");
		int nByteLength = (*iter).number * 9;
		unsigned char *pData = new unsigned char[nByteLength];
		int m = 0;
		for(int j = 0; j< (*iter).number; j++)
		{
			Point3D* pPoint = &((*iter).point[j]);

			//拷贝数据至数组
			temp=(short)pPoint->x;
			memcpy(pData + m, &temp,2);
			m += 2;
			temp=(short)pPoint->y;
			memcpy(pData + m, &temp,2);
			m += 2;
			temp=(short)pPoint->z;
			memcpy(pData + m, &temp,2);
			m += 2;
			memcpy(pData + m, &(pPoint->r),1);
			m++;
			memcpy(pData + m, &(pPoint->g),1);
			m++;
			memcpy(pData + m, &(pPoint->b),1);
			m++;
		}
		fwrite(pData, 1, nByteLength, pFile);
		delete pData;
		fclose(pFile);
		//delete (*iter).point;
	}
	

	while(true)
	{
		glutReshapeFunc (reshape);         // 窗口变化时重绘图像
		glutMouseFunc(mouse);            // 鼠标按键响应
		glutSpecialFunc(key);
		glutMotionFunc(motion);            // 鼠标移动响应
		glutPostRedisplay();                  // 刷新画面
		glutMainLoopEvent();
	}
	close();
	system("PAUSE");
	return 0;
}
