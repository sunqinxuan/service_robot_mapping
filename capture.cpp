#include "capture.h"
#include "icp.h"

extern float xyzdata[480][640][3];
extern float xyzdata2[480][640][3];
extern uchar texture[480][640][3];
extern uchar texture2[480][640][3];

//IplImage pImg;
//cv::Mat mImageBGR(480, 640,CV_8UC3);

VideoStream mDepthStream;
VideoStream mColorStream;
Device mDevice;	
int width=640, height=480;
clock_t t_start,t_end;  


cv::Mat cImageBGR;
extern char recvbuf[12];
extern point curpos;


void load3dDataToGL(unsigned char* xyz)
{
	unsigned char *tmp=xyz;
	for (int i=0;i<height;i++)
	{ 
		for (int j=0;j<width;j++)
		{
			memcpy(xyzdata[i][j],tmp,12);
			tmp+=12;
		}
	} 
}

void loadTextureToGL(Mat img)
{   
	uchar *pt;
	for (int i=0;i<icp_height;i++)
	{
		pt=img.ptr<uchar>(i);
		for (int j=0;j<icp_width;j++)
		{
			texture[i][j][2]=pt[j*3];
			texture[i][j][1]=pt[j*3+1];
			texture[i][j][0]=pt[j*3+2];
		}
	}      
}



int initial( void )
{	
	t_start = clock();
	// 1. Initial OpenNI
	if( OpenNI::initialize() != STATUS_OK )
	{
		cerr << "OpenNI Initial Error: " << OpenNI::getExtendedError() << endl;
		return -1;
	}
	t_end = clock() ;
	printf( "OpenNI Initial time: %f ms\n",(double)(t_end-t_start) );

  
	// 2. Open Device
	t_start=clock();
	
	if( mDevice.open( ANY_DEVICE ) != STATUS_OK )
	{
		cerr << "Can't Open Device: " << OpenNI::getExtendedError() << endl;
		return -1;
	}
	t_end = clock() ;
	printf( "open device time: %f ms\n",(double)(t_end-t_start) );

  
	// 3. Create depth stream
	t_start=clock();
	
	if( mDevice.hasSensor( SENSOR_DEPTH ) )
	{
		if( mDepthStream.create( mDevice, SENSOR_DEPTH ) == STATUS_OK )
		{
			// 3a. set video mode
			VideoMode mMode;
			mMode.setResolution( 640, 480 );
			mMode.setFps( 30 );
			mMode.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
  
			if( mDepthStream.setVideoMode( mMode) != STATUS_OK )
			{
				cout << "Can't apply VideoMode: "<< OpenNI::getExtendedError() << endl;
			}

			cout<<"depth stream created"<<endl;
		}
		else
		{
			cerr << "Can't create depth stream on device: "<< OpenNI::getExtendedError() << endl;
			return -1;
		}
	}
	else
	{
		cerr << "ERROR: This device does not have depth sensor" << endl;
		return -1;
	}
	t_end = clock() ;
	printf( "create depth stream time: %f ms\n",(double)(t_end-t_start) );

  
	// 4. Create color stream
	t_start=clock();
	
	if( mDevice.hasSensor( SENSOR_COLOR ) )
	{
		if( mColorStream.create( mDevice, SENSOR_COLOR ) == STATUS_OK )
		{
			// 4a. set video mode
			VideoMode mMode;
			mMode.setResolution( 640, 480 );
			mMode.setFps( 30 );
			mMode.setPixelFormat( PIXEL_FORMAT_RGB888 );
  
			if( mColorStream.setVideoMode( mMode) != STATUS_OK )
			{
				cout << "Can't apply VideoMode: " << OpenNI::getExtendedError() << endl;
			}
			
			cout<<"color stream created"<<endl;
  
			// 4b. image registration
			if( mDevice.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
			{
				mDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
				cout<<"registration"<<endl;
			}
		}
		else
		{
			cerr << "Can't create color stream on device: "<< OpenNI::getExtendedError() << endl;
			return -1;
		}
	}
	t_end = clock() ;
	printf( "create color stream time: %f ms\n",(double)(t_end-t_start) );
	return 0;
}

int close( void )
{
	// 9. stop
	mDepthStream.destroy();
	mColorStream.destroy();
	mDevice.close();
	OpenNI::shutdown();  
	return 0;
}



int capture( void )
{
	


	// 5. create OpenCV Window
	//cv::namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );

	// 6. start
	VideoFrameRef  mColorFrame;
	VideoFrameRef  mDepthFrame;
	mDepthStream.start();
	mColorStream.start();
	int iMaxDepth = mDepthStream.getMaxPixelValue();
	
	
	//while( true )
	{
		// 7. check is color stream is available
		if( mColorStream.isValid() )
		{

			// 7a. get color frame
			if( mColorStream.readFrame( &mColorFrame ) == STATUS_OK )
			{
				//t_start = clock();
				// 7b. convert data to OpenCV format

				const cv::Mat mImageRGB(
						mColorFrame.getHeight(), mColorFrame.getWidth(),
						CV_8UC3, (void*)mColorFrame.getData() );
			
				//unsigned char* temp=(unsigned char*)mColorFrame.getData();

				//for(int i=0;i<921600;i++)
				//{
				//	pImage->rgb[i]=temp[i];
				//	//printf("%f, ",(float)temp[i]);
				//}
				//pImage->flag=true;

				// 7c. convert form RGB to BGR				
				cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
				uchar *pt;
				for (int i=0;i<icp_height;i++)
				{
					pt=cImageBGR.ptr<uchar>(i);
					for (int j=0;j<icp_width;j++)
					{
						texture[i][j][2]=pt[j*3];
						texture[i][j][1]=pt[j*3+1];
						texture[i][j][0]=pt[j*3+2];
					}
				} 
				// 7d. show image
				//cv::imshow( "Color Image", cImageBGR );
				//mImageBGR=cImageBGR;
				//pImg= IplImage(mImageBGR);
			}			
		}
		//cout<<"here"<<endl;

		// 8a. get depth frame
		const DepthPixel* pDepthArray = NULL;
		if( mDepthStream.readFrame( &mDepthFrame ) == STATUS_OK )
		{
			//convert coordinate:depth to world
			pDepthArray = (const DepthPixel*)mDepthFrame.getData();

			//t_start=clock();
			//unsigned char *tmp=pPointSet->xyz;
			for( int y = 0; y < mDepthFrame.getHeight(); ++ y )
			{
				for( int x = 0; x < mDepthFrame.getWidth(); ++ x )
				{
					int idx = x + y * mDepthFrame.getWidth();
					const DepthPixel&  rDepth = pDepthArray[idx];
					//cout<<x<<","<<y<<","<<rDepth<<endl;
					float fX, fY, fZ;
					CoordinateConverter::convertDepthToWorld( mDepthStream,	x, y, rDepth, &fX, &fY, &fZ );	
					//fZ=rDepth;
					//fX=(x-320)*fZ*0.0019047619;
					//fY=-(y-240)*fZ*0.0019047619;
					xyzdata[y][x][0]=fX;
					xyzdata[y][x][1]=fY;
					xyzdata[y][x][2]=fZ;
					//memcpy(tmp,&fX,4);
					//memcpy(tmp+4,&fY,4);
					//memcpy(tmp+8,&fZ,4);
					//tmp+=12;
					//xyzdata[y][x][0]=fX;
					//xyzdata[y][x][1]=fY;
					//xyzdata[y][x][2]=fZ;
				}
			}
			//pPointSet->flag=true;
			//t_end=clock();

			//load3dDataToGL(pPointSet->xyz);           
			//loadTextureToGL(cImageBGR);
		}
		memcpy(&curpos.x,recvbuf,4);
		memcpy(&curpos.y,recvbuf+4,4);
		memcpy(&curpos.ang,recvbuf+8,4);
		curpos.x*=1000;
		curpos.y*=1000;
	}
	return 0;
}



