#include "GL/glut.h"
#include "scs.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <ctime>
#include "system.h"

u16 TimeUsed=0,g_nCount10Hz,g_nCount10ms;
clock_t timezero,timezero2;
s16 GuideLine[IMG_ROWS][IMG_COLS];
b2 MotorPIDAllow;
unsigned char line[GRAPH_WIDTH];
s32 ShowNum;
unsigned char graph[GRAPH_HEIGHT][GRAPH_WIDTH];
extern u8 MaxRow,MaxRRow,MaxLRow;
extern b2 g_nLCap[IMG_ROWS],g_nRCap[IMG_ROWS],g_nLCapU[IMG_ROWS],g_nRCapU[IMG_ROWS];
extern u8 g_nLPos[IMG_ROWS+1],g_nRPos[IMG_ROWS+1],g_nLPosU[IMG_ROWS],g_nRPosU[IMG_ROWS];

void AI_Camera ();
void Display ();

void AI_Electromagnetic (){
}

void AI_Balance (){
}

int main(int argc, char *argv[])
{
	switch (0) {
		default:
		case 0:
			sSetCar (camera);
			sSetAiFunc (AI_Camera);
			sSetCamera(sVector(0.23,0,0.2));//0.33
			sSetDepressionAngle(30);
			sEnableCustomWindow ();
			sEnableRoute();
			sEnableReverse();
			cvNamedWindow("ÉãÏñÍ·Í¼Ïñ");
			sSetDisplayFunc(Display);
			DirWeightInit();
			sSetBatteryPosition(sVector(0.0,0.02,0.02));
			timezero=clock();MotorPIDAllow=1;
			break;
		case 1:
			sSetCar (balance);
			sSetAiFunc (AI_Balance);
			sEnableCustomWindow ();
			sSetDisplayFunc (Display);
			sSetDepressionAngle (60.0);
			break;
		case 2:
			sSetCar (electromagnetic);
			sSetAiFunc (AI_Electromagnetic);
			break;
	}
	sSetTrack ("C:/CyberCar/ai/track/demo1.trk");
	sRegister("I've read the license. And I accept it.");
	scsMainLoop (&argc,argv);

	cvDestroyWindow("ÉãÏñÍ·Í¼Ïñ");
	return 0;
}
void AI_Camera ()
{
	static clock_t Lclock=clock(),Lclock2=clock();
	//if (clock()>=Lclock+20){
		sGetGraph (graph);				// ¶ÁÈ¡Í¼Ïñ
		//std::cout<<"cost time is: "<<(Lclock2-Lclock)<<std::endl;
		//Lclock=clock();
	//}
	for (int i=0;i<IMG_ROWS;++i)
		for (int j=0;j<IMG_COLS;++j){
			if (graph[i][IMG_COLS-1-j]<=20||graph[i][IMG_COLS-1-j]>=200)
			g_nCamFrameBuffer[i][j]=graph[i][IMG_COLS-1-j];
			else g_nCamFrameBuffer[i][j]=255;
			GuideLine[i][j]=0;
		}
	if ((unsigned long)(clock()-timezero)>=100){timezero=clock();TimeUsed++;++g_nCount10Hz;}
	if ((unsigned long)(clock()-timezero2)>=10){timezero2=clock();++g_nCount10ms;}
	GetLine();
	MotorCtrl();
	DirCtrl();
	Speed_PID();
}
void Display ()
{
	/*glTranslated ( -0.5 , -0.5 ,0.0);	// Æ½ÒÆ
	glScaled (0.2 ,0.2 ,0.2);			// Ëõ·Å
	glColor3d (1.0 ,0.0 ,0.0);			// ÑÕÉ«
	
	glBegin (GL_LINE_STRIP );			// »­Ïß¶Î
	for (int i=0; i<5; i++) {
		glVertex2d (( double)i ,1.0);	// Ïß¶Î×ø±ê
		glVertex2d (( double)i ,0.0);
	}
	glEnd ();							// ½áÊø
	
	glLoadIdentity ();					// ¸´Î»
	glTranslated (0.0 ,0.5 ,0.0);		// Æ½ÒÆ
	glColor3d (0.0 ,1.0 ,0.0);			// ÑÕÉ«
	glRectd ( -0.1 , -0.1 ,0.1 ,0.1);*/	// »­¾ØÐÎ
	const int amp=2;
	IplImage* imgtemp=cvCreateImage(cvSize(amp*IMG_COLS,amp*IMG_ROWS),8,3);
	RgbImage rgbtemp(imgtemp);
	for (int i=0;i<IMG_ROWS;++i){
		for (int j=0;j<IMG_COLS;++j){
			for (int ii=0;ii<amp;++ii)
				for (int jj=0;jj<amp;++jj){
					rgbtemp[i*amp+ii][j*amp+jj].b=rgbtemp[i*amp+ii][j*amp+jj].g=rgbtemp[i*amp+ii][j*amp+jj].r=g_nCamFrameBuffer[IMG_ROWS-1-i][IMG_COLS-1-j];
				}
		}
		if (g_nLCap[IMG_ROWS-1-i]){
			for (int ii=0;ii<amp;++ii)
				for (int jj=0;jj<amp;++jj){
					rgbtemp[i*amp+ii][(IMG_COLS-1-g_nLPos[IMG_ROWS-1-i])*amp+jj].b=255;
					rgbtemp[i*amp+ii][(IMG_COLS-1-g_nLPos[IMG_ROWS-1-i])*amp+jj].g=rgbtemp[i*amp+ii][(IMG_COLS-1-g_nLPos[IMG_ROWS-1-i])*amp+jj].r=0;
				}
		}
		if (g_nRCap[IMG_ROWS-1-i]){
			for (int ii=0;ii<amp;++ii)
				for (int jj=0;jj<amp;++jj){
					rgbtemp[i*amp+ii][(IMG_COLS-1-g_nRPos[IMG_ROWS-1-i])*amp+jj].b=255;
					rgbtemp[i*amp+ii][(IMG_COLS-1-g_nRPos[IMG_ROWS-1-i])*amp+jj].g=rgbtemp[i*amp+ii][(IMG_COLS-1-g_nRPos[IMG_ROWS-1-i])*amp+jj].r=0;
				}
		}

		for (int j=0;j<IMG_COLS;++j){
			for (int ii=0;ii<amp;++ii)
				for (int jj=0;jj<amp;++jj){
					rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][0])%IMG_COLS*amp+jj].r=255;
					rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][0])%IMG_COLS*amp+jj].g=rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][0])%IMG_COLS*amp+jj].b=0;
					rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][3])%IMG_COLS*amp+jj].r=128;
					rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][3])%IMG_COLS*amp+jj].g=rgbtemp[i*amp+ii][(2*IMG_COLS-1-GuideLine[IMG_ROWS-1-i][3])%IMG_COLS*amp+jj].b=50;
					}
		}
	}
	clock_t timer=clock()-timezero;
	float temp=(float)timer/1000.0f;
	char monitor_timer[10];
	sprintf(monitor_timer,"%f",temp);
	cvShowImage("ÉãÏñÍ·Í¼Ïñ",imgtemp);
	cvReleaseImage(&imgtemp);
}
