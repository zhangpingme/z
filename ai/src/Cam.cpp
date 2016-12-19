#include "system.h"
#include "scs.h"

#define WID 95
#define thr 70

U8 g_nCamFrameIndex,g_nNowRow;
int g_nCamFrameBuffer[IMG_ROWS][IMG_COLS];

b2 g_nLCap[IMG_ROWS],g_nRCap[IMG_ROWS],g_nLCapTB[IMG_ROWS],g_nRCapTB[IMG_ROWS],g_nLMayTB,g_nRMayTB;
u8 g_nLPos[IMG_ROWS+1],g_nRPos[IMG_ROWS+1],g_nLPosTB[IMG_ROWS],g_nRPosTB[IMG_ROWS],g_nWidth[IMG_ROWS+1];

int dir;
void GetLine(void){

    static int left = WID / 2, right = WID / 2 , mid = WID / 2;
    int i;

    for (i = mid; i > 0; i--){
      if (g_nCamFrameBuffer[20][i] < thr)
      break;
      }
    left = i;
    for (i = mid; i < WID; i++){
      if (g_nCamFrameBuffer[20][i] < thr)
      break;
      }
    right = i;

    mid = left + right;
    mid /= 2;

    static int e;
    static int last_e;
    e = mid  - WID / 2;
    int dir;
    dir = 2 * e + 6 * (e-last_e);      //修改此处参数调整舵机转向
    last_e = e;
	sSetServoDir(-dir);
	sSetMotorL(2);
	sSetMotorR(2);						//修改此处参数调整电机输出
    
}