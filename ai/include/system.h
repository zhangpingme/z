#ifndef SYSTEM_H
#define SYSTEM_H
#include <stdio.h>
#include "highgui.h"
#include "cv.h"
#include "type.h"
#include "Cam.h"
#include "Drive.h"

#define	SERVO_INIT 4720
#define SERVO_PER_DEGREE 30	//per servo's degree,about 1/2 of the nosewheel
#define	SERVO_WIDTH	6000

extern u16 g_nCount10Hz,g_nCount10ms;
extern int dir;


//Functions
#endif
