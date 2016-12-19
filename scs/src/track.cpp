/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#include <math.h>
#include <malloc.h>
#include <string.h>

#include <GL/glut.h>
#include <ode/ode.h>

#include "track.h"
#include "common.h"
#include "car.h"
#include "draw.h"
#include "vector.h"

#define TYPE_BARRIER	0x01
#define TYPE_BARRIER_B	0x02
#define TYPE_DASH		0x04
#define TYPE_SLOP		0x08
#define TYPE_MIDL		0x10
#define TYPE_RIGHTA		0x20
#define TYPE_BLOCK		0x40
#define TYPE_RA_ID		0x80


	static int TriNum = 0;

	static dTriMeshDataID MeshData;
	static double track[MAXSEGMENT*2];
	static unsigned short type[MAXSEGMENT] = {0};
	int PointNum = 0;
	int PointNumEx=0;//for discontinuous Block

	static double *vertices;
	static dTriIndex *indices;

	double EndLineDistance;
	static int Segment;
	double TotalLength = 0.0;

	double PathSecurity = 0.0;

	int MiddleLineFlag = 0;
	int TrackReverseFlag = 0;

	static double ElevationAngle = ELEVATION_ANGLE;

	sVector LftOut	[MAXPOINT];
	sVector RgtOut	[MAXPOINT];
	sVector LftIn	[MAXPOINT];
	sVector RgtIn	[MAXPOINT];
	sVector SecureL	[MAXPOINT];
	sVector SecureR	[MAXPOINT];
	sVector Middle	[MAXPOINT];
	sVector Path	[MAXPOINT];
	double Cur		[MAXPOINT];
	double Limit	[MAXPOINT];
	sVector DashLO	[MAXPOINT];
	sVector DashLI	[MAXPOINT];
	sVector DashRO	[MAXPOINT];
	sVector DashRI	[MAXPOINT];
	sVector MIDLL	[MAXPOINT];
	sVector MIDLR	[MAXPOINT];
	sVector TriBlock[MAXPOINT][2];//like a slope but its width is 10cm, 0 is left
	int pMIDL		[MAXPOINT];
	int pRAID		[MAXPOINT];
	int pTriBlock	[MAXPOINT];
	sVector ELineL,ELineR;
	int PathNum = 0;
	int DashNum = 0;
	int pMidLNum=0;
	int pRAIDNum=0;
	int pTriBNum=0;

	double epsilon = 0.001;

void ReadTrack (const char * fname)
{
	FILE *fin;
	char line[256];
	char s[256] = {0};
	char c = 'A';
	
	if (strlen(fname)<1) {
		printf ("\tTRACK: There is no track file.\n");
		getchar ();
		exit (1);
	}
	fin = fopen(fname,"r");
	if (!fin) {
		printf ("\tTRACK: Can not open track file \"%s\".\n",fname);
		getchar ();
		exit (1);
	}

	printf ("\tTRACK: Reading track file \"%s\".\n",fname);
	do	fgets (line,255,fin);
	while (line[0]=='/');
	sscanf(line,"%lf%c",&EndLineDistance,&c);

	EndLineDistance /= 100.0;
	printf("\tTRACK: EndLineDistance: %5.2f.\n",EndLineDistance);
	int i = 0;
	do {
		do {
			if (fgets (line,255,fin)==NULL) {i++; break;}
		} while (strlen(line)<3 || line[0]=='/');
		if (i>=MAXSEGMENT) {
			printf("\tTRACK: The Track is too long to draw.\n");
			break;
		}
		memset (s,0,sizeof(s));
		sscanf (line,"%lf%lf%s",&track[i*2],&track[i*2+1],s);
		if (strlen(s)>1 && s[0]=='/' && s[1]=='/');
		else
			for (int j=0; j<(int)strlen(s); j++)
				switch (s[j]) {
					case '+': type[i] |= TYPE_BARRIER;	break;
					case '*': type[i] |= TYPE_BARRIER_B;break;
					case '.': type[i] |= TYPE_DASH;	break;
					case '^': type[i] |= TYPE_SLOP;	break;
					case '-': type[i] |= TYPE_MIDL; break;
					case '#': type[i] |= TYPE_RIGHTA; break;
					case '=': type[i] |= TYPE_BLOCK; break;
					case '!': type[i] |= TYPE_RA_ID; break;
					default : printf("TRACK: Unrecognized track type \"%c\".\n",s[j]);
				}
		i++;
	} while (track[(i-1)*2]!=0);//依靠长度为0来判断结束,故障碍要变矩形上升下降长度应写0.1之类的小值
	Segment = i-1;
	fclose(fin);
}

static void Reverse ()
{
	for (int i=0; i<Segment; i++)
		track[i*2+1] *= -1;
	for (int i=0; i<Segment/2; i++) {
		double t = track[i*2];
		track[i*2] = track[(Segment-i-1)*2];
		track[(Segment-i-1)*2] = t;
		t = track[i*2+1];
		track[i*2+1] = track[(Segment-i-1)*2+1];
		track[(Segment-i-1)*2+1] = t;
		unsigned short c = type[i];
		type[i] = type[Segment-i-1];
		type[Segment-i-1] = c;
	}
	EndLineDistance *= -1.0;
}

static void CalcPoint_OUT ()
{
	sVector dir(0,1,0);	// current direction
	TotalLength = 0.0;
	if (TrackReverseFlag) {
		LftOut[0].set(-TRACK_WIDTH_OUT/2.0,2.0,TRACK_HEIGHT);
		RgtOut[0].set( TRACK_WIDTH_OUT/2.0,2.0,TRACK_HEIGHT);
	}
	else {
		LftOut[0].set(-TRACK_WIDTH_OUT/2.0,0.0,TRACK_HEIGHT);
		RgtOut[0].set( TRACK_WIDTH_OUT/2.0,0.0,TRACK_HEIGHT);
	}
	DashLO[0] = LftOut[0];
	DashRO[0] = RgtOut[0];
	
	double SlopDir = 1.0;
	int p = 1;	// pointer
	int dp = 0;	// dash pointer
	int pmlp=0;//middle black line pointer
	int raidp=0;//right angle identification pointer
	int TriBp=0;//Triangle Block pointer
	if (cartype == balance)	ElevationAngle = ELEVATION_ANGLE_B;
	
	for (int i=0; i<Segment; i++) {		// Calc for each segment
		double length = track[i*2]/100.0;
		double degree = track[i*2];
		double R = track[i*2+1]/100.0;
		if ((type[i]&TYPE_MIDL)>0){//获取中心黑线段的头指针和头位置
			pMIDL[pmlp]=p-1;
			pmlp++;
			sVector v=RgtOut[p-1]-LftOut[p-1];v.Normalize();
			MIDLL[p-1]=LftOut[p-1]+v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
			MIDLR[p-1]=RgtOut[p-1]-v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
		}
		if ((type[i]&TYPE_RA_ID)>0){//获取直角前黑色标识的头指针
			pRAID[raidp]=p-1;
			raidp++;
		}
		if ((type[i]&TYPE_BLOCK)>0){//获取三角障碍的头指针
			if (track[2*i+1]>=0)
				pTriBlock[TriBp]=p-1;
			else
				pTriBlock[TriBp]=-(p-1);
			TriBp++;
		}
		if (fabs(R) < TRACK_WIDTH_OUT/2.0) {	// straight
			double uplen = BARRIER_HEIGHT;
			if (cartype==balance) uplen = BARRIER_HEIGHT*sqrt(3.0);
			if ((type[i]&TYPE_BARRIER)>0 ||(type[i]&TYPE_BARRIER_B)>0) {	// barrier
				if ((type[i]&TYPE_BARRIER_B)>0 && cartype!=balance) goto OtherBarrier;
				if ((type[i]&TYPE_BARRIER)>0 && cartype==balance) goto OtherBarrier;
				sVector up(0,0,BARRIER_HEIGHT);
				LftOut[p] = LftOut[p-1] + up + dir*uplen;	// move up
				RgtOut[p] = RgtOut[p-1] + up + dir*uplen;
				p ++;
				length -= 2*uplen;
				if (length<0.0) length = 0.0;
				TotalLength += uplen;
			}
OtherBarrier:
			if ((type[i] & TYPE_SLOP) >0) {
				dir.SetZ (SlopDir * tan(ElevationAngle*DEG_TO_RAD/2.0));
				LftOut[p] = LftOut[p-1] + TRANSITION_LENGTH * dir;
				RgtOut[p] = RgtOut[p-1] + TRANSITION_LENGTH * dir;
				p ++;
				length -= 2*TRANSITION_LENGTH;
				if (length<0.0) length = 0.0;
				TotalLength += TRANSITION_LENGTH;
				dir.SetZ (SlopDir * tan(ElevationAngle*DEG_TO_RAD));
			} else	dir.SetZ (0.0);
			LftOut[p] = LftOut[p-1] + length * dir;	// move forward
			RgtOut[p] = RgtOut[p-1] + length * dir;
			TotalLength += length;
			if ((type[i]&TYPE_DASH)>0) {
				DashLO[dp] = LftOut[p];
				DashRO[dp] = RgtOut[p];
				dp ++;
			}
			if ((type[i]&TYPE_MIDL)>0){//获取中心黑线段的尾位置
				sVector v=RgtOut[p]-LftOut[p];v.Normalize();
				MIDLL[p]=LftOut[p]+v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
				MIDLR[p]=RgtOut[p]-v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
			}
			p ++;
			if ((type[i]&TYPE_BARRIER) >0 || (type[i]&TYPE_BARRIER_B)>0) {
				if ((type[i]&TYPE_BARRIER_B)>0 && cartype!=balance) goto OtherBarrier2;
				if ((type[i]&TYPE_BARRIER)>0 && cartype==balance) goto OtherBarrier2;
				sVector down(0,0,-BARRIER_HEIGHT);
				LftOut[p] = LftOut[p-1] + down + dir*uplen;	// move down
				RgtOut[p] = RgtOut[p-1] + down + dir*uplen;
				p ++;
				TotalLength += uplen;
			}
OtherBarrier2:
			if ((type[i] & TYPE_SLOP) >0) {
				dir.SetZ (SlopDir * tan(ElevationAngle*DEG_TO_RAD/2.0));
				LftOut[p] = LftOut[p-1] + TRANSITION_LENGTH * dir;
				RgtOut[p] = RgtOut[p-1] + TRANSITION_LENGTH * dir;
				p ++;
				TotalLength += TRANSITION_LENGTH;
				SlopDir *= -1.0;
			} else	dir.SetZ (0.0);
			if (p>=MAXPOINT-1) {
				printf("\tTRACK: The Track is too long to draw.\n");
				goto StopCalcPoint;
			}
		}
		else if ((type[i]&TYPE_RIGHTA) >0){//break angle
			double StepDegree=degree*DEG_TO_RAD;
			double LenL=TRACK_WIDTH_OUT/2.0*(1-tan(StepDegree/2.0));
			double LenR=TRACK_WIDTH_OUT/2.0*(1+tan(StepDegree/2.0));//symmetric
			if (R<0.0){// swap curve's direction
				double temp=LenL;
				LenL=LenR;
				LenR=temp;
				StepDegree=-StepDegree;
				R=-R;//here it only need to be >= TRACK_WIDTH_OUT/2.0
			}
			LftOut[p]=LftOut[p-1]+LenL*dir;//move forward
			RgtOut[p]=RgtOut[p-1]+LenR*dir;
			dir.RotateZ (StepDegree);//turn an angle
			TotalLength+=(LenL+LenR)/2.0;
			p++;
			if (p>=MAXPOINT-1) {
				printf("\tTRACK: The Track is too long to draw.\n");
				goto StopCalcPoint;
			}
			LftOut[p]=LftOut[p-1]+LenL*dir;//move forward again
			RgtOut[p]=RgtOut[p-1]+LenR*dir;
			TotalLength+=(LenL+LenR)/2.0;//keep this angle
			p++;
			if (p>=MAXPOINT-1) {
				printf("\tTRACK: The Track is too long to draw.\n");
				goto StopCalcPoint;
			}
		}else{					// curve
			int StepNum = (int)fabs(R*degree*DEG_TO_RAD/TRACK_STEP);
			double StepDegree = degree*DEG_TO_RAD/StepNum;
			double LenL = (fabs(R)-TRACK_WIDTH_OUT/2.0)*sin(StepDegree/2.0)*2.0;
			double LenR = (fabs(R)+TRACK_WIDTH_OUT/2.0)*sin(StepDegree/2.0)*2.0;

			if ((type[i]&TYPE_BARRIER) >0) {	// barrier
				sVector up(0,0,BARRIER_HEIGHT);
				LftOut[p] = LftOut[p-1] + up;	// move down
				RgtOut[p] = RgtOut[p-1] + up;
				p ++;
			}

			if (R < 0.0) {			// swap curve's direction
				double temp = LenL;
				LenL = LenR;
				LenR = temp;
				StepDegree = -StepDegree;
				R = -R;
			}
			for (int j=0; j<StepNum; j++) {	// calc each step
				dir.RotateZ (StepDegree/2.0);	// turn the first half
				LftOut[p] = LftOut[p-1] + LenL * dir;	// move forward
				RgtOut[p] = RgtOut[p-1] + LenR * dir;
				dir.RotateZ (StepDegree/2.0);	// turn the last half
				TotalLength += fabs(StepDegree*R);
				if ((type[i]&TYPE_DASH)>0) {
					DashLO[dp] = LftOut[p];
					DashRO[dp] = RgtOut[p];
					dp ++;
				}
				if ((type[i]&TYPE_MIDL)>0){//获取中心黑线段的尾位置
					sVector v=RgtOut[p]-LftOut[p];v.Normalize();
					MIDLL[p]=LftOut[p]+v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
					MIDLR[p]=RgtOut[p]-v*(TRACK_WIDTH_OUT-LINE_WIDTH)/2;
				}
				p ++;
				if (p>=MAXPOINT-1) {
					printf("\tTRACK: The Track is too long to draw.\n");
					goto StopCalcPoint;
				}
			}
			if ((type[i]&TYPE_BARRIER) >0) {
				sVector down(0,0,-BARRIER_HEIGHT);
				LftOut[p] = LftOut[p-1] + down;	// move up
				RgtOut[p] = RgtOut[p-1] + down;
				p ++;
				if (p>=MAXPOINT-1) {
					printf("\tTRACK: The Track is too long to draw.\n");
					goto StopCalcPoint;
				}
			}
		}
		if ((type[i]&TYPE_MIDL)>0){//获取中心黑线段的尾指针
			pMIDL[pmlp]=p-1;
			pmlp++;
		}
		if ((type[i]&TYPE_RA_ID)>0){//获取直角前黑色标识的尾指针,只有直线有
			pRAID[raidp]=p-1;
			raidp++;
		}
		if ((type[i]&TYPE_BLOCK)>0){//获取三角障碍的头指针,只有直线有
			if (track[2*i+1]>=0)
				pTriBlock[TriBp]=p-1;
			else
				pTriBlock[TriBp]=-(p-1);
			TriBp++;
		}
	}
StopCalcPoint :
	// try to connect tobe a circle
	if (	LftOut[p-1].GetX()<0 && RgtOut[p-1].GetX()>0 &&	// position limit
		LftOut[p-1].GetY()<LftOut[0].GetY() && RgtOut[p-1].GetY()<RgtOut[0].GetY() &&
		Distance (LftOut[p-1],LftOut[0])<0.3 && Distance (RgtOut[p-1],RgtOut[0])<0.3 &&	// distance limit
		fabs(LftOut[p-1].GetX()-RgtOut[p-1].GetX())>0.9*TRACK_WIDTH_OUT)	// direction limit
	{
		LftOut[p] = LftOut[0];
		RgtOut[p] = RgtOut[0];
		TotalLength += (LftOut[p]-LftOut[p-1]+RgtOut[p]-RgtOut[p-1]).GetLen()/2.0;
		p ++;
		printf ("\tTRACK: Connect to be a circle automatically!\n");
	} else	printf ("\tTRACK: Can not connect to be a circle!\n");
	PointNum = p;
	DashNum = dp;
	pMidLNum=pmlp;
	pRAIDNum=raidp;
	pTriBNum=TriBp;
	sVector dirZ(0,0,1);
	int pEx=0;
	for (int i=0;i<pTriBNum;i+=2){
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		double sumLenL;
		switch ((i/2)%3){
		case 0:{
			sumLenL=0;
			sVector v=RgtOut[j0]-LftOut[j0];v.Normalize();
			if (pTriBlock[i]>=0)
				TriBlock[j0][0]=LftOut[j0]+v*0.075;//put left
			else
				TriBlock[j0][0]=LftOut[j0]+v*0.275;
			TriBlock[j0][1]=TriBlock[j0][0]+v*0.10;
			j0++;
			for (int j=j0;j<=jt;j++){
				sumLenL+=Distance(LftOut[j],LftOut[j-1]);
			}
			for (int j=j0;j<=jt;j++){
				double LenL=Distance(LftOut[j],LftOut[j-1]);
				sVector v=RgtOut[j]-LftOut[j];v.Normalize();
				sVector dir=LftOut[j]-LftOut[j-1];dir.Normalize();
				TriBlock[j][0]=TriBlock[j-1][0]+dir*LenL+0.05*dirZ*LenL/sumLenL;//high 5cm
				TriBlock[j][1]=TriBlock[j][0]+v*0.10;
			}
			break;
			   }
		case 1:
			j0++;
			for (int j=j0;j<=jt;j++){
				double LenL=Distance(LftOut[j],LftOut[j-1]);
				sVector v=RgtOut[j]-LftOut[j];v.Normalize();
				sVector dir=LftOut[j]-LftOut[j-1];dir.Normalize();
				TriBlock[j][0]=TriBlock[j-1][0]+dir*LenL;
				TriBlock[j][1]=TriBlock[j][0]+v*0.10;
			}
			break;
		case 2:
			sumLenL=0;
			j0++;
			for (int j=j0;j<=jt;j++){
				sumLenL+=Distance(LftOut[j],LftOut[j-1]);
			}
			for (int j=j0;j<=jt;j++){
				double LenL=Distance(LftOut[j],LftOut[j-1]);
				sVector v=RgtOut[j]-LftOut[j];v.Normalize();
				sVector dir=LftOut[j]-LftOut[j-1];dir.Normalize();
				TriBlock[j][0]=TriBlock[j-1][0]+dir*LenL-0.05*dirZ*LenL/sumLenL;
				TriBlock[j][1]=TriBlock[j][0]+v*0.10;
			}
			break;
		default:;
		}
		pEx+=jt-j0+1;//we don't connect jt to next j0
	}
	PointNumEx=pEx;
	printf("\tTRACK: TotalLength of track: %5.2f.\n",TotalLength);
	return ;
}

#define DASHSTEP 20

static void InsertDash ()
{
	for (int i=DashNum-1; i>=0; i--) {
		DashLO[DASHSTEP*i] = DashLO[i];
		DashRO[DASHSTEP*i] = DashRO[i];
	}
	for (int i=0; i<DashNum-1; i++)
		for (int j=1; j<DASHSTEP; j++) {
			DashLO[i*DASHSTEP+j] = (DashLO[(i+1)*DASHSTEP]-DashLO[i*DASHSTEP])*j/DASHSTEP + DashLO[i*DASHSTEP];
			DashRO[i*DASHSTEP+j] = (DashRO[(i+1)*DASHSTEP]-DashRO[i*DASHSTEP])*j/DASHSTEP + DashRO[i*DASHSTEP];
		}
	DashNum = DashNum*DASHSTEP - DASHSTEP + 1;
}

static void CalcDash_IN ()
{
	for (int i=0; i<DashNum; i++) {
		sVector v = DashRO[i] - DashLO[i];
		v.Normalize ();
		DashLI[i] = DashLO[i] + v * LINE_WIDTH*1.1;
		DashRI[i] = DashRO[i] - v * LINE_WIDTH*1.1;
	}
	for (int i=0; i<DashNum; i++) {
		sVector v = DashRO[i] - DashLO[i];
		v.Normalize ();
		DashLO[i] -= v * 0.001;
		DashRO[i] += v * 0.001;
	}
}

static void save (sVector v1, sVector v2, sVector v3)
{
	static int first = 1;
	static int i = 0;
	static int r = 0;
	if (r != TrackReverseFlag) {
		r = TrackReverseFlag;
		first = 1;
		free (vertices);
		i = 0;
	}
	if (first) {
		first = 0;
		vertices = (double*)malloc((PointNum+PointNumEx)*6*3*3*sizeof(double));	// 6 points(2 triangle), 3 dimension, 3 surface
	}
	vertices[i++] = v1.GetX();
	vertices[i++] = v1.GetY();
	vertices[i++] = v1.GetZ();

	vertices[i++] = v2.GetX();
	vertices[i++] = v2.GetY();
	vertices[i++] = v2.GetZ();

	vertices[i++] = v3.GetX();
	vertices[i++] = v3.GetY();
	vertices[i++] = v3.GetZ();

	TriNum += 3;
	if (i>PointNum*6*3*3) {
		printf("TRACK: save error!\n");
		getchar();
		exit (1);
	}
}

static void CalcMesh ()
{
	//vertices = (double*)malloc(PointNum*6*3*3*sizeof(double));
	TriNum = 0;
	for (int i=0; i<PointNum-1; i++) {
		save (LftOut[i],RgtOut[i],RgtOut[i+1]);
		save (RgtOut[i+1],LftOut[i+1],LftOut[i]);
	}

	for (int i=0; i<PointNum-1; i++) {
		sVector L0 = ProjectionXY (LftOut[i]);
		sVector L1 = ProjectionXY (LftOut[i+1]);	// do not create profile if it is not very high
		if (LftOut[i].GetZ()>2*TRACK_HEIGHT+0.001 || LftOut[i+1].GetZ()>2*TRACK_HEIGHT+0.001)
			save (LftOut[i],LftOut[i+1],L1);
		if (LftOut[i].GetZ()>2*TRACK_HEIGHT+0.001)
			save (L1,L0,LftOut[i]);
	}
	for (int i=0; i<PointNum-1; i++) {
		sVector R0 = ProjectionXY (RgtOut[i]);
		sVector R1 = ProjectionXY (RgtOut[i+1]);
		if (RgtOut[i].GetZ()>2*TRACK_HEIGHT+0.001)
			save (RgtOut[i],R0,R1);
		if (RgtOut[i].GetZ()>2*TRACK_HEIGHT+0.001 || RgtOut[i+1].GetZ()>2*TRACK_HEIGHT+0.001)
			save (R1,RgtOut[i+1],RgtOut[i]);
	}

	for (int i=0;i<pTriBNum;i+=2){
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		switch ((i/2)%3){
		case 0:break;
		case 1:j0++;break;
		case 2:j0++;jt--;break;//don't connect jt to next j0 
		default:;
		}
		for (int j=j0;j<=jt;j++){
			save(TriBlock[j][0],TriBlock[j][1],TriBlock[j+1][1]);
			save(TriBlock[j+1][1],TriBlock[j+1][0],TriBlock[j][0]);
		}
	}
	for (int i=0;i<pTriBNum;i+=2){
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		switch ((i/2)%3){
		case 0:break;
		case 1:j0++;break;
		case 2:j0++;jt--;break;//don't connect jt to next j0 
		default:;
		}
		for (int j=j0;j<=jt;j++){
			sVector L0 = ProjectionXY (TriBlock[j][0]);
			sVector L1 = ProjectionXY (TriBlock[j+1][0]);
			if (TriBlock[j][0].GetZ()>2*TRACK_HEIGHT+0.001||TriBlock[j+1][0].GetZ()>2*TRACK_HEIGHT+0.001)
				save (TriBlock[j][0],TriBlock[j+1][0],L1);
			if (LftOut[i].GetZ()>2*TRACK_HEIGHT+0.001)
				save (L1,L0,TriBlock[j][0]);
		}
	}
	for (int i=0;i<pTriBNum;i+=2){
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		switch ((i/2)%3){
		case 0:break;
		case 1:j0++;break;
		case 2:j0++;jt--;break;//don't connect jt to next j0 
		default:;
		}
		for (int j=j0;j<jt;j++){
			sVector R0 = ProjectionXY (TriBlock[j][1]);
			sVector R1 = ProjectionXY (TriBlock[j+1][1]);
			if (RgtOut[i].GetZ()>2*TRACK_HEIGHT+0.001)
				save (TriBlock[j][1],R0,R1);
			if (TriBlock[j][1].GetZ()>2*TRACK_HEIGHT+0.001||TriBlock[j+1][1].GetZ()>2*TRACK_HEIGHT+0.001)
				save (R1,TriBlock[j+1][1],TriBlock[j][1]);
		}
	}

	indices = (dTriIndex*)malloc(TriNum*sizeof(dTriIndex));
	for (int i=0; i<TriNum; i++)
		indices[i] = i;
}

static void MakeMesh (dSpaceID space)
{
	dMatrix3 R;
	MeshData = dGeomTriMeshDataCreate ();

	dGeomTriMeshDataBuildDouble (
		MeshData,
		vertices,
		3 * sizeof(double),
		TriNum,
		indices,
		TriNum,
		3 * sizeof(dTriIndex)
	);
	dGeomID track_mesh = dCreateTriMesh (space, MeshData, 0, 0, 0);
	dGeomTriMeshEnableTC (track_mesh, dCylinderClass, 0);
	dGeomTriMeshEnableTC (track_mesh, dBoxClass, 0);
	dGeomSetPosition (track_mesh, 0.0, 0.0, 0.0);
	dRSetIdentity (R);
	dGeomSetRotation (track_mesh, R);
}

static void CalcPoint_IN ()
{
	for (int i=0; i<PointNum; i++) {
		sVector v = RgtOut[i] - LftOut[i];
		v.Normalize ();
		LftIn[i] = LftOut[i] + v * LINE_WIDTH;
		RgtIn[i] = RgtOut[i] - v * LINE_WIDTH;
	}
}

static void CalcPoint_Middle ()
{
	for (int i=0; i<PathNum; i++)
		Middle[i] = (SecureL[i]+SecureR[i])/2.0;
}

static void CalcSecure ()
{
	int j = 0;
	for (int i=0; i<PointNum-1; i++,j++) {
		SecureL[j] = LftOut[i];
		SecureR[j] = RgtOut[i];

		sVector v1 = LftOut[i] - RgtOut[i];
		sVector v2 = LftOut[i+1] - RgtOut[i+1];
		if (v1!=v2){
			double distance=(Distance(LftOut[i],LftOut[i+1])+Distance(RgtOut[i],RgtOut[i+1]))/2;
			int StepNum=(int)(distance/TRACK_STEP);
			if (StepNum<2) continue;
			sVector dir=(LftOut[i+1]-LftOut[i]+RgtOut[i+1]-RgtOut[i]);
			dir.Normalize ();
			for (int k=0; k<StepNum; k++,j++) {
				SecureL[j+1] = SecureL[j] + dir * distance/(StepNum+1.0);
				SecureR[j+1] = SecureR[j] + dir * distance/(StepNum+1.0);
			}
		}else{
			double distance = Distance (LftOut[i],LftOut[i+1]);
			int StepNum = (int)(distance/TRACK_STEP);
			sVector dir = LftOut[i+1] - LftOut[i];
			dir.Normalize ();
			for (int k=0; k<StepNum; k++,j++) {
				SecureL[j+1] = SecureL[j] + dir * distance/(StepNum+1.0);
				SecureR[j+1] = SecureR[j] + dir * distance/(StepNum+1.0);
			}
		}
	}
	PathNum = j;

	if (PathSecurity > (TRACK_WIDTH_OUT-CarWidth)/2.0) {
		PathSecurity = (TRACK_WIDTH_OUT-CarWidth)/2.0-0.001;
		printf ("PATH: The security of path is too high.\n");
	}
	if (PathSecurity < -CarWidth/2.0) {
		PathSecurity = -CarWidth/2.0;
		printf ("PATH: The security of path is too low.\n");
	}
	for (int i=0; i<PathNum; i++) {
		sVector v = SecureR[i] - SecureL[i];
		v.Normalize ();
		SecureL[i] += v * (PathSecurity+CarWidth/2.0);
		SecureR[i] -= v * (PathSecurity+CarWidth/2.0);
	}
}

void MakeTrack (dSpaceID space, const char * fname)
{
	static int first = 1;
	static int r = 1;
	if (first) {
		first = 0;
		ReadTrack (fname);
		Reverse ();
	}
	if (r!=TrackReverseFlag) {
		r = TrackReverseFlag;
		Reverse ();
		CalcPoint_OUT ();
		CalcMesh ();
		MakeMesh (space);
		CalcPoint_IN ();
		InsertDash ();
		CalcDash_IN ();
		CalcSecure ();
		CalcPoint_Middle ();
	}
	else	MakeMesh (space);
}

void DestroyTrack ()
{
	dGeomTriMeshDataDestroy (MeshData);
}

static void DrawEndLine ()
{
	double len = EndLineDistance;
	int WinID = glutGetWindow ();
	while (len<0.0) len += TotalLength;
	while (len>TotalLength) len -= TotalLength;

	for (int i=1; i<PointNum; i++) {
		sVector p = (LftOut[i-1]+RgtOut[i-1]) / 2.0;
		sVector n = (LftOut[i  ]+RgtOut[i  ]) / 2.0;
		len -= (n-p).GetLen();
		if (len<0.0) {
			sVector dir = (RgtOut[i]-LftOut[i]).RotateZ(M_PI_2).Normalize();
			ELineL = LftOut[i] + dir*len;
			ELineR = RgtOut[i] + dir*len;
			break;
		}
	}
		
	glColor3f (0.0f,0.0f,0.0f);
	glPushMatrix ();
	if (WinID==WinGod)	epsilon = ViewPoint.GetZ()/500.0;
	else				epsilon = 0.001;
	glTranslated(ELineL.GetX(),ELineL.GetY(),ELineL.GetZ()+epsilon);
	epsilon = 0.001;
	sVector v = ELineR - ELineL;
	double theta = atan(v.GetY()/v.GetX());
	if (v.GetX()<0)	if (v.GetY()>0)	theta += M_PI;
			else		theta -= M_PI;
	glRotated (theta*RAD_TO_DEG,0,0,1);
	/*glRectd (TRACK_WIDTH_OUT/8.0,0,TRACK_WIDTH_OUT*3.0/8.0,LINE_WIDTH);
	glRectd (TRACK_WIDTH_OUT*5.0/8.0,0,TRACK_WIDTH_OUT*7.0/8.0,LINE_WIDTH);*/
	glRectd (-TRACK_WIDTH_OUT/4.0,0,0,LINE_WIDTH);//修改后的起跑标识
	glRectd (TRACK_WIDTH_OUT,0,TRACK_WIDTH_OUT*5/4.0,LINE_WIDTH);
	glPopMatrix ();
}

void DrawMiddleLine ()
{
	glColor3f (0,0,0);
	glLineWidth (1);
	glBegin (GL_LINES);
	epsilon = ViewPoint.GetZ()/500.0;
	for (int i=0; i<PathNum; i++)
		glVertex3d (Middle[i].GetX(),Middle[i].GetY(),Middle[i].GetZ()+epsilon);
	epsilon = 0.001;
	glEnd ();
}

void DrawTrack ()
{
	int WinID = glutGetWindow ();

	glColor3f (0.6f,0.6f,0.6f);
	glBegin (GL_QUAD_STRIP);
	for (int i=0; i<PointNum; i++) {				// left profile
		if (LftOut[i].GetZ()<0.02) {
			glEnd();
			glBegin (GL_QUAD_STRIP);
			continue;
		}
		glVertex3d (LftOut[i].GetX(),LftOut[i].GetY(),LftOut[i].GetZ()-epsilon);
		glVertex3d (LftOut[i].GetX(),LftOut[i].GetY(),0);
	}
	glEnd ();

	glColor3f (0.6f,0.6f,0.6f);
	glBegin (GL_QUAD_STRIP);
	for (int i=0; i<PointNum; i++) {				// right profile
		if (RgtOut[i].GetZ()<0.02) {
			glEnd();
			glBegin (GL_QUAD_STRIP);
			continue;
		}
		glVertex3d (RgtOut[i].GetX(),RgtOut[i].GetY(),RgtOut[i].GetZ()-epsilon);
		glVertex3d (RgtOut[i].GetX(),RgtOut[i].GetY(),0);
	}
	glEnd ();

	glColor3f (0.0f,0.0f,0.0f);
	glBegin (GL_QUAD_STRIP);
	epsilon = 0.001;
	for (int i=0; i<PointNum; i++) {				// border
		glVertex3d (LftOut[i].GetX(),LftOut[i].GetY(),LftOut[i].GetZ()-epsilon);
		glVertex3d (RgtOut[i].GetX(),RgtOut[i].GetY(),RgtOut[i].GetZ()-epsilon);
	}
	glEnd ();
	//Draw Triangle Block
	glColor3f (0.06f,0.06f,0.06f);
	epsilon = 0.001;
	for (int i=0; i<pTriBNum; i+=2) {				
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		glBegin (GL_QUAD_STRIP);
		for (int j=j0;j<=jt;j++){// left profile
			glVertex3d (TriBlock[j][0].GetX(),TriBlock[j][0].GetY(),TriBlock[j][0].GetZ()-epsilon);
			glVertex3d (TriBlock[j][0].GetX(),TriBlock[j][0].GetY(),0);
		}
		glEnd ();
		glBegin (GL_QUAD_STRIP);
		for (int j=j0;j<=jt;j++){// right profile
			glVertex3d (TriBlock[j][1].GetX(),TriBlock[j][1].GetY(),TriBlock[j][1].GetZ()-epsilon);
			glVertex3d (TriBlock[j][1].GetX(),TriBlock[j][1].GetY(),0);
		}
		glEnd ();
	}
	glColor3f (0.0f,0.0f,0.0f);
	for (int i=0; i<pTriBNum; i+=2) {
		int j0=abs(pTriBlock[i]),jt=abs(pTriBlock[i+1]);
		glBegin (GL_QUAD_STRIP);
		for (int j=j0;j<=jt;j++){
			glVertex3d (TriBlock[j][0].GetX(),TriBlock[j][0].GetY(),TriBlock[j][0].GetZ()-epsilon);
			glVertex3d (TriBlock[j][1].GetX(),TriBlock[j][1].GetY(),TriBlock[j][1].GetZ()-epsilon);
		}
		glEnd ();
	}

glEnable(GL_POLYGON_OFFSET_FILL);
glPolygonOffset(-1.0f, -1.0f);
	glColor3d (0.9,0.0,0.0);
	glBegin (GL_QUAD_STRIP);
	double len = 0.05;
	for (int i=0; i<DashNum; i++) {					// dash left
		double dlen;
		if (i>0) dlen = (DashLI[i] - DashLI[i-1]).GetLen ();
		else	dlen = 0.0;
		len += dlen;
		if (dlen>TRACK_STEP/5) {
			len = 0.05;
			glEnd ();
			glBegin (GL_QUAD_STRIP);
		}
		if (((int)(len/0.1))%2)	glColor3d (0.66,0.66,0.66);
		else			glColor3d (0.0,0.0,0.0);
		glVertex3d (DashLO[i].GetX(),DashLO[i].GetY(),DashLO[i].GetZ()-epsilon);
		glVertex3d (DashLI[i].GetX(),DashLI[i].GetY(),DashLI[i].GetZ());
	}
	glEnd ();


	glColor3d (0.66,0.66,0.66);
	glBegin (GL_QUAD_STRIP);
	len = 0.05;
	for (int i=0; i<DashNum; i++) {					// dash right
		double dlen;
		if (i>0) dlen = (DashRI[i] - DashRI[i-1]).GetLen ();
		else	dlen = 0.0;
		len += dlen;
		if (dlen>TRACK_STEP/5) {
			len = 0.05;
			glEnd ();
			glBegin (GL_QUAD_STRIP);
		}
		if (((int)(len/0.1))%2)	glColor3d (0.66,0.66,0.66);
		else			glColor3d (0.0,0.0,0.0);
		glVertex3d (DashRO[i].GetX(),DashRO[i].GetY(),DashRO[i].GetZ()-epsilon);
		glVertex3d (DashRI[i].GetX(),DashRI[i].GetY(),DashRI[i].GetZ());
	}
	glEnd ();
glDisable(GL_POLYGON_OFFSET_FILL);

	glColor3f (0.66f,0.66f,0.66f);
	glBegin (GL_QUAD_STRIP);
	if (WinID==WinGod)	epsilon = ViewPoint.GetZ()/1000.0;
	else epsilon = 0.0;
	for (int i=0; i<PointNum; i++) {				// track surface (cover the black border)
		if (fabs(LftIn[i].GetZ() - TRACK_HEIGHT - BARRIER_HEIGHT)<0.001 && cartype == balance)
			glColor3f (0.0f,0.0f,0.0f);
		else
			glColor3f (0.66f,0.66f,0.66f);
		glVertex3d (LftIn[i].GetX(),LftIn[i].GetY(),LftIn[i].GetZ()+epsilon);
		glVertex3d (RgtIn[i].GetX(),RgtIn[i].GetY(),RgtIn[i].GetZ()+epsilon);
	}
	epsilon = 0.001;
	glEnd ();
	//Draw Middle Black Line
	glColor3f (0.66f,0.66f,0.66f);
	epsilon = 0.001;
	for (int i=0; i<pMidLNum-1; i+=2) {				// clear two black lines
		glBegin (GL_QUAD_STRIP);
		for (int j=pMIDL[i];j<=pMIDL[i+1];j++){
			glVertex3d (LftOut[j].GetX(),LftOut[j].GetY(),LftOut[j].GetZ()-epsilon);
			glVertex3d (RgtOut[j].GetX(),RgtOut[j].GetY(),RgtOut[j].GetZ()-epsilon);
		}
		glEnd ();
	}
	glColor3f (0.0f,0.0f,0.0f);
	if (WinID==WinGod)	epsilon = ViewPoint.GetZ()/500.0;
	else epsilon = 0.001;
	for (int i=0; i<pMidLNum-1; i+=2) {				// track surface (cover the white board)
		glBegin (GL_QUAD_STRIP);
		for (int j=pMIDL[i];j<=pMIDL[i+1];j++){
			glVertex3d (MIDLL[j].GetX(),MIDLL[j].GetY(),MIDLL[j].GetZ()+epsilon);
			glVertex3d (MIDLR[j].GetX(),MIDLR[j].GetY(),MIDLR[j].GetZ()+epsilon);
		}
		glEnd ();
	}
	//Draw Right Angle Identification
	for (int i=0; i<pRAIDNum-1; i+=2) {
		glBegin (GL_QUAD_STRIP);
		for (int j=pRAID[i];j<=pRAID[i+1];j++){
			glVertex3d (LftOut[j].GetX(),LftOut[j].GetY(),LftOut[j].GetZ()+epsilon);
			glVertex3d (RgtOut[j].GetX(),RgtOut[j].GetY(),RgtOut[j].GetZ()+epsilon);
		}
		glEnd ();
	}
	epsilon = 0.001;

	glColor3f (0.6f,0.6f,0.6f);
	glBegin (GL_QUAD_STRIP);
	for (int i=0; i<PointNum; i++) {				// track button
		glVertex3d (LftOut[i].GetX(),LftOut[i].GetY(),0.0);
		glVertex3d (RgtOut[i].GetX(),RgtOut[i].GetY(),0.0);
	}
	glEnd ();

	DrawEndLine ();//第十届没有起跑线,但可以修改后做标识起点

	if (WinID!=WinCar && viewtype!=car && MiddleLineFlag)	DrawMiddleLine();
}
