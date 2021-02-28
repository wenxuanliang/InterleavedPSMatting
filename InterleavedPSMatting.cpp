/*************************************************************************************\
Microsoft Research Asia

Module Name:
	
	This file is used for the interleaved lighting PS, Segmentation and Alpha-Matting.
	This cpp file is inherited from MdigPSMatting.cpp.
	So the History of MdigPSMatting is just listed here.

History:

	File name: MdigPSMatting.cpp
	Synopsis:  This program tried to combine the Cg code with the MdigProcess Program.
 
	Note: The average processing time must be shorter than the grab time or
             some frames will be missed. Also, if the processing results are not
             displayed and the frame count is not drawn or printed, the
             CPU usage is reduced significantly.
	Version 1.0:	Created by Wenxuan Liang at Jan09 2010
			Based on the experiment in the project MdigCgCombine;
			Add the matting part.
			It worked after simple debugging.
	Version 1.1:	Created by Wenxuan Liang at Jan10 2010
			Three mode is completed. The matting algorithm has been tested and passed after
			placing the light sources at better locations.
	Version 1.2:	Created by Wenxuan Liang at Jan11 2010
			Notice that UNSIGNED_CHAR texture data is auto-normalized when being sampled.
			Faked background has been added.
			The frame rate of matting is 25Hz, for PS is 30Hz, and for PSMatting is just 20Hz.
			So maybe the Cg code for matting should be further optimized.
	Version 1.3:	Created by Wenxuan Liang at Jan12 2010
			Try to subtract the Dark Image before transferring the ambient image.
			But it doesn't help. So the Version 1.3 is skipped.
	Version 1.4:	Created by Wenxuan Liang at Jan17 2010
			Add the bit shift to prevent saturation under multiplex lights.
			Also de-multiplex the light image in the matting shader.
			Version 1.4 is incomplete.
			The the de-multiplex has been realized in version 1.2.1.
			Bit shifting is not so urgent, so version 1.4 is hanging...
	Version 1.5:	Created by Wenxuan Liang at Jan18 2010
			We want to add alpha matting into this code.
			Also updating a minor mistake in frame-missing decision. 
			Note that expressions in C always lift the precision during calculation.
\*************************************************************************************/

#ifdef WIN32
#include <windows.h>
#endif
#include <mil.h> 
#include <GL/glew.h>  

#include "..\\Common\\Exception.hpp"
#include "..\\Common\\Image.hpp"
#include "..\\Common\\CgFragmentProgram.hpp"

#include <GL/glut.h>
#include <GL/glext.h>

#include <iostream>
#include <math.h>
using namespace std;



/*	Global variables.	*/
const	int		iDarkBufNmbr	= 128;
const	int		iBgStatFrmNmbr	= 64;
const	int		iPackedBufNmbr	= 4;
// Used for normal grabbing.
const	int		iAccDeg			= 8;
const	int		iLghtNmbr		= 4;
const	int		iGrpFrmNmbr		= iAccDeg * iLghtNmbr;
const	int		iGrabBufNmbr	= 2 * iGrpFrmNmbr;
// Used for alpha matting.
const	int		iAccDeg_am		= 11;
const	int		iLghtNmbr_am	= 3;
const	int		iGrpFrmNmbr_am	= iAccDeg_am * iLghtNmbr_am;
const	int		iGrabBufNmbr_am	= 2 * iGrpFrmNmbr_am;
//#define AlterBufSize 60
#define NUM_LIGHTS (3) 
//#define iPackedBufNmbr (61)

//#define INFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Mat\\frm_%d.pgm"
#define INFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Pad\\Pad-L%d-F%03d.pgm"
//#define BG_NORMAL_FILE_TMPL		"..\\Data\\brickWallNormal-L%d-F%03d.pgm"
//#define BG_NORMAL_FILE_TMPL		"..\\Data\\ballWallNormal-L%d-F%03d.pgm"
//#define BG_ALBEDO_FILE_TMPL		"..\\Data\\brickWallColor-L%d-F%03d.pgm"
#define BG_NORMAL_FILE_TMPL		"..\\Data\\bricksNormal-L%d-F%03d.pgm"
#define BG_ALBEDO_FILE_TMPL		"..\\Data\\bricksColor-L%d-F%03d.pgm"
#define BG_ALBEDO_FILE_TMPL_2	"..\\Data\\NationalPark-L%d-F%03d.pgm"
#define BG_ALBEDO_FILE_TMPL_3	"..\\Data\\VietnamMuiNe-L%d-F%03d.pgm"
#define BG_ALBEDO_FILE_TMPL_4	"..\\Data\\Datong-L%d-F%03d.pgm"
#define BG_ALBEDO_FILE_TMPL_5	"..\\Data\\HuangLongPools-L%d-F%03d.pgm"
//#define NEWFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Mat\\Mat-F%03d-L%d.pgm"
//#define INFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Pillow\\Pillowfrm_%d.pgm"
//#define INFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Wenxuan\\Wenxuanfrm_%d.pgm"
//#define INFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Bennett\\Bennettfrm_%d.pgm"
#define START_IMG_NUM (0)
// #define DARKFILENAME "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Mat\\Dark.pgm"
#define CALIBFILE "..\\Data\\Mat\\Lp_transpose.txt"

// Name of Cg fragment shader programs
#define CG_PS_PASS1				"..\\CgShaders\\P1S3NormAlbedo.cg"
#define CG_PS_P1SRCS			"..\\CgShaders\\P1S4Sources.cg"
#define CG_PS_P2NTBAYER			"..\\CgShaders\\P2DiffuseBayer.cg"
#define CG_PS_P2VIZNORMS		"..\\CgShaders\\P2VizNorms.cg"
#define CG_PS_P2CLONE			"..\\CgShaders\\P2Clone.cg"

//#define	MULTIPLEX_LIGHT
#ifdef	MULTIPLEX_LIGHT
	#define CG_MATTING_BGMEAN		"..\\CgShaders\\BackgroundMean.cg"
	#define CG_MATTING_BGVAR		"..\\CgShaders\\BackgroundVar.cg"
	#define CG_MATTING_JUSTSHOW		"..\\CgShaders\\JustShow.cg"
	#define CG_MATTING_MATTING		"..\\CgShaders\\Matting.cg"
#else
	#define CG_MATTING_BGMEAN		"..\\CgShaders\\BackgroundMean_v1_0.cg"
	#define CG_MATTING_BGVAR		"..\\CgShaders\\BackgroundVar_v1_0.cg"
	#define CG_MATTING_JUSTSHOW		"..\\CgShaders\\JustShow_v1_0.cg"
	#define CG_MATTING_MATTING		"..\\CgShaders\\Matting_v1_0.cg"
#endif
#define CG_JUSTSHOW_DEMOSAIC	"..\\CgShaders\\JustShowDemosaic.cg"
#define CG_MATTING_MEDIAN		"..\\CgShaders\\MattingMedian.cg"
#define CG_PS_MATTING			"..\\CgShaders\\PSMatting.cg"
#define CG_ALPHA_MATTING		"..\\CgShaders\\AlphaMatting.cg"

// For making videos
#define UILOGFILE "uiLog.txt"
#define OUTFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Video1\\ntClip%04d.ppm"


// For making videos
#define UILOGFILE "uiLog.txt"
#define OUTFILE_TMPL "D:\\Shared\\Bennett\\RealTimePhotometric\\Data\\Video1\\ntClip%04d.ppm"

enum PhotometricShaders 
{
	PS_SOURCES,		 // Show raw source images
	PS_NORMALS,		 // Show normal visualization
	PS_ALBEDO,		 // Computed albedo
	PS_DIFFUSE,		 // Rendered with diffuse lighting
	PS_PHONG,		 // Rendered with diffuse and specular (Phong)
	PS_ERROR,		 // Renders error between measured and predicted intensities
	PS_GRBG_SOURCES, // Sources with Bayer demosaicing
	PS_P2NT,		 // Two pass diffuse with original normal transformation
	PS_P2NTREL,		 // Two pass diffuse with normal transformation relative to avg
	PS_P2DEBUG		 // Two pass, debug shader place holder
};

#define PS_LEFTBUTTON (0x01)
#define PS_MIDDLEBUTTON (0x02)
#define PS_RIGHTBUTTON (0x04)
int mouseButtonState = 0;		// Used to track mouse button state

#define WIDTH (640)
#define HEIGHT (480)
int windowH = HEIGHT;
int windowW = WIDTH;

int startImage = START_IMG_NUM;  // Which file number to start with reading textures
int numLights = NUM_LIGHTS;

// For dumping keystrokes and mouse info to make videos.
enum UiModes
{
		UI_NORMAL,	// regular
		UI_LOG,		// log mouse and keyboard strokes
		UI_REPLAY	// render from UI log file
};
UiModes uiMode = UI_NORMAL; 
FILE *uiLog;
float *fPixelData;					// Buffer for rendering to files
unsigned char *ucPixelData;			// Buffer for rendering to files
int frameNumber = 0;				// Frame counter
int channelToShow = 1;
int iLightToChange = 1;

float **Lp = NULL;
float lightDir1[3] = {0.0,0.0,-1.0};
float lightDir2[3] = {1.0,1.0,-1.0};
float lightDir3[3] = {-1.0, 0.0, -1.0};
float normGain	= 1.0;
float ambientLight = 0.0;

PhotometricShaders shader = PS_P2NT;

CGcontext cgContext;
CGprofile cgFragmentProfile;
CgFragmentProgram *fpSources = NULL;	// Shows source images
CgFragmentProgram *fpPsPass1 = NULL;	// First pass, compute normal & albedo
CgFragmentProgram *fpPsPass2 = NULL;	// Second pass: effects!
CgFragmentProgram *fpBgMean	= NULL;
CgFragmentProgram *fpBgVar = NULL;
CgFragmentProgram *fpJustShow = NULL;
CgFragmentProgram *fpMatting = NULL;
CgFragmentProgram *fpMatMedian = NULL;
CgFragmentProgram *fpPSMatting = NULL;
CgFragmentProgram *fpJustShowDemosaic = NULL;
CgFragmentProgram *fpAlphaMatting = NULL;

GLuint	glutWindowHandle;
GLuint	srcTextureID;		// ID for texture of source images
GLuint	normAlbTexID;		// ID for the texture to render to
GLuint	twoPassFbId;			// ID for the frame buffer we'll render to -- HACK TODO: rename
GLuint	bgMeanFrmBufID;
GLuint	bgMeanTextureID;
GLuint	bgVarFrmBufID;
GLuint	bgVarTextureID;
GLuint	mattingFrmBufID;
GLuint	mattingTextureID;
GLuint	fakeBgNormalTextureID;
GLuint	fakeBgAlbedoTextureID;
const	int iBgNmbr = 4;
GLuint	fakeBgAlbedoTextureID_AM[iBgNmbr];
int		fakeBgIndx = 0;

enum playMode
{
	MODE_PS,
	MODE_MATTING,
	MODE_PS_MATTING,
	MODE_ALPHA_MATTING
};
playMode pmSelection = MODE_ALPHA_MATTING;


// THREADING
#define		NUM_GRAB_BUFFERS (3)	// triple buffering
int		numGrabBuffers = NUM_GRAB_BUFFERS;
unsigned char	*pTextureBuffers[NUM_GRAB_BUFFERS];	
unsigned char	*pAmbientBuffers[NUM_GRAB_BUFFERS];
// Grabber increments and fills grabBuffer unless that would write displayBuffer
// Display updates display buffer until displayBuffer = grabBuffer
// Just put access to these two variables in the critical section
int	g_LastGrabBuffer = 0;					// LAST GRAB BUFFER
int g_NextDisplayBuffer = numGrabBuffers-1;	// NEXT DISPLAY BUFFER
CRITICAL_SECTION	textureCriticalSection;
// gfx sleeps on this, grabber uses it to wake gfx:
CONDITION_VARIABLE	grabberConditionVariable;

bool bKillGrabberThread = false;

// PIXEL BUFFER OBJECTS  -- does not seem to help
GLuint pboBuf[2];			// Double-buffered PBOs
int pboReadingIdx = 0;		// Xfer from this to OpenGL textures
int pboWritingIdx = 1;		// Write from CPU to this buffer.
bool bUsePBOs = false;

// DEBUG TEST CODE TO GET PBO's WORKING
int numFrames = iPackedBufNmbr;
Image **srcImages;			// srcImages[fNum*numLights+lNum]
int numFrame = 0;
int frameCounter=0;
long tickCount=0;
bool bStreamVideo = true;	// Pause playback
bool bDoBgStat = true;
bool bJustShow = true;

//-----------------------------------------------------------------------------
// Global variables used in the Mil_file.
//-----------------------------------------------------------------------------
// User's processing function hook data structure.
struct HookDataStruct
{
	MIL_INT	iAmbCntOffset;
	MIL_INT ProcessedImageCount;
	MIL_INT	iBitsToShiftRight;
	bool	bDoCalibCounter;
};
HookDataStruct UserHookData, UserHookData_am;

MIL_ID	MilGrabBufList[iGrabBufNmbr];		// On-board.	64 *	pxlNmbr
MIL_ID	MilGrabBufList_am[iGrabBufNmbr_am];
// The single-band BayerImage is converted to 4 channel one.
// MIL_ID	MilBayerImage;							// On-board.	1	*	pxlNmbr
MIL_ID	MilPackedBufOnBoard;				// On-board, RGB24 format
MIL_ID	MilAmbBufOnBoard;
MIL_ID	MilChildBuf_Blue, MilChildBuf_Green, MilChildBuf_Red;
MIL_ID	MilPackedBufHost[iPackedBufNmbr];				// Off-board	iPackedBufNmbr * 3 * pxlNmbr
MIL_ID	MilAmbBufHost[iPackedBufNmbr];
MIL_ID	MilImageAcc;
MIL_ID	MilFlashAcc;
MIL_ID	MilFlashBigAcc;
MIL_ID	MilLights[iLghtNmbr * 2];			// On-board. 4	*	2 * pxlNmbr
MIL_ID	MilLightsShift[iLghtNmbr * 2];
MIL_ID	MilLights_am[iLghtNmbr_am * 2];
MIL_ID	MilDarkImage[iDarkBufNmbr];			// Off-board is enough
MIL_ID	MilDarkAcc;
MIL_ID	MilDarkAveOnBoard;
MIL_UINT	MilPackedBufHostAddr;
MIL_UINT	MilAmbBufHostAddr;
static int nLightIdx=0;
//static int nCalledCount=0;
MIL_UINT8	pucFrmCntBytes[8]; 
unsigned char *ppMilPackedBuf[iPackedBufNmbr];
unsigned char *ppMilAmbBufHost[iPackedBufNmbr];
static int	iPackedBufIndx = 0;

//-----------------------------------------------------------------------------
// All declarations of functions.
//-----------------------------------------------------------------------------

class ExitException : public Exception
{
public:
    ExitException(const string & message);
};

ExitException::ExitException(const string & message) : Exception(message)
{
    // nothing to do
	exit(0);
}


void cleanup(){
	int i;

	delete(fpPsPass1);
	delete(fpPsPass2);
	CgFragmentProgram::Cleanup();
	if(NULL!=Lp){
		for(i=0;i<NUM_LIGHTS;i++){
			delete(Lp[i]);
		}
		delete(Lp);
	}

	if(UI_LOG == uiMode){fclose(uiLog);}
	if(UI_REPLAY == uiMode){
		fclose(uiLog);
		free(fPixelData);
		free(ucPixelData);
	}

}

void Keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27:
        throw ExitException("exit");
        break;
	case 'q':
		cleanup();
		throw ExitException("exit");
        break;
	//case 'r':  // Toggle shader
	//	DecrementShader();
	//	break;
	//case 't':  // Toggle shader
	//	IncrementShader();
	//	break;
	case 'a': // Go to previous sources images
		//sourceSet = (sourceSet<=0)? 3 : sourceSet-1;
		//pCgPsPass1->SetTextures(2*sourceSet,1+2*sourceSet);
		//printf("Source is image set: %d\n",sourceSet);
		pmSelection = MODE_ALPHA_MATTING;
		if(bDoBgStat == true){
			numFrame = 0;
		}
		printf("MODE: Alpha Matting!\n");
		break;
	case 'm': // Enter the Matting mode.
		pmSelection = MODE_MATTING;
		if(bDoBgStat == true){
			numFrame = 0;
		}
		printf("MODE: Matting!\n");
		break;
	case 'p': // Enter the photomatric stereo mode.
		pmSelection = MODE_PS;
		printf("MODE: Photometric Stereo!\n");
		break;
	case 't':
		if(bDoBgStat == true){
			numFrame = 0;
		}
		printf("MODE: Photometric Stereo + Matting + Fake Background!\n");
		pmSelection = MODE_PS_MATTING;
		break;
	case 'c':
		bDoBgStat = true;
		break;
	case ' ':	// Increment view number for source image display
		bStreamVideo = !bStreamVideo;
		break;
	case 's':
		bJustShow = !bJustShow;
		if(bJustShow)
		{
			printf("bJustShow is TRUE!\n");
		}
		else
		{
			printf("bJustShow is FALSE!\n");
		}
		break;
	case '1':
		channelToShow = 1;
		iLightToChange = 1;
		break;
	case '2':
		channelToShow = 2;
		iLightToChange = 2;
		break;
	case '3':
		channelToShow = 3;
		iLightToChange = 3;
		break;
	case '4':
		channelToShow = 4;
		break;
	case '5':
		UserHookData.iBitsToShiftRight = 1;	// Subtracting 1/2
		break;
	case '6':
		UserHookData.iBitsToShiftRight = 2;	// Subtracting 1/4
		break;
	case '7':
		UserHookData.iBitsToShiftRight = 3;	// Subtracting 1/8
		break;
	case '8':
		UserHookData.iBitsToShiftRight = 0;
		break;
	default:
        break;
    }
    glutPostRedisplay();
}

// Update lightDir when user clicks and drags in display window. 
// 
void Motion(int x, int y){
	float* lightVecPtr;

	switch(mouseButtonState){
	case(PS_LEFTBUTTON):
		switch(iLightToChange)
		{
		case 1:
			lightVecPtr = lightDir1;
			break;
		case 2:
			lightVecPtr = lightDir2;
			break;
		case 3:
			lightVecPtr = lightDir3;
			break;
		default:
			break;
		}
		*(lightVecPtr+0) = (x-320.0)/320.0;
		*(lightVecPtr+1) = (y-240.0)/240.0;
		*(lightVecPtr+2) = -1.0;
		//lLength = sqrt(lightDir[0]*lightDir[0] + lightDir[1]*lightDir[1] + 1);
		//lightDir[0] = lightDir[0]/lLength;
		//lightDir[1] = lightDir[1]/lLength;
		//lightDir[2] = lightDir[2]/lLength;
		//printf("Light Direction: %f %f %f\n", lightDir[0], lightDir[1], lightDir[2]);
		//printf("Mouse coords: %d %d\n", x, y);
		glutPostRedisplay();
		break;
	case(PS_MIDDLEBUTTON):
		//pCgPsPhong->SetSpecularExponent((480.0-y)/20.0);
		ambientLight = (480.0 - (float)y)/480.0;
		break;
	case(PS_RIGHTBUTTON):
		normGain = (480.0-(float)y)/100.0; // Range from 1 to 6
		//normGain = (normGain<0.0)?0.0:normGain;
	    glutPostRedisplay();
		break;
	}
}

// Tracks mouse button state
void Mouse(int button, int state, int x, int y){
	switch(button){
	case(GLUT_LEFT_BUTTON):
		if(state==GLUT_UP) {
			mouseButtonState &= ~(PS_LEFTBUTTON);
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		if(state==GLUT_DOWN) {
			mouseButtonState |= PS_LEFTBUTTON;
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		break;
	case(GLUT_MIDDLE_BUTTON):
		if(state==GLUT_UP) {
			mouseButtonState &= ~(PS_MIDDLEBUTTON);
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		if(state==GLUT_DOWN) {
			mouseButtonState |= PS_MIDDLEBUTTON;
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		break;
	case(GLUT_RIGHT_BUTTON):
		if(state==GLUT_UP) {
			mouseButtonState &= ~(PS_RIGHTBUTTON);
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		if(state==GLUT_DOWN) {
			mouseButtonState |= PS_RIGHTBUTTON;
			//printf("Mouse state: %d\n",mouseButtonState);
		}
		break;
	}
}

void Idle()
{
	int nextDisplayBuffer;
	bool bUpdateTexture = false;

	if(bStreamVideo){
		EnterCriticalSection(&textureCriticalSection);
		nextDisplayBuffer = (g_NextDisplayBuffer+1 == numGrabBuffers) ? 
			0 : g_NextDisplayBuffer+1;
		// Xfer new image unless I've caught the grabber
		bUpdateTexture = !(nextDisplayBuffer==g_LastGrabBuffer);
		if(!bUpdateTexture){
			SleepConditionVariableCS(&grabberConditionVariable,
				&textureCriticalSection,INFINITE);
		}
		LeaveCriticalSection(&textureCriticalSection);

		if(bUpdateTexture){
			glutPostRedisplay();
		}
	}
}


// Error checking routine
void checkGLErrors(const char *label) {
    GLenum errCode;
    const GLubyte *errStr;
    if ((errCode = glGetError()) != GL_NO_ERROR) {
        errStr = gluErrorString(errCode);
        printf("OpenGL ERROR: ");
        printf((char*)errStr);
        printf("(Label: ");
        printf(label);
        printf(")\n.");
    }
}

// BSW HACK DEBUG TODO get rid of this later
// Compare textures
//bool areIdenticalTextures(float *a, float *b, int w, int h){
//	int x,y;
//
//	for(y=0;y<h;y++){
//		for(x=0;x<w;x++){
//			if(a[y*w+h] != b[y*w+h]){return false;}
//		}
//	}
//
//	return true;
//}


/**
 * Checks framebuffer status.
 * Copied directly out of the spec, modified to deliver a return value.
 */
bool checkFramebufferStatus() {
    GLenum status;
    status = (GLenum) glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    switch(status) {
        case GL_FRAMEBUFFER_COMPLETE_EXT:
            return true;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
			printf("Framebuffer incomplete, incomplete attachment\n");
            return false;
        case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
			printf("Unsupported framebuffer format\n");
            return false;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
			printf("Framebuffer incomplete, missing attachment\n");
            return false;
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
			printf("Framebuffer incomplete, attached images must have same dimensions\n");
            return false;
        case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
			printf("Framebuffer incomplete, attached images must have same format\n");
            return false;
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
			printf("Framebuffer incomplete, missing draw buffer\n");
            return false;
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
			printf("Framebuffer incomplete, missing read buffer\n");
            return false;
    }
	return false;
}

void InitAndBindFboTexPair(GLuint &fboId, GLuint &texId, GLuint attachPoint){
	// Initialize the texture to hold results of first rendering pass.
	// First, create a new texture name
	glGenTextures (1, &texId);
	checkGLErrors("Generated texture\n");
	// bind the texture name to a texture target
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,texId);
	// turn off filtering and set proper wrap mode 
	// (obligatory for float textures atm)
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);

	// Allocate texture graphics memory.
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,		// Texture target (format)
		0,										// No mipmap levels
		GL_FLOAT_RGBA32_NV,						// Internal format -- GL_FLOAT_RGBA32_NV/GL_FLOAT_R32_NV4 
        WIDTH,									// Texture width
		HEIGHT,									// Texture height
		0,										// Turns off borders
		GL_RGBA,								// Texture format -- GL_RGBA/GL_LUMINANCE
		GL_FLOAT,								// Tell GL that we'll be passing in floating point data
		0										// NULL if not specifying data now
		);
	checkGLErrors("Allocated texture\n");

	// Create off-screen frame buffer object
    glGenFramebuffersEXT(1, &fboId); 
	// Bind off-screen framebuffer object (FBO)
	// Note binding with twoPassFbId=0 restores the window system 
	// specific frame buffer target. Good to know...
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboId);
	// Attach texture to frame buffer object.
	glFramebufferTexture2DEXT(
		GL_FRAMEBUFFER_EXT,						// obligatory
		attachPoint,							// Attachment point (up to 4! query with GL_MAX_COLOR_ATTACHMENTS_EXT)
		GL_TEXTURE_RECTANGLE_ARB,				// Texture target 
		texId,									// Texture Id to attach
		0										// Mipmap level, 0 because we don't use it
		);
	checkGLErrors("Allocated texture\n");
	checkFramebufferStatus();
	checkGLErrors("Allocated texture\n");
}

void CleanUpTwoPassRendering(){
	glDeleteFramebuffersEXT(1,&twoPassFbId);
	glDeleteTextures(1,&normAlbTexID);
}


void UpdateUiLog(){
	//fprintf_s(uiLog,"lightDir: %f %f %f\n",lightDir[0],lightDir[1],lightDir[2]);
	//fprintf_s(uiLog,"shader: %d\n",shader);
	//fprintf_s(uiLog,"normGain: %f\n",normGain);
	fprintf_s(uiLog,"%f %f %f\n",lightDir1[0],lightDir1[1],lightDir1[2]);
	fprintf_s(uiLog,"%d\n",shader);
	fprintf_s(uiLog,"%f\n",normGain);
}

void UpdateFromUiLog(){

}

// First take at rendering in two passes. 
// 1. Compute normals and albedo.
// 2. Compute average normal and albedo.
// 3. Experiment.
//		- Subtract average and render
//      - Perturb relative to local average, saturate. 
//      - Could amplify albedo, too.
// 
// Note
// 1. Textures switch from read only to write only,
//    so you have to reassign them after they've been written.
// 2. Remember to clear the buffers when you have float4 data!
// 

void TwoPassRender(void){

	bool bUpdateTexture = false;
	int nextDisplayBuffer = 0;

	// viewport transform for 1:1 pixel=texel=data mapping
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0,WIDTH,0.0,HEIGHT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glViewport(0,0,WIDTH,HEIGHT);

	if(bStreamVideo){
		// Update textures
		if(!bUsePBOs){
			// WITHOUT PBO 
			EnterCriticalSection(&textureCriticalSection);
			nextDisplayBuffer = (g_NextDisplayBuffer+1 == numGrabBuffers) ? 
					0 : g_NextDisplayBuffer+1;
			// Xfer new image unless I've caught the grabber
			bUpdateTexture = !(nextDisplayBuffer==g_LastGrabBuffer);
			LeaveCriticalSection(&textureCriticalSection);

			if(bUpdateTexture){
				glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
				glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
					GL_RGBA, GL_UNSIGNED_BYTE, pTextureBuffers[g_LastGrabBuffer]);
				checkGLErrors("load4Textures\n");
			}

			// Update the next display buffer
			EnterCriticalSection(&textureCriticalSection);
			g_NextDisplayBuffer = nextDisplayBuffer;
			LeaveCriticalSection(&textureCriticalSection);
		}
		else {// WITH PBO
			pboReadingIdx = (pboReadingIdx + 1) % 2;
			pboWritingIdx = (pboReadingIdx + 1) % 2;
			glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
			glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pboBuf[pboReadingIdx]);
			glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
				GL_RGBA, GL_UNSIGNED_BYTE, 0);
			glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pboBuf[pboWritingIdx]);
			glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, WIDTH*HEIGHT*4, 0, GL_STREAM_DRAW_ARB);
			GLubyte* ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB,
				GL_WRITE_ONLY_ARB);
			if(ptr)
			{
				Image *pImg = srcImages[numFrame];
				// XXX: to test, copy from system memory
				memcpy(ptr,pImg->pData,WIDTH*HEIGHT*4);
				glUnmapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB); // release the mapped buffer
			}
			// Bind to zero to release
			glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
		}
		// Update frame number
		numFrame = (numFrame+1 == iPackedBufNmbr) ? 0 : numFrame+1;
	}	

	// Pass 1
	// Render normals and albedo into floating point RGBA texture
	// Enable normal+albedo shader
	fpPsPass1->EnableAndBind();
	fpPsPass1->SetParam3fv("Lp0",Lp[0]);
	fpPsPass1->SetParam3fv("Lp1",Lp[1]);
	fpPsPass1->SetParam3fv("Lp2",Lp[2]);
	fpPsPass1->SetTextureParam("view0123",srcTextureID);
	fpPsPass1->EnableTextureParam("view0123");

	// Set the FBO as render target
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, twoPassFbId);
	checkFramebufferStatus(); 

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glClear(GL_COLOR_BUFFER_BIT);

	glPolygonMode(GL_FRONT,GL_FILL);
	// make quad filled to hit every pixel/texel
	glBegin(GL_QUADS);
	// Invert so texture is right side up
	glTexCoord2f(0.0, HEIGHT); 
	glVertex2f(0.0, 0.0);
	glTexCoord2f(WIDTH, HEIGHT); 
	glVertex2f(WIDTH, 0.0);
	glTexCoord2f(WIDTH, 0.0); 
	glVertex2f(WIDTH, HEIGHT);
	glTexCoord2f(0.0, 0.0); 
	glVertex2f(0.0, HEIGHT);
	glEnd();

	// Second pass -- render albedo
	fpPsPass1->DisableTextureParam("view0123");
	fpPsPass1->Disable();
	fpPsPass2->EnableAndBind();
	fpPsPass2->SetParam3fv("Ldir",lightDir1);
	fpPsPass2->SetParam1f("gain",normGain);
	fpPsPass2->SetParam1f("ambLighting", ambientLight);
	fpPsPass2->SetTextureParam("normAndAlbedo",normAlbTexID);
	fpPsPass2->EnableTextureParam("normAndAlbedo");

	// Set the back color buffer as render target
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);		// render to frame buffer now
	//Not necessary: glDrawBuffer(GL_BACK_LEFT);

	// Pass 2
	// Render normals and albedo into floating point RGBA texture
	// make quad filled to hit every pixel/texel
    glClear(GL_COLOR_BUFFER_BIT);
	glPolygonMode(GL_FRONT,GL_FILL);
	// and render quad
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0); 
	glVertex2f(0.0, 0.0);
	glTexCoord2f(WIDTH, 0.0); 
	glVertex2f(WIDTH, 0.0);
	glTexCoord2f(WIDTH, HEIGHT); 
	glVertex2f(WIDTH, HEIGHT);
	glTexCoord2f(0.0, HEIGHT); 
	glVertex2f(0.0, HEIGHT);
	glEnd();

	fpPsPass2->DisableTextureParam("normAndAlbedo");
	fpPsPass2->Disable();
}

void AlphaMatting()
{
	bool bUpdateTexture = false;
	int nextDisplayBuffer = 0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, WIDTH, 0.0, HEIGHT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glViewport(0, 0, WIDTH, HEIGHT);
	//glViewport(0, 0, 1028, 1024);
	// We have to combine the ambient data into the alpha channel first.
	if(bStreamVideo)
	{
		EnterCriticalSection(&textureCriticalSection);
		nextDisplayBuffer = (g_NextDisplayBuffer+1 == numGrabBuffers) ? 
				0 : g_NextDisplayBuffer+1;
		// Xfer new image unless I've caught the grabber
		bUpdateTexture = !(nextDisplayBuffer==g_LastGrabBuffer);
		LeaveCriticalSection(&textureCriticalSection);

		if(bUpdateTexture){
			glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
			glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
				GL_RGBA, GL_UNSIGNED_BYTE, pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Textures\n");
		}
		// Update the next display buffer
		EnterCriticalSection(&textureCriticalSection);
		g_NextDisplayBuffer = nextDisplayBuffer;
		LeaveCriticalSection(&textureCriticalSection);

		if(bDoBgStat == true)
		{
			glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
			glTexSubImage2D(
				GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
				pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Testures\n");

			fpBgMean->EnableAndBind();
			fpBgMean->SetParam1f("frmProcessed", (float)numFrame);
			fpBgMean->SetTextureParam("newFrame", srcTextureID);
			fpBgMean->EnableTextureParam("newFrame");
			fpBgMean->SetTextureParam("bgMean", bgMeanTextureID);
			fpBgMean->EnableTextureParam("bgMean");
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, bgMeanFrmBufID);
			checkFramebufferStatus();
			glPolygonMode(GL_FRONT, GL_FILL);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0); 
				glVertex2f(0.0, 0.0);
				glTexCoord2f(WIDTH, 0.0); 
				glVertex2f(WIDTH, 0.0);
				glTexCoord2f(WIDTH, HEIGHT);
				glVertex2f(WIDTH, HEIGHT);
				glTexCoord2f(0.0, HEIGHT);
				glVertex2f(0.0, HEIGHT);
			glEnd();
			fpBgMean->DisableTextureParam("newFrame");
			fpBgMean->DisableTextureParam("bgMean");
			fpBgMean->Disable();
			numFrame++;
		} // End: if(bDoBgStat == true)
		else
		{
			if (bJustShow == true)
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);

				fpJustShow->EnableAndBind();
				fpJustShow->SetParam1i("channel", channelToShow);
				//fpJustShow->SetTextureParam("bgMean", srcTextureID);
				fpJustShow->SetTextureParam("texToShow", srcTextureID);
				fpJustShow->EnableTextureParam("texToShow");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mattingFrmBufID);
				checkFramebufferStatus();
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, 0.0); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, 0.0); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, HEIGHT);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, HEIGHT);
					glVertex2f(0.0, HEIGHT);
				glEnd();	
				fpJustShow->DisableTextureParam("texToShow");
				fpJustShow->Disable();

				fpJustShowDemosaic->EnableAndBind();
				fpJustShowDemosaic->SetTextureParam("texToShow", mattingTextureID);
				fpJustShowDemosaic->EnableTextureParam("texToShow");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
				checkFramebufferStatus();

				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpJustShowDemosaic->DisableTextureParam("texToShow");
				fpJustShowDemosaic->Disable();
			}//End: if(bJustShow == true)
			else // Do the matting.
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);
				checkGLErrors("load4Testures\n");

				fpAlphaMatting->EnableAndBind();
				fpAlphaMatting->SetTextureParam("bgMean", bgMeanTextureID);
				fpAlphaMatting->EnableTextureParam("bgMean");
				fpAlphaMatting->SetTextureParam("newFrame", srcTextureID);
				fpAlphaMatting->EnableTextureParam("newFrame");
				fpAlphaMatting->SetTextureParam("fakeBg", fakeBgAlbedoTextureID_AM[fakeBgIndx]);
				fpAlphaMatting->EnableTextureParam("fakeBg");			
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
				checkFramebufferStatus();

				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpAlphaMatting->DisableTextureParam("bgMean");
				fpAlphaMatting->DisableTextureParam("newFrame");	
				fpAlphaMatting->DisableTextureParam("fakeBg");				
				fpAlphaMatting->Disable();

			}//End: if(bJustShow == true) esle{}
		}// End: if(bDoBgStat == true) else()
	}//End: if(bStreamVideo)
}

void Matting()
{	
	bool bUpdateTexture = false;
	int nextDisplayBuffer = 0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, WIDTH, 0.0, HEIGHT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glViewport(0, 0, WIDTH, HEIGHT);
	// We have to combine the ambient data into the alpha channel first.
	if(bStreamVideo)
	{
		EnterCriticalSection(&textureCriticalSection);
		nextDisplayBuffer = (g_NextDisplayBuffer+1 == numGrabBuffers) ? 
				0 : g_NextDisplayBuffer+1;
		// Xfer new image unless I've caught the grabber
		bUpdateTexture = !(nextDisplayBuffer==g_LastGrabBuffer);
		LeaveCriticalSection(&textureCriticalSection);

		if(bUpdateTexture){
			unsigned char* dst = pTextureBuffers[g_LastGrabBuffer]+3;
			unsigned char* src = pAmbientBuffers[g_LastGrabBuffer];
			for (int i=0; i<WIDTH*HEIGHT; i++)
			{
				*dst = *src++;
				dst = dst + 4;
			}	
			glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
			glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
				GL_RGBA, GL_UNSIGNED_BYTE, pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Textures\n");
		}
		// Update the next display buffer
		EnterCriticalSection(&textureCriticalSection);
		g_NextDisplayBuffer = nextDisplayBuffer;
		LeaveCriticalSection(&textureCriticalSection);

		if(bDoBgStat == true)
		{
			glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
			glTexSubImage2D(
				GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
				pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Testures\n");
			
			fpBgVar->EnableAndBind();
			fpBgVar->SetParam1f("frmProcessed", (float)numFrame);
			fpBgVar->SetTextureParam("newFrame", srcTextureID);
			fpBgVar->EnableTextureParam("newFrame");
			fpBgVar->SetTextureParam("bgMean", bgMeanTextureID);
			fpBgVar->EnableTextureParam("bgMean");
			fpBgVar->SetTextureParam("bgVar", bgVarTextureID);
			fpBgVar->EnableTextureParam("bgVar");
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, bgVarFrmBufID);
			checkFramebufferStatus();
			//glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

			glPolygonMode(GL_FRONT, GL_FILL);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0);
				glVertex2f(0.0, 0.0);
				glTexCoord2f(WIDTH, 0.0); 
				glVertex2f(WIDTH, 0.0);
				glTexCoord2f(WIDTH, HEIGHT);
				glVertex2f(WIDTH, HEIGHT);
				glTexCoord2f(0.0, HEIGHT);
				glVertex2f(0.0, HEIGHT);
			glEnd();
			fpBgVar->DisableTextureParam("newFrame");
			fpBgVar->DisableTextureParam("bgMean");
			fpBgVar->DisableTextureParam("bgVar");
			fpBgVar->Disable();

			fpBgMean->EnableAndBind();
			fpBgMean->SetParam1f("frmProcessed", (float)numFrame);
			fpBgMean->SetTextureParam("newFrame", srcTextureID);
			fpBgMean->EnableTextureParam("newFrame");
			fpBgMean->SetTextureParam("bgMean", bgMeanTextureID);
			fpBgMean->EnableTextureParam("bgMean");
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, bgMeanFrmBufID);
			checkFramebufferStatus();
			glPolygonMode(GL_FRONT, GL_FILL);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0); 
				glVertex2f(0.0, 0.0);
				glTexCoord2f(WIDTH, 0.0); 
				glVertex2f(WIDTH, 0.0);
				glTexCoord2f(WIDTH, HEIGHT);
				glVertex2f(WIDTH, HEIGHT);
				glTexCoord2f(0.0, HEIGHT);
				glVertex2f(0.0, HEIGHT);
			glEnd();
			fpBgMean->DisableTextureParam("newFrame");
			fpBgMean->DisableTextureParam("bgMean");
			fpBgMean->Disable();
			numFrame++;
		} // End: if(bDoBgStat == true)
		else
		{
			if (bJustShow == true)
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);

				fpJustShow->EnableAndBind();
				fpJustShow->SetParam1i("channel", channelToShow);
				//fpJustShow->SetTextureParam("bgMean", srcTextureID);
				fpJustShow->SetTextureParam("texToShow", srcTextureID);
				fpJustShow->EnableTextureParam("texToShow");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mattingFrmBufID);
				checkFramebufferStatus();
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, 0.0); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, 0.0); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, HEIGHT);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, HEIGHT);
					glVertex2f(0.0, HEIGHT);
				glEnd();	
				fpJustShow->DisableTextureParam("texToShow");
				fpJustShow->Disable();

				fpJustShowDemosaic->EnableAndBind();
				fpJustShowDemosaic->SetTextureParam("texToShow", mattingTextureID);
				fpJustShowDemosaic->EnableTextureParam("texToShow");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
				checkFramebufferStatus();

				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpJustShowDemosaic->DisableTextureParam("texToShow");
				fpJustShowDemosaic->Disable();
			}//End: if(bJustShow == true)
			else // Do the matting.
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);
				checkGLErrors("load4Testures\n");

				fpMatting->EnableAndBind();
				fpMatting->SetTextureParam("bgMean", bgMeanTextureID);
				fpMatting->EnableTextureParam("bgMean");
				fpMatting->SetTextureParam("bgVar", bgVarTextureID);
				fpMatting->EnableTextureParam("bgVar");
				fpMatting->SetTextureParam("newFrame", srcTextureID);
				fpMatting->EnableTextureParam("newFrame");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mattingFrmBufID);
				checkFramebufferStatus();
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, 0.0); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, 0.0); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, HEIGHT);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, HEIGHT);
					glVertex2f(0.0, HEIGHT);
				glEnd();

				fpMatting->DisableTextureParam("bgMean");
				fpMatting->DisableTextureParam("bgVar");
				fpMatting->DisableTextureParam("newFrame");
				fpMatting->Disable();

				fpMatMedian->EnableAndBind();
				fpMatMedian->SetTextureParam("maskFrame", mattingTextureID);
				fpMatMedian->EnableTextureParam("maskFrame");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);	
				checkFramebufferStatus();
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpMatMedian->DisableTextureParam("maskFrame");
				fpMatMedian->Disable();

			}//End: if(bJustShow == true) esle{}
		}// End: if(bDoBgStat == true) else()
	}//End: if(bStreamVideo)
}

void PSMatting()
{	
	bool bUpdateTexture = false;
	int nextDisplayBuffer = 0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, WIDTH, 0.0, HEIGHT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glViewport(0, 0, WIDTH, HEIGHT);

	if(bStreamVideo)
	{		
		EnterCriticalSection(&textureCriticalSection);
		nextDisplayBuffer = (g_NextDisplayBuffer+1 == numGrabBuffers) ? 
				0 : g_NextDisplayBuffer+1;
		// Xfer new image unless I've caught the grabber
		bUpdateTexture = !(nextDisplayBuffer==g_LastGrabBuffer);
		LeaveCriticalSection(&textureCriticalSection);

		if(bUpdateTexture){
			// We have to combine the ambient data into the alpha channel first.
			unsigned char* dst = pTextureBuffers[g_LastGrabBuffer]+3;
			unsigned char* src = pAmbientBuffers[g_LastGrabBuffer];
			for (int i=0; i<WIDTH*HEIGHT; i++)
			{
				*dst = *src++;
				dst = dst + 4;
			}
			glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
			glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
				GL_RGBA, GL_UNSIGNED_BYTE, pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Textures\n");
		}
		// Update the next display buffer
		EnterCriticalSection(&textureCriticalSection);
		g_NextDisplayBuffer = nextDisplayBuffer;
		LeaveCriticalSection(&textureCriticalSection);

		if(bDoBgStat == true)
		{
			glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
			glTexSubImage2D(
				GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
				pTextureBuffers[g_LastGrabBuffer]);
			checkGLErrors("load4Testures\n");
			
			fpBgVar->EnableAndBind();
			fpBgVar->SetParam1f("frmProcessed", (float)numFrame);
			fpBgVar->SetTextureParam("newFrame", srcTextureID);
			fpBgVar->EnableTextureParam("newFrame");
			fpBgVar->SetTextureParam("bgMean", bgMeanTextureID);
			fpBgVar->EnableTextureParam("bgMean");
			fpBgVar->SetTextureParam("bgVar", bgVarTextureID);
			fpBgVar->EnableTextureParam("bgVar");
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, bgVarFrmBufID);
			checkFramebufferStatus();
			//glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

			glPolygonMode(GL_FRONT, GL_FILL);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0);
				glVertex2f(0.0, 0.0);
				glTexCoord2f(WIDTH, 0.0); 
				glVertex2f(WIDTH, 0.0);
				glTexCoord2f(WIDTH, HEIGHT);
				glVertex2f(WIDTH, HEIGHT);
				glTexCoord2f(0.0, HEIGHT);
				glVertex2f(0.0, HEIGHT);
			glEnd();
			fpBgVar->DisableTextureParam("newFrame");
			fpBgVar->DisableTextureParam("bgMean");
			fpBgVar->DisableTextureParam("bgVar");
			fpBgVar->Disable();

			fpBgMean->EnableAndBind();
			fpBgMean->SetParam1f("frmProcessed", (float)numFrame);
			fpBgMean->SetTextureParam("newFrame", srcTextureID);
			fpBgMean->EnableTextureParam("newFrame");
			fpBgMean->SetTextureParam("bgMean", bgMeanTextureID);
			fpBgMean->EnableTextureParam("bgMean");
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, bgMeanFrmBufID);
			checkFramebufferStatus();

			glPolygonMode(GL_FRONT, GL_FILL);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0); 
				glVertex2f(0.0, 0.0);
				glTexCoord2f(WIDTH, 0.0); 
				glVertex2f(WIDTH, 0.0);
				glTexCoord2f(WIDTH, HEIGHT);
				glVertex2f(WIDTH, HEIGHT);
				glTexCoord2f(0.0, HEIGHT);
				glVertex2f(0.0, HEIGHT);
			glEnd();
			fpBgMean->DisableTextureParam("newFrame");
			fpBgMean->DisableTextureParam("bgMean");
			fpBgMean->Disable();
			numFrame++;
		} // End: if(bDoBgStat == true)
		else
		{
			if (bJustShow == true)
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);

				fpJustShow->EnableAndBind();
				fpJustShow->SetParam1i("channel", channelToShow);
				fpJustShow->SetTextureParam("texToShow", srcTextureID);
				fpJustShow->EnableTextureParam("texToShow");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
				checkFramebufferStatus();
				
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpJustShow->DisableTextureParam("texToShow");
				fpJustShow->Disable();
			}//End: if(bJustShow == true)
			else // Do the PS + Matting.
			{
				glBindTexture(GL_TEXTURE_RECTANGLE_NV, srcTextureID);
				glTexSubImage2D(
					GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 
					pTextureBuffers[g_LastGrabBuffer]);
				checkGLErrors("load4Testures\n");
				//----------------------------------
				// Matting Mask: first pass
				//----------------------------------
				fpMatting->EnableAndBind();
				fpMatting->SetTextureParam("bgMean", bgMeanTextureID);
				fpMatting->EnableTextureParam("bgMean");
				fpMatting->SetTextureParam("bgVar", bgVarTextureID);
				fpMatting->EnableTextureParam("bgVar");
				fpMatting->SetTextureParam("newFrame", srcTextureID);
				fpMatting->EnableTextureParam("newFrame");
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mattingFrmBufID);
				checkFramebufferStatus();
				glPolygonMode(GL_FRONT, GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, 0.0); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, 0.0); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, HEIGHT);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, HEIGHT);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpMatting->DisableTextureParam("bgMean");
				fpMatting->DisableTextureParam("bgVar");
				fpMatting->DisableTextureParam("newFrame");
				fpMatting->Disable();

				//------------------------------------------
				// PS Rendering: Pass 1
				// Render normals and albedo into floating point RGBA texture
				// ------------------------------------------
				fpPsPass1->EnableAndBind();
				fpPsPass1->SetParam3fv("Lp0",Lp[0]);
				fpPsPass1->SetParam3fv("Lp1",Lp[1]);
				fpPsPass1->SetParam3fv("Lp2",Lp[2]);
				fpPsPass1->SetTextureParam("view0123",srcTextureID);
				fpPsPass1->EnableTextureParam("view0123");
				// Set the FBO as render target
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, twoPassFbId);
				checkFramebufferStatus(); 
				glPolygonMode(GL_FRONT,GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, 0.0); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, 0.0); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, HEIGHT);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, HEIGHT);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpPsPass1->DisableTextureParam("view0123");
				fpPsPass1->Disable();

				//--------------------------------
				// Second pass -- render albedo and masking.
				//--------------------------------
				fpPSMatting->EnableAndBind();
				//fpPSMatting->SetParam3fv("Ldir1",lightDir1);
				//fpPSMatting->SetParam3fv("Ldir2",lightDir2);
				//fpPSMatting->SetParam3fv("Ldir3",lightDir3);
				fpPSMatting->SetParam3fv("Ldir", lightDir1);
				fpPSMatting->SetParam1f("gain", normGain);
				fpPSMatting->SetParam1f("ambLighting", ambientLight);
				fpPSMatting->SetTextureParam("normAndAlbedo",normAlbTexID);
				fpPSMatting->EnableTextureParam("normAndAlbedo");
				fpPSMatting->SetTextureParam("maskFrame", mattingTextureID);
				fpPSMatting->EnableTextureParam("maskFrame");
				fpPSMatting->SetTextureParam("fakeBgNormal", fakeBgNormalTextureID);
				fpPSMatting->EnableTextureParam("fakeBgNormal");
				fpPSMatting->SetTextureParam("fakeBgAlbedo", fakeBgAlbedoTextureID);
				fpPSMatting->EnableTextureParam("fakeBgAlbedo");
				// Set the back color buffer as render target
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);		// render to frame buffer now
				glPolygonMode(GL_FRONT,GL_FILL);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0, HEIGHT); 
					glVertex2f(0.0, 0.0);
					glTexCoord2f(WIDTH, HEIGHT); 
					glVertex2f(WIDTH, 0.0);
					glTexCoord2f(WIDTH, 0);
					glVertex2f(WIDTH, HEIGHT);
					glTexCoord2f(0.0, 0.0);
					glVertex2f(0.0, HEIGHT);
				glEnd();
				fpPSMatting->DisableTextureParam("normAndAlbedo");
				fpPSMatting->DisableTextureParam("maskFrame");
				fpPSMatting->DisableTextureParam("fakeBgNormal");
				fpPSMatting->DisableTextureParam("fakeBgAlbedo");
				fpPSMatting->Disable();
			}//End: if(bJustShow == true) esle{}
		}// End: if(bDoBgStat == true) else()
	}//End: if(bStreamVideo)
}

//Writes frame buffer to a file.
void WriteFrame(){
	FILE *ofid;
	char oFileName[256];
	int x, y;
	unsigned char r,g,b;
	float *pfData = fPixelData;
	unsigned char *pucData;

	for(y=0;y<windowH;y++){
		pucData = ucPixelData + (windowH-1-y)*3*windowW;	// Set pointer to start of row
		for(x=0;x<windowW;x++){
			r = (unsigned char)(255.0 * (*pfData++));
			g = (unsigned char)(255.0 * (*pfData++));
			b = (unsigned char)(255.0 * (*pfData++));
			pfData++;
			*pucData++ = r;
			*pucData++ = g;
			*pucData++ = b;
		}
	}
	// Generate output filename
	sprintf_s(oFileName,256,OUTFILE_TMPL,frameNumber++);
	printf("%s\n",oFileName);
	// Open new file
	fopen_s(&ofid,oFileName,"wb");
	fprintf(ofid,"P6\n%d %d\n255 ",windowW,windowH); // Raw three color PPM
	fwrite(ucPixelData,sizeof(unsigned char),windowW*windowH*3,ofid);
	// close new file
	fclose(ofid);
}

void Display(void)
{
	long currentTick;
	float fps;
    // Non-standard user mode: log or grab user interaction from log file
	switch(uiMode){
	case UI_LOG:
		UpdateUiLog();
		break;
	case UI_REPLAY:
		UpdateFromUiLog();
		break;
	default:
		break;
	}
	
	switch(pmSelection)
	{
	case MODE_PS:
		TwoPassRender();
		break;
	case MODE_MATTING:
		if ( (bDoBgStat == true)&&(numFrame < iBgStatFrmNmbr) )
		{
			printf("Number of Frames that have been processed: %d.\n", numFrame);	
		}
		else
		{
			bDoBgStat = false;
		}
		Matting();
		break;
	case MODE_PS_MATTING:
		if ( (bDoBgStat == true)&&(numFrame < iBgStatFrmNmbr) )
		{
			printf("Number of Frames that have been processed: %d.\n", numFrame);	
		}
		else
		{
			bDoBgStat = false;
		}
		PSMatting();
		break;
	case MODE_ALPHA_MATTING:
		if ( (bDoBgStat == true)&&(numFrame < iBgStatFrmNmbr) )
		{
			printf("Number of Frames that have been processed: %d.\n", numFrame);	
		}
		else
		{
			bDoBgStat = false;
		}
		AlphaMatting();
	default:
		break;
	}
	
	if(UI_REPLAY==uiMode){
		glReadPixels(0,0,windowW,windowH,GL_RGBA,GL_FLOAT,fPixelData);
		WriteFrame();
	}

    glutSwapBuffers();

	frameCounter++;
	if(frameCounter==100){
		fakeBgIndx = (fakeBgIndx == (iBgNmbr-1))? 0 : fakeBgIndx+1;
		currentTick = GetTickCount();
		fps = 100.0f/((currentTick-tickCount)/1000.0f);
		printf("fps: %f\r",fps);
		tickCount=currentTick;
		frameCounter=0;
	}

}

float** loadLPrimeTransposeFromFile(char *lPrimeFileName, float **Lp, int numLights){
	FILE* fid;
	int i;

	if(NULL!=fopen_s(&fid,lPrimeFileName,"r"))
		throw Exception("(loadLPrimeFromFile):Could not open Lprime file.");

	if(NULL==Lp){	
		Lp = new float*[numLights];
	}
	for(i=0;i<numLights;i++){
		Lp[i] = new float[3];
		fscanf_s(fid,"%f %f %f",Lp[i],Lp[i]+1,Lp[i]+2);
	}
	fclose(fid);
	return Lp;
}


// XXX: hack to test PBOs
Image **loadImages(int numFrames, int numLights, char *fileTmpl ){
	char inFileName[256];
	int fNum,lNum;
	Image imgs[4];
	Image tmpImg;
	Image **pImages;

	pImages = new Image *[numFrames];
	sprintf_s(inFileName,256,fileTmpl,0,0);
	imgs[3].readPgm(inFileName);					// bogus 4th image
	for(fNum=0; fNum<numFrames; fNum++){
		for(lNum=0; lNum<numLights; lNum++){
			sprintf_s(inFileName,256,fileTmpl,lNum,fNum);
			printf("%s\n",inFileName);
			imgs[lNum].readPgm(inFileName);
		}
		pImages[fNum]=NULL;
		pImages[fNum] = 
			Image::packGrayToRGBA(&(imgs[0]),&(imgs[1]),&(imgs[2]),&(imgs[3]),pImages[fNum]);
	}
	return(pImages);
}


void init()
{
	unsigned char *texture = NULL;
	Image *views;
	Image **bgImages;

	//darkImg.readPgm(DARKFILENAME);
	Lp = loadLPrimeTransposeFromFile(CALIBFILE,NULL, NUM_LIGHTS);

	views = new Image[4];
	if(numLights!=3){throw Exception("ERROR: Assuming 3 input images! Change to load different fragment program!\n");}

	CgFragmentProgram::Init(cgContext,cgFragmentProfile);
	// Loading four sets of textures into memory for comparison.
	// load4Textures(startImage, INFILE_TMPL, views, 0, NUM_LIGHTS, false);

	// Thrown in so we can capture frames from the live user interaction.
	// Store the interaction and replay later. 
	uiMode = UI_NORMAL;
	switch(uiMode){
		case UI_NORMAL:
			break;
		case UI_LOG:
			if(fopen_s(&uiLog,UILOGFILE,"w")) throw new Exception("Blew it opening uiLogFile");
			break;
		case UI_REPLAY:
			if(fopen_s(&uiLog,UILOGFILE,"r")) throw new Exception("Blew it opening uiLogFile");
			frameNumber = 0;
			// Allocate frame buffers
			fPixelData = (float *)malloc(4*windowW*windowH*sizeof(float));
			ucPixelData = (unsigned char *)malloc(3*windowW*windowH*sizeof(unsigned char));
			break;
	}

	// Generate texture name for source images. 
	glGenTextures(1, &srcTextureID);
	// Initialize texture. Bind this named texture. 
	glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
	checkGLErrors("load4Textures\n");
	// Tell OpenGL the pixel alignment is byte order (RGB)
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	checkGLErrors("load4Textures\n");
	// Set parameters for the current OpenGL texture
	// Use linear interpolation because I want to translate images by subpixel amounts.
	glTexParameteri(GL_TEXTURE_RECTANGLE_NV, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_NV, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_NV, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_NV, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	checkGLErrors("load4Textures\n");
	// Allocates texture but does not transfer:  
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	checkGLErrors("load4Textures\n");

	// Load data into background normal texture. The 4th channel is dummy.
	glGenTextures(1, &fakeBgNormalTextureID);
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgNormalTextureID);
	checkGLErrors("Generate fake background Normal Texture.\n");
	bgImages = loadImages(1, 3, BG_NORMAL_FILE_TMPL);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Normal Texture.\n");

	// Load data into background albedo texture. The 4th channel is dummy.
	glGenTextures(1, &fakeBgAlbedoTextureID);
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgAlbedoTextureID);
	checkGLErrors("Generate fake background Albedo Texture.\n");
	bgImages = loadImages(1, 3, BG_ALBEDO_FILE_TMPL);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Albedo Texture.\n");

	// Load data into background albedo for alpha-matting.
	glGenTextures(iBgNmbr, fakeBgAlbedoTextureID_AM);

	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgAlbedoTextureID_AM[0]);
	checkGLErrors("Generate fake background Albedo Texture for alpha matting.\n");
	bgImages = loadImages(1, 3, BG_ALBEDO_FILE_TMPL_2);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Albedo Texutre for Alpha Matting.\n");

	//glGenTextures(1, &fakeBgAlbedoTextureID_AM[1]);
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgAlbedoTextureID_AM[1]);
	checkGLErrors("Generate fake background Albedo Texture for alpha matting.\n");
	bgImages = loadImages(1, 3, BG_ALBEDO_FILE_TMPL_3);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Albedo Texutre for Alpha Matting.\n");

	//glGenTextures(1, &fakeBgAlbedoTextureID_AM[2]);
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgAlbedoTextureID_AM[2]);
	checkGLErrors("Generate fake background Albedo Texture for alpha matting.\n");
	bgImages = loadImages(1, 3, BG_ALBEDO_FILE_TMPL_4);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Albedo Texutre for Alpha Matting.\n");

	//glGenTextures(1, &fakeBgAlbedoTextureID_AM[3]);
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, fakeBgAlbedoTextureID_AM[3]);
	checkGLErrors("Generate fake background Albedo Texture for alpha matting.\n");
	bgImages = loadImages(1, 3, BG_ALBEDO_FILE_TMPL_5);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
		(*bgImages)->pData);
	checkGLErrors("Load Data to Background Albedo Texutre for Alpha Matting.\n");

	// srcImages = loadImages(numFrames, numLights, INFILE_TMPL);

	// XXX: Load one image to get us started
	//for(int i = 0; i < numGrabBuffers; i++){
	//	pTextureBuffers[i] = (*srcImages)[0].pData;
	//}
	//glBindTexture(GL_TEXTURE_RECTANGLE_NV,srcTextureID);
	//glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0, 0, WIDTH, HEIGHT,
	//	GL_RGBA, GL_UNSIGNED_BYTE, (*srcImages)[0].pData);


	// One Fbo/Tex pair to hold the computed normal/albedo texture 
	InitAndBindFboTexPair(twoPassFbId,normAlbTexID,GL_COLOR_ATTACHMENT0_EXT);
	InitAndBindFboTexPair(bgMeanFrmBufID, bgMeanTextureID, GL_COLOR_ATTACHMENT0_EXT);
	InitAndBindFboTexPair(bgVarFrmBufID, bgVarTextureID, GL_COLOR_ATTACHMENT0_EXT);
	InitAndBindFboTexPair(mattingFrmBufID, mattingTextureID, GL_COLOR_ATTACHMENT0_EXT);

	// Initialize Pixel Buffer Object for fast transfers
    // get OpenGL info. This is it. Easy.
	if(bUsePBOs){
		glGenBuffersARB(2,pboBuf);
	}
	// Pass 1 computes normal and albedo at each Bayer pixel
	///fpPsPass1 = new CgFragmentProgram(CG_PS_P1SRCS,cgContext,cgFragmentProfile);
	fpPsPass1 = new CgFragmentProgram(CG_PS_PASS1,cgContext,cgFragmentProfile);

	// Pass 2 applies effects.
	// fpPsPass2 = new CgFragmentProgram(CG_PS_P2CLONE,cgContext,cgFragmentProfile);
	// fpPsPass2 = new CgFragmentProgram(CG_PS_P2VIZNORMS,cgContext,cgFragmentProfile);
	fpPsPass2 = new CgFragmentProgram(CG_PS_P2NTBAYER,cgContext,cgFragmentProfile);
	fpPsPass2->SetParam3fv("Ldir",lightDir1);
	fpPsPass2->SetParam1f("gain",normGain);

	fpBgMean	= new CgFragmentProgram(CG_MATTING_BGMEAN, cgContext, cgFragmentProfile);
	fpBgVar		= new CgFragmentProgram(CG_MATTING_BGVAR, cgContext, cgFragmentProfile);
	fpJustShow	= new CgFragmentProgram(CG_MATTING_JUSTSHOW, cgContext, cgFragmentProfile);
	fpMatting	= new CgFragmentProgram(CG_MATTING_MATTING, cgContext, cgFragmentProfile);
	fpMatMedian = new CgFragmentProgram(CG_MATTING_MEDIAN, cgContext, cgFragmentProfile);
	fpPSMatting = new CgFragmentProgram(CG_PS_MATTING, cgContext, cgFragmentProfile);
	fpJustShowDemosaic = new CgFragmentProgram(CG_JUSTSHOW_DEMOSAIC, cgContext, cgFragmentProfile);
	fpAlphaMatting = new CgFragmentProgram(CG_ALPHA_MATTING, cgContext, cgFragmentProfile);

}


// Image update thread
// In future: do NOT put the copy in the critical section. 
// Just check the buffer numbers in the critical section.
// Otherwise both threads will lock on the data transfers, which 
// is not what we want. 
MIL_INT MFTYPE FlashFunction_am(MIL_INT HookType, MIL_ID HookId, void MPTYPE *HookDataPtr)
{
	HookDataStruct *UserHookDataPtr = (HookDataStruct *)HookDataPtr;
	// iAmbCntOffset stores the offset that should be used in current process.
	int		iAmbCntOffset	=	UserHookDataPtr->iAmbCntOffset;
	int		n = ( (UserHookDataPtr->ProcessedImageCount++) % (iGrabBufNmbr_am) );
	int		iSubGrpIndx = (n % iLghtNmbr_am);
	int		iAmbIndx, iLghtIndx, iFrmCntDif;
	int		nextGrabBuffer;
	bool	bFrmMissed, bDoGrab = false;

	//nCalledCount ++;

	if (n == 0)
	{
		MimArith(MilGrabBufList_am[n], MilLights_am[iSubGrpIndx], MilLights_am[iSubGrpIndx], M_ADD);
		MbufGetLine(MilGrabBufList_am[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[0]);
		if(UserHookDataPtr->bDoCalibCounter)
		{
			UserHookDataPtr->bDoCalibCounter = false;
			UserHookDataPtr->iAmbCntOffset = 
				( (pucFrmCntBytes[1]*256+pucFrmCntBytes[0]+1) % iLghtNmbr_am );
		}
		else if ( (pucFrmCntBytes[1] == 0) && (pucFrmCntBytes[0] == 0) )
		{// This mean just between the frm 32 and 33, transaction of frame counter occurs.
			UserHookDataPtr->iAmbCntOffset = (iAmbCntOffset == 0) ? iLghtNmbr_am-1 : iAmbCntOffset-1;
		}
	}
	else if ( n < (iGrpFrmNmbr_am-1) )
	{	// n = 1, 2, ....
		//Accumulate Ambient, L1, L2, L3
		MimArith(MilGrabBufList_am[n], MilLights_am[iSubGrpIndx], MilLights_am[iSubGrpIndx], M_ADD);
	}
	else if ( n == (iGrpFrmNmbr_am-1) )
	{ // 33 as one groug have been grabbed.
			//First check if frm is missed.
		MbufGetLine(MilGrabBufList_am[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[2]);
		iFrmCntDif = (pucFrmCntBytes[3]-pucFrmCntBytes[1])*256 + pucFrmCntBytes[2] - pucFrmCntBytes[0];
		if (iFrmCntDif < 0)
		{
			iFrmCntDif = iFrmCntDif + 65536;
			// We know it overflows.
			UserHookDataPtr->iAmbCntOffset = (iAmbCntOffset==0)? iLghtNmbr_am-1 : iAmbCntOffset-1;
		}
		bFrmMissed = ( iFrmCntDif != (iGrpFrmNmbr_am - 1) );
		// Then check if buffer is available for grabbing.
		EnterCriticalSection(&textureCriticalSection);
		// Grab into next buffer unless it's the next display buffer
		nextGrabBuffer = (g_LastGrabBuffer+1 == numGrabBuffers) ? 
			0 : g_LastGrabBuffer+1;
		bDoGrab = (nextGrabBuffer != g_NextDisplayBuffer);
		LeaveCriticalSection(&textureCriticalSection);

		if( (bDoGrab == true)&&(bFrmMissed==false) )
		{
			//MosPrintf(MIL_TEXT("One Frame!\n"));
			//// Update next buffer, increment frame count, incr grabBuffer
			//pTextureBuffers[nextGrabBuffer] = srcImages[frameNum]->pData;
			//frameNum = (frameNum+1==iPackedBufNmbr) ? 0 : frameNum + 1;
			// Accumulate as usual.
			MimArith(MilGrabBufList_am[n], MilLights_am[iSubGrpIndx], MilLights_am[iSubGrpIndx], M_ADD);
			// Make sure which group corresponding to Ambient light.
			iAmbIndx = ( 
				(iAmbCntOffset - ((pucFrmCntBytes[1]*256+pucFrmCntBytes[0])%iLghtNmbr_am) + iLghtNmbr_am) 
				% iLghtNmbr_am
				);

			iLghtIndx = iAmbIndx;
			MimArith(MilLights_am[iLghtIndx], 0, MilChildBuf_Red, M_SUB_CONST+M_SATURATION);

			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr_am) ? 0:iLghtIndx+1;
			MimArith(MilLights_am[iLghtIndx], MilLights_am[iAmbIndx], MilChildBuf_Blue, M_SUB+M_SATURATION);

			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr_am) ? 0:iLghtIndx+1;
			MimArith(MilLights_am[iLghtIndx], MilLights_am[iAmbIndx], MilChildBuf_Green, M_SUB+M_SATURATION);

			MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_ALL_BANDS);
			//MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_BLUE);
			//MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_GREEN);
			//MbufCopy(MilAmbBufOnBoard, MilAmbBufHost[iPackedBufIndx]);

			// Update next buffer, increment frame count, incr grabBuffer
			pTextureBuffers[nextGrabBuffer] = ppMilPackedBuf[iPackedBufIndx];
			pAmbientBuffers[nextGrabBuffer] = ppMilAmbBufHost[iPackedBufIndx];		
			iPackedBufIndx = (iPackedBufIndx+1 == iPackedBufNmbr) ? 0:iPackedBufIndx+1;
		
			// Update the grab buffer index
			EnterCriticalSection(&textureCriticalSection);
			g_LastGrabBuffer = nextGrabBuffer;
			LeaveCriticalSection(&textureCriticalSection);

			// Wake up graphics if waiting for frame
			WakeConditionVariable (&grabberConditionVariable);

		}//Ending: if(bDoGrab && !bFrmMissed)
		else if(bFrmMissed == true)
		{
			MosPrintf(MIL_TEXT("Frame Missed, since the difference is %d.\n"), iFrmCntDif);
		}//Ending: else if(bFrmMissed == true)

		// Clear Accumulating buffer.
		for ( int i = 0; i < iLghtNmbr_am; i++ )
		{
			MbufClear(MilLights_am[i], 0);
		}
	}
	else if ( n == iGrpFrmNmbr_am )
	{
		MimArith(
			MilGrabBufList_am[n], MilLights_am[iSubGrpIndx+iLghtNmbr_am], 
			MilLights_am[iSubGrpIndx+iLghtNmbr_am], M_ADD);
		MbufGetLine(MilGrabBufList_am[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[4]);
		if ( (pucFrmCntBytes[4] == 0) && (pucFrmCntBytes[5] == 0) )
		{// This mean just between the frm 32 and 33, transaction of frame counter occurs.
			UserHookDataPtr->iAmbCntOffset = (iAmbCntOffset == 0) ? iLghtNmbr_am-1 : iAmbCntOffset-1;
		}
	}
	else if ( n < (2*iGrpFrmNmbr_am-1) ) 
	{
		//Accumulate into another buffer, ambient, L1, L2, L3
		MimArith(
			MilGrabBufList_am[n], MilLights_am[iSubGrpIndx+iLghtNmbr_am], 
			MilLights_am[iSubGrpIndx+iLghtNmbr_am], M_ADD
			);
	}
	else 
	{	// n = (2*iGrpFrmNmbr_am-1), another group have been filled.
		// First check if frm is missed.
		//MosPrintf(MIL_TEXT("One Frame Packed!\n"));
		MbufGetLine(MilGrabBufList_am[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[6]);
		iFrmCntDif = (pucFrmCntBytes[7] - pucFrmCntBytes[5]) * 256 + pucFrmCntBytes[6] - pucFrmCntBytes[4];
		if (iFrmCntDif < 0)
		{
			iFrmCntDif = iFrmCntDif + 65536;
			// This means overflowing occurs and offset should be change.
			// But note that in this turn! Offset should not change!!
			UserHookDataPtr->iAmbCntOffset = (iAmbCntOffset == 0) ? iLghtNmbr_am-1 : iAmbCntOffset-1;
		}
		bFrmMissed = ( iFrmCntDif != (iGrpFrmNmbr_am-1) );
		// Then check if buffer is available for grabbing.
		EnterCriticalSection(&textureCriticalSection);
		// Grab into next buffer unless it's the next display buffer
		nextGrabBuffer = (g_LastGrabBuffer+1 == numGrabBuffers) ? 
			0 : g_LastGrabBuffer+1;
		bDoGrab = (nextGrabBuffer != g_NextDisplayBuffer);
		LeaveCriticalSection(&textureCriticalSection);
		if( (bDoGrab == true)&&(bFrmMissed==false) )
		{
			// Accumulate the last one as before.
			MimArith(
				MilGrabBufList_am[n], MilLights_am[iSubGrpIndx+iLghtNmbr_am], 
				MilLights_am[iSubGrpIndx+iLghtNmbr_am], M_ADD
				);
			// Make sure which group corresponding to Ambient light.
			iAmbIndx = ( 
				(iAmbCntOffset - ((pucFrmCntBytes[5]*256+pucFrmCntBytes[4])%iLghtNmbr_am) + iLghtNmbr_am)
				% iLghtNmbr_am );
			// Subtract and pack them. The fourth channel is used to save the ambient/4.
			// Maybe divided by 4 is not necessary, and just truncate the unsigned number is feasible.
			
			iLghtIndx = iAmbIndx;
			MimArith(MilLights_am[iLghtIndx+iLghtNmbr_am], 0, MilChildBuf_Red, M_SUB_CONST+M_SATURATION);
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilDarkAveOnBoard, MilAmbBufOnBoard, M_SUB + M_SATURATION);
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], 0, MilAmbBufOnBoard, M_SUB_CONST + M_SATURATION);
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], 4, MilAmbBufOnBoard, M_DIV_CONST);

			//Because MilPackedBufHost is BGR32, so light1 corresponds to blue channel!
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr_am) ? 0:iLghtIndx+1;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_BLUE); 
			MimArith(
				MilLights_am[iLghtIndx+iLghtNmbr_am], MilLights_am[iAmbIndx+iLghtNmbr_am],
				MilChildBuf_Blue, M_SUB + M_SATURATION
				);
			
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr_am) ? 0:iLghtIndx+1;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);			
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_GREEN);
			MimArith(
				MilLights_am[iLghtIndx+iLghtNmbr_am], MilLights_am[iAmbIndx+iLghtNmbr_am],
				MilChildBuf_Green, M_SUB + M_SATURATION
				);

			//iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			////MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);
			////MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_RED);
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilChildBuf_Red, M_SUB + M_SATURATION);

			MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_ALL_BANDS);
			//MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_BLUE);
			//MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_GREEN);
			//MbufCopy(MilAmbBufOnBoard, MilAmbBufHost[iPackedBufIndx]);
			
			// Update next buffer, increment frame count, incr grabBuffer
			pTextureBuffers[nextGrabBuffer] = ppMilPackedBuf[iPackedBufIndx];
			pAmbientBuffers[nextGrabBuffer] = ppMilAmbBufHost[iPackedBufIndx];
			iPackedBufIndx = (iPackedBufIndx+1 == iPackedBufNmbr) ? 0:iPackedBufIndx+1;
			
			// Update the grab buffer index
			EnterCriticalSection(&textureCriticalSection);
			g_LastGrabBuffer = nextGrabBuffer;
			LeaveCriticalSection(&textureCriticalSection);

			// Wake up graphics if waiting for frame
			WakeConditionVariable (&grabberConditionVariable);
		}//Ending: if(bDoGrab && !bFrmMissed)
		else if(bFrmMissed == true)
		{
			MosPrintf(MIL_TEXT("Frame Missed, since the difference is %d.\n"), iFrmCntDif);
		}//Ending: else if(bFrmMissed == true)

		// Clear Accumulating Buffer.
		for ( int i = iLghtNmbr_am; i < 2*iLghtNmbr_am; i++ )
		{
			MbufClear(MilLights_am[i], 0);
		}
	}

	return 0;
}

MIL_INT MFTYPE FlashFunction(MIL_INT HookType, MIL_ID HookId, void MPTYPE *HookDataPtr)
{
	// Variables for Mil grabbing.
	HookDataStruct *UserHookDataPtr = (HookDataStruct *)HookDataPtr;
	// Wenxuan Dec31: Though retrieving the ID is more safe, but I can imagine it is not so efficient!
	// For now, we just assume the function fill the buffer list one after another.
	// So, the retrieving process is ignored!
	// MIL_ID ModifiedBufferId;
	// Retrieve the MIL_ID of the grabbed buffer. 
	// MdigGetHookInfo(HookId, M_MODIFIED_BUFFER+M_BUFFER_ID, &ModifiedBufferId);
	
	// Get the index for the buffer in the buffer list.
	int iAmbCntOffset = UserHookDataPtr->iAmbCntOffset;
	int n = ( (UserHookDataPtr->ProcessedImageCount++)%(2*iGrpFrmNmbr) );
	int iSubGrpIndx = (n%iLghtNmbr);
	int	iAmbIndx;
	int	iLghtIndx;
	int iFrmCntDif;
	bool bFrmMissed;
	// Variables for thread control.
	int nextGrabBuffer;
	bool bDoGrab = false;
	int iBitsToShiftRight = UserHookDataPtr->iBitsToShiftRight;
	bool bShift = (iBitsToShiftRight > 0);

	//nCalledCount++;

	if ( n == 0 )
	{
		MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx], MilLights[iSubGrpIndx], M_ADD);
		MbufGetLine(MilGrabBufList[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[0]);
	}
	else if ( n < (iGrpFrmNmbr-1) )
	{	// n = 1, 2, ....
		//Accumulate Ambient, L1, L2, L3
		MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx], MilLights[iSubGrpIndx], M_ADD);
	}
	else if ( n == (iGrpFrmNmbr-1) )
	{ // 32 as one groug have been grabbed.
		MbufGetLine(MilGrabBufList[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[2]);
		iFrmCntDif = (pucFrmCntBytes[3]-pucFrmCntBytes[1])*256 + pucFrmCntBytes[2] - pucFrmCntBytes[0];
		iFrmCntDif = (iFrmCntDif < 0)? iFrmCntDif+65536:iFrmCntDif;
		bFrmMissed = ( iFrmCntDif != (iGrpFrmNmbr - 1) );
		// Then check if buffer is available for grabbing.
		EnterCriticalSection(&textureCriticalSection);
		// Grab into next buffer unless it's the next display buffer
		nextGrabBuffer = (g_LastGrabBuffer+1 == numGrabBuffers) ? 
			0 : g_LastGrabBuffer+1;
		bDoGrab = (nextGrabBuffer != g_NextDisplayBuffer);
		LeaveCriticalSection(&textureCriticalSection);

		if( (bDoGrab == true)&&(bFrmMissed==false) )
		{
			//MosPrintf(MIL_TEXT("One Frame!\n"));
			//// Update next buffer, increment frame count, incr grabBuffer
			//pTextureBuffers[nextGrabBuffer] = srcImages[frameNum]->pData;
			//frameNum = (frameNum+1==iPackedBufNmbr) ? 0 : frameNum + 1;
			// Accumulate as usual.
			MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx], MilLights[iSubGrpIndx], M_ADD);
			// Make sure which group corresponding to Ambient light.
			iAmbIndx = ( (iAmbCntOffset - ((pucFrmCntBytes[1]*256+pucFrmCntBytes[0])%iLghtNmbr) + iLghtNmbr) % iLghtNmbr );
			// Subtract and pack them. The fourth channel is used to save the ambient/4.
			// Maybe divided by 4 is not necessary, and just truncate the unsigned number is feasible.

			iLghtIndx = iAmbIndx;
			MimArith(MilLights[iLghtIndx], 0, MilAmbBufOnBoard, M_SUB_CONST + M_SATURATION);
			//MimArith(MilLights[iLghtIndx], MilDarkAveOnBoard, MilAmbBufOnBoard, M_SUB + M_SATURATION);
			//if (bShift == true)
			//{
			//	MimShift(MilLights[iLghtIndx], MilLightsShift[iLghtIndx], -iBitsToShiftRight);
			//	MimArith(MilLights[iLghtIndx], MilLightsShift[iLghtIndx], MilLights[iLghtIndx], M_SUB_CONST+M_SATURATION);
			//}
			//else
			//{
			//	MimArith(MilLights[iLghtIndx], 0, MilAmbBufOnBoard, M_SUB_CONST + M_SATURATION);
			//}
			//MimArith(MilLights[iLghtIndx], 4, MilAmbBufOnBoard, M_DIV_CONST);

			//Because MilPackedBufHost is BGR32, so light1 corresponds to blue channel!
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Blue, M_SUB + M_SATURATION);
			//MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilBayerImage, M_SUB + M_SATURATION);
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_BLUE); 
			//if (bShift == true)
			//{
			//	MimShift(MilLights[iLghtIndx], MilLightsShift[iLghtIndx], -iBitsToShiftRight);
			//	MimArith(MilLights[iLghtIndx], MilLightsShift[iAmbIndx], MilChildBuf_Blue, M_SUB+M_SATURATION);
			//}
			//else
			//{
			//	MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Blue, M_SUB + M_SATURATION);
			//}
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Green, M_SUB + M_SATURATION);
			//MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilBayerImage, M_SUB + M_SATURATION);			
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_GREEN);
			//if (bShift == true)
			//{
			//	MimShift(MilLights[iLghtIndx], MilLightsShift[iLghtIndx], -iBitsToShiftRight);
			//	MimArith(MilLights[iLghtIndx], MilLightsShift[iAmbIndx], MilChildBuf_Green, M_SUB + M_SATURATION);
			//}
			//else
			//{
			//	MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Green, M_SUB + M_SATURATION);
			//}
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Red, M_SUB + M_SATURATION);
			//MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilBayerImage, M_SUB + M_SATURATION);
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_RED);	
			//if (bShift == true)
			//{
			//	MimShift(MilLights[iLghtIndx], MilLightsShift[iLghtIndx], -iBitsToShiftRight);
			//	MimArith(MilLights[iLghtIndx], MilLightsShift[iAmbIndx], MilChildBuf_Red, M_SUB + M_SATURATION);
			//}
			//else
			//{
			//	MimArith(MilLights[iLghtIndx], MilLights[iAmbIndx], MilChildBuf_Red, M_SUB + M_SATURATION);
			//}
			MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_ALL_BANDS);
			MbufCopy(MilAmbBufOnBoard, MilAmbBufHost[iPackedBufIndx]);

			// Update next buffer, increment frame count, incr grabBuffer
			pTextureBuffers[nextGrabBuffer] = ppMilPackedBuf[iPackedBufIndx];
			pAmbientBuffers[nextGrabBuffer] = ppMilAmbBufHost[iPackedBufIndx];		
			iPackedBufIndx = (iPackedBufIndx+1 == iPackedBufNmbr) ? 0:iPackedBufIndx+1;
		
			// Update the grab buffer index
			EnterCriticalSection(&textureCriticalSection);
			g_LastGrabBuffer = nextGrabBuffer;
			LeaveCriticalSection(&textureCriticalSection);

			// Wake up graphics if waiting for frame
			WakeConditionVariable (&grabberConditionVariable);

		}//Ending: if(bDoGrab && !bFrmMissed)
		else if(bFrmMissed == true)
		{
			MosPrintf(MIL_TEXT("Frame Missed, since the difference is %d.\n"),iFrmCntDif);
		}//Ending: else if(bFrmMissed == true)
		// Clear Accumulating buffer.
		for ( int i = 0; i < iLghtNmbr; i++ )
		{
			MbufClear(MilLights[i], 0);
		}
	}
	else if ( n == iGrpFrmNmbr )
	{
		MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx+iLghtNmbr], MilLights[iSubGrpIndx+iLghtNmbr], M_ADD);
		MbufGetLine(MilGrabBufList[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[4]);
	}
	else if ( n < (2*iGrpFrmNmbr-1) ) 
	{
		//Accumulate into another buffer, ambient, L1, L2, L3
		MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx+iLghtNmbr], MilLights[iSubGrpIndx+iLghtNmbr], M_ADD);
	}
	else 
	{	// n = n < (2*iGrpFrmNmbr-1), another group have been filled.
		// First check if frm is missed.
		//MosPrintf(MIL_TEXT("One Frame Packed!\n"));
		MbufGetLine(MilGrabBufList[n], 0, 0, 1, 0, M_DEFAULT, M_NULL, &pucFrmCntBytes[6]);
		iFrmCntDif = (pucFrmCntBytes[7]-pucFrmCntBytes[5])*256 + pucFrmCntBytes[6] - pucFrmCntBytes[4];
		iFrmCntDif = (iFrmCntDif < 0)? iFrmCntDif+65536 : iFrmCntDif;
		bFrmMissed = ( iFrmCntDif != (iGrpFrmNmbr - 1) );
		// Then check if buffer is available for grabbing.
		EnterCriticalSection(&textureCriticalSection);
		// Grab into next buffer unless it's the next display buffer
		nextGrabBuffer = (g_LastGrabBuffer+1 == numGrabBuffers) ? 
			0 : g_LastGrabBuffer+1;
		bDoGrab = (nextGrabBuffer != g_NextDisplayBuffer);
		LeaveCriticalSection(&textureCriticalSection);
		if( (bDoGrab == true)&&(bFrmMissed==false) )
		{
			// Accumulate the last one as before.
			MimArith(MilGrabBufList[n], MilLights[iSubGrpIndx+iLghtNmbr], MilLights[iSubGrpIndx+iLghtNmbr], M_ADD);
			// Make sure which group corresponding to Ambient light.
			iAmbIndx = ( (iAmbCntOffset - ((pucFrmCntBytes[5]*256+pucFrmCntBytes[4])%iLghtNmbr) + iLghtNmbr) % iLghtNmbr );
			// Subtract and pack them. The fourth channel is used to save the ambient/4.
			// Maybe divided by 4 is not necessary, and just truncate the unsigned number is feasible.
			
			iLghtIndx = iAmbIndx;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilDarkAveOnBoard, MilAmbBufOnBoard, M_SUB + M_SATURATION);
			MimArith(MilLights[iLghtIndx+iLghtNmbr], 0, MilAmbBufOnBoard, M_SUB_CONST + M_SATURATION);
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], 4, MilAmbBufOnBoard, M_DIV_CONST);

			//Because MilPackedBufHost is BGR32, so light1 corresponds to blue channel!
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_BLUE); 
			MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilChildBuf_Blue, M_SUB + M_SATURATION);
			
			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);			
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_GREEN);
			MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilChildBuf_Green, M_SUB + M_SATURATION);

			iLghtIndx = ((iLghtIndx + 1) == iLghtNmbr) ? 0:iLghtIndx+1;
			//MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilBayerImage, M_SUB + M_SATURATION);
			//MbufCopyColor(MilBayerImage, MilPackedBufHost[iPackedBufIndx], M_RED);
			MimArith(MilLights[iLghtIndx+iLghtNmbr], MilLights[iAmbIndx+iLghtNmbr], MilChildBuf_Red, M_SUB + M_SATURATION);

			MbufCopyColor(MilPackedBufOnBoard, MilPackedBufHost[iPackedBufIndx], M_ALL_BANDS);
			MbufCopy(MilAmbBufOnBoard, MilAmbBufHost[iPackedBufIndx]);
			
			// Update next buffer, increment frame count, incr grabBuffer
			pTextureBuffers[nextGrabBuffer] = ppMilPackedBuf[iPackedBufIndx];
			pAmbientBuffers[nextGrabBuffer] = ppMilAmbBufHost[iPackedBufIndx];
			iPackedBufIndx = (iPackedBufIndx+1 == iPackedBufNmbr) ? 0:iPackedBufIndx+1;
			
			// Update the grab buffer index
			EnterCriticalSection(&textureCriticalSection);
			g_LastGrabBuffer = nextGrabBuffer;
			LeaveCriticalSection(&textureCriticalSection);

			// Wake up graphics if waiting for frame
			WakeConditionVariable (&grabberConditionVariable);
		}//Ending: if(bDoGrab && !bFrmMissed)
		else if(bFrmMissed == true)
		{
			MosPrintf(MIL_TEXT("Frame Missed, since the difference is %d.\n"),iFrmCntDif);
		}//Ending: else if(bFrmMissed == true)

		// Clear Accumulating Buffer.
		for ( int i = iLghtNmbr; i < 2*iLghtNmbr; i++ )
		{
			MbufClear(MilLights[i], 0);
		}
	}
	return 0;
}

MIL_INT MFTYPE VoidFunction(MIL_INT HookType, MIL_ID HookId, void MPTYPE *HookDataPtr){
	return 1;
}


DWORD WINAPI GrabberThread (LPVOID pParam){
	MIL_ID		MilApplication;
	MIL_ID		MilSystem;
	MIL_ID		MilDisplay;
	MIL_ID		MilDigitizer;
	//MIL_INT		MilGrabBufferListSize;
	//MIL_INT		FrameMissed;
	//MIL_INT		ProcessFrameCount  = 0;
	//MIL_INT		NbFrames           = 0;
	MIL_INT		n = 0;
	//MIL_DOUBLE	ProcessFrameRate = 0.0;
	MIL_INT		mil_iSizeX;
	MIL_INT		mil_iSizeY;
	MIL_INT		GrabMode;

	/* Allocate defaults. */
	/* When allocating both the default image buffer and the default display using MappAllocDefault(), 
	the image buffer is given a displayable attribute, cleared, and selected to the display.*/
	MappAllocDefault(M_SETUP, &MilApplication, &MilSystem, &MilDisplay, &MilDigitizer, M_NULL);	
	/* Inquire for size. */
	mil_iSizeX = MdigInquire(MilDigitizer, M_SIZE_X, M_NULL);
	mil_iSizeY = MdigInquire(MilDigitizer, M_SIZE_Y, M_NULL);
	
	// Lots of Allocation Process begins here.
	for(int i = 0; i<iDarkBufNmbr; i++)
	{
		MbufAlloc2d(
			MilSystem, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, 
			M_IMAGE + M_GRAB + M_PROC + M_OFF_BOARD + M_NON_PAGED, 
			&MilDarkImage[i]
		);
		if (MilDarkImage[i] != M_NULL)
		{
			MbufClear(MilDarkImage[i], 0xFF);
		}
		else
		{
			MosPrintf(MIL_TEXT("Something wrong happens when allocating the MilDarkImage[%d]!\n"), i);
			break;
		}
	}
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC, &MilImageAcc);
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC, &MilFlashAcc);
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC, &MilFlashBigAcc);
	for(int i=0; i<iLghtNmbr*2; ++i)
	{
		MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC+M_ON_BOARD, &MilLights[i]);
		MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC+M_ON_BOARD, &MilLightsShift[i]);
		if (MilLights[i] == M_NULL)
		{
			MosPrintf(MIL_TEXT("Something wrong happens when allocating the MilLights[%d]!\n"), i);
		}
		else
		{
			MbufClear(MilLights[i], 0);
			MbufClear(MilLightsShift[i], 0);
		}
	}
	// MilLights_Alpha_Matting
	for(int i=0; i<iLghtNmbr_am * 2; i++)
	{
		MbufAlloc2d(
			MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED,
			M_IMAGE+M_PROC+M_ON_BOARD, &MilLights_am[i]);
		if (MilLights_am[i] == M_NULL)
		{
			MosPrintf(MIL_TEXT("Something wrong happens when allocating the MilLights_am[%d]!\n"), i);
		}
		else //For the first accumulation.
		{
			MbufClear(MilLights[i], 0);
		}
	}
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC, &MilDarkAcc);
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 16L+M_UNSIGNED, M_IMAGE+M_PROC+M_ON_BOARD, &MilDarkAveOnBoard); 
	//MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 8L + M_UNSIGNED, M_IMAGE + M_PROC + M_ON_BOARD, &MilBayerImage);
	MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, M_IMAGE + M_PROC + M_ON_BOARD, &MilAmbBufOnBoard);
	MbufAllocColor(
		MilSystem, 3, mil_iSizeX, mil_iSizeY, 8L + M_UNSIGNED, 
		//M_IMAGE + M_PROC + M_PACKED + M_BGR32 + M_ON_BOARD,
		M_IMAGE + M_PROC + M_PLANAR + M_RGB24 + M_ON_BOARD,
		&MilPackedBufOnBoard
		);
	if (MilPackedBufOnBoard == M_NULL)
	{
		MosPrintf(MIL_TEXT("Wrong when allocating MilPackedBufOnBoard.\n"));
	}
	else
	{
		//MbufChildColor2d(MilPackedBufOnBoard, M_BLUE, 0, 0, mil_iSizeX, mil_iSizeY, &MilChildBuf_Blue);
		//MbufChildColor2d(MilPackedBufOnBoard, M_GREEN, 0, 0, mil_iSizeX, mil_iSizeY, &MilChildBuf_Green);
		//MbufChildColor2d(MilPackedBufOnBoard, M_RED, 0, 0, mil_iSizeX, mil_iSizeY, &MilChildBuf_Red);
		MbufChildColor(MilPackedBufOnBoard, M_BLUE, &MilChildBuf_Blue);
		MbufChildColor(MilPackedBufOnBoard, M_GREEN, &MilChildBuf_Green);
		MbufChildColor(MilPackedBufOnBoard, M_RED, &MilChildBuf_Red);
		if ((MilChildBuf_Blue == M_NULL) || (MilChildBuf_Green == M_NULL) || (MilChildBuf_Red == M_NULL))
		{
			MosPrintf(MIL_TEXT("Wrong when allocating MilChildBuf!\n"));
		}
	}	
	// MbufAllocColor(MilSystem, 3, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, M_IMAGE+M_GRAB+M_PROC+M_DISP, &MilImageDisp32);
	for (int i=0; i<iPackedBufNmbr; i++)
	{
		MbufAllocColor(
			MilSystem, 3, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, 
			M_IMAGE + M_PACKED + M_BGR32 + M_HOST_MEMORY,  
			&MilPackedBufHost[i]);
		MbufAlloc2d(MilSystem, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, M_IMAGE + M_HOST_MEMORY, &MilAmbBufHost[i]);
		if (MilPackedBufHost[i] == M_NULL)
		{
			MosPrintf(MIL_TEXT("Wrong when allocating MilPackedBufHost.\n"));
		}
		else
		{
			MbufInquire(MilPackedBufHost[i], M_HOST_ADDRESS, &MilPackedBufHostAddr);
			//MosPrintf(MIL_TEXT("The host address of MilPackedBufHost is %d.\n"), MilPackedBufHostAddr);
			ppMilPackedBuf[i] = (unsigned char*)MilPackedBufHostAddr;
		}
		if (MilAmbBufHost[i] == M_NULL)
		{
			MosPrintf(MIL_TEXT("Wrong when allocating MilAmbBufHost.\n"));
		}
		else
		{
			MbufInquire(MilAmbBufHost[i], M_HOST_ADDRESS, &MilAmbBufHostAddr);
			ppMilAmbBufHost[i] = (unsigned char*)MilAmbBufHostAddr;
		}	
	}

	/* Allocate the grab buffers and clear them. */
	for(int i = 0; i<iGrabBufNmbr; i++)
	{
		MbufAlloc2d(
			MilSystem, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED, 
			M_IMAGE + M_GRAB + M_PROC + M_ON_BOARD, &MilGrabBufList[i]
			);
		if (M_NULL != MilGrabBufList[i])
		{
			MbufClear(MilGrabBufList[i], 0xFF);
		}
		else
		{
			MosPrintf(MIL_TEXT("Error in Allocating MilGrabBufList[%d].\n"), i);
			break;
		}
	}
	// Allocating another grabbing buffer.
	for(int i=0; i<iGrabBufNmbr_am; i++)
	{
		MbufAlloc2d(
			MilSystem, mil_iSizeX, mil_iSizeY, 8L+M_UNSIGNED,
			M_IMAGE + M_GRAB + M_PROC + M_ON_BOARD, &MilGrabBufList_am[i]
		);
		// Clear them is unnecessary.
	}
	
	MosPrintf(MIL_TEXT("Press to Select Procession Function for : \n"));
	//MosPrintf(MIL_TEXT("Now just press any key and turn on the LED! \n"));
	MosPrintf(MIL_TEXT("\t'a' for PS_Segmentation.\n"));
	MosPrintf(MIL_TEXT("\t'm' for Alpha_Matting.\n"));
	GrabMode = MosGetch();

	/* Start the processing. The processing function is called for every frame grabbed. */;
	//nCalledCount = 0;
	UserHookData.ProcessedImageCount = 0;
	UserHookData_am.ProcessedImageCount = 0;
	//UserHookData.iBitsToShiftRight = 0;
	//MdigProcess(MilDigitizer, MilGrabBufList, MilGrabBufferListSize, M_START, M_ASYNCHRONOUS, proFun, &UserHookData);
	// Then begin. But for the 3 lights condition, I will do calibration in the first frame.
	if (GrabMode == 'a')
	{
		MdigProcess(MilDigitizer, MilDarkImage, iDarkBufNmbr, M_SEQUENCE, M_DEFAULT, VoidFunction, M_NULL);
		// Check the frame count of the 1st frame of dark image.
		MbufGetLine(MilDarkImage[0], 0, 0, 1, 0, M_DEFAULT, &n, pucFrmCntBytes);
		if(n == 2)
		{
			UserHookData.iAmbCntOffset = ( (pucFrmCntBytes[0] + 256*pucFrmCntBytes[1]) % iLghtNmbr );
		}
		else
		{
			MosPrintf(MIL_TEXT("Wrong in calibrating the frame count!\n"));
		}
		MdigProcess(MilDigitizer, MilGrabBufList, iGrabBufNmbr, M_START, M_ASYNCHRONOUS, FlashFunction, &UserHookData);
	}
	else
	{
		UserHookData_am.bDoCalibCounter = true;
		MdigProcess(MilDigitizer, MilGrabBufList_am, iGrabBufNmbr_am, M_START, M_ASYNCHRONOUS, FlashFunction_am, &UserHookData_am);
	}/* Wenxuan: Selection here changes the static variable nLightIdx. */
	MosPrintf(MIL_TEXT("You can press 'q' in DOS to stop grabbing.\n"));
	while(1)
	{
		if(MosGetch() == 'q')
		{
			break;
		}
	}
	if (GrabMode == 'a')
	{
		MdigProcess(
			MilDigitizer, MilGrabBufList, iGrabBufNmbr, 
			M_STOP, M_DEFAULT, M_NULL, &UserHookData);
	}
	else
	{	
		MdigProcess(
			MilDigitizer, MilGrabBufList_am, iGrabBufNmbr_am, 
			M_STOP, M_DEFAULT, M_NULL, &UserHookData_am);
	}
	

	/* Wenxuan: Release all the resources. */
	for(int i = 0; i < iDarkBufNmbr; i++)
	{
		MbufFree(MilDarkImage[i]);
	}
	for(int i = 0; i < iGrabBufNmbr; i++)
	{
		MbufFree(MilGrabBufList[i]);
	}
	for(int i = 0; i < iGrabBufNmbr_am; i++)
	{
		MbufFree(MilGrabBufList_am[i]);
	}
	for(int i = 0; i < iLghtNmbr*2; i++)
	{
		MbufFree(MilLights[i]);
		MbufFree(MilLightsShift[i]);
	}
	for(int i = 0; i < iLghtNmbr_am * 2; i++)
	{
		MbufFree(MilLights_am[i]);
	}
	MbufFree(MilDarkAcc); 
	MbufFree(MilDarkAveOnBoard); 
	//MbufFree(MilBayerImage);
	MbufFree(MilChildBuf_Blue);
	MbufFree(MilChildBuf_Green);
	MbufFree(MilChildBuf_Red);
	MbufFree(MilPackedBufOnBoard);
	MbufFree(MilAmbBufOnBoard);
	for (int i=0; i<iPackedBufNmbr; i++)
	{
		MbufFree(MilPackedBufHost[i]);
		MbufFree(MilAmbBufHost[i]);
	}
	MbufFree(MilImageAcc);
	MbufFree(MilFlashAcc);
	MbufFree(MilFlashBigAcc);
	/* Release defaults. */
	MappFreeDefault(MilApplication, MilSystem, MilDisplay, MilDigitizer, M_NULL);

	return 0;
}


int Main(int argc, char **argv)
{
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    //glutInitWindowSize(windowW,windowH);
	glutInitWindowSize(640,480);
	glutInitWindowPosition(100, 100);
    glutWindowHandle = glutCreateWindow(argv[0]);
	glewInit();

    glutKeyboardFunc(Keyboard);
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
	glutMotionFunc(Motion);
	glutMouseFunc(Mouse);

	init();

	HANDLE hGrabberThread;
	DWORD dwGenericThread;
	InitializeCriticalSection(&textureCriticalSection);
	// NULL can be pointer to parameter
	// 0 means run immediately after creation
	// dwGenericThread receives the thred identifier
	hGrabberThread = CreateThread(NULL,0,GrabberThread,NULL,0,&dwGenericThread);
	//SetThreadPriority(hThread,THREAD_PRIORITY_HIGHEST);
	InitializeConditionVariable(&grabberConditionVariable);


    glutMainLoop();

	bKillGrabberThread = false;
	DeleteCriticalSection(&textureCriticalSection);
	CloseHandle(hGrabberThread);
    
	cleanup();
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return Main(argc, argv);
    }
    catch(ExitException e)
    {        
        return 0;
    }
    catch(Exception e)
    {
        cerr<<"Error : "<<e.Message()<<endl;
        return 1;
    }
}


