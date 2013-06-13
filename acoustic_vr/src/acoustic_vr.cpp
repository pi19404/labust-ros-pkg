/*===============================================================================================
 3D Demo Example
 Copyright (c), Firelight Technologies Pty, Ltd 2005-2011.

 Example to show occlusion
===============================================================================================*/
#include <iostream>
#include <fmod/fmod.hpp>
#include <fmod/fmod_errors.h>
#include <fmod/fmod_event.hpp>
#include <fmod/fmod_event_net.hpp>
#include <labust/xml/GyrosWriter.hpp>
#include <moos/MoosConfig.hpp>
#include <moos/GyrosMoosCommsInterface.h>
//#include <windows.h>
#include <boost/algorithm/string.hpp>

#include <GL/glut.h>
#include <SOIL/SOIL.h>

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stack>
#include <string>
#include <fstream>
#include <sstream>
#include <JoystickReader.h>
#include <time.h>

#include <ros/ros.h>

//#include <crtdbg.h>

LABUST::JoystickReader *joystick;
LABUST::COMMUNICATION::GyrosMoosCommsInterface *comms;
std::string filterMessageLabel;
std::string rovMessageLabel;
std::string dvlMessageLabel;
std::string TARGETMessageLabel;
namespace FMOD
{

//#define SHOW_GUI_DEBUG_TEXT
#define GL_CLAMP_TO_EDGE 0x812F
#define	FMOD_INIT_DSOUND_HRTFFULL 0x00000800

//time_t start_time;
//time(&start_time);
//std::ofstream file("C:/Documents and Settings/Toni/Desktop/Doktorski Logs/Log_Doc.txt");
std::ofstream file("files/Log_Doc.txt");

std::stringstream converter;
float Head;
float HeadIni=0.0f;

unsigned char key1 =0;

void ERRCHECK(FMOD_RESULT result)
{
    if (result != FMOD_OK)
    {
        printf("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
        exit(-1);
    }
}

int INTERFACE_UPDATETIME = 15; // milliseconds

int intMax = 32767;
int r;
int iDiter = 0;
int KDiterTime = 1;
float AmpDiter = 0.0f;
float FreqDiter = 1.0f;
float tDiter[13] ={10,15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70};
float FiDiter[13] = {0, 15, -10, -25, 0, 5, -15, 15, -10, -3, -10, 5, 0};
float ti=0;
float Integral=0.0f;
bool fullscreen = false;
bool wait = false;

float forw=0.0f;
float InputStoh=0.0f;
int InputStoh1=0;
double lastObjectLong = 0.0;
double zPos = 0.0;
float aSQR = 1.0f;
float bSQR = 0.0f;
float cSQR = 0.0f;
// varijable koje se trebaju primiti iz MOOSa
float Altitude = 2.5f;
float ROVLat = 43.5f;
float ROVLong = 15.5f;
float ROVHeading = 0.0f;
float ROVRoll = 0.0f;
float ROVPitch = 0.0f;
float Depth = 0.0f;
float VelVec = 0.0f;
float HE = 0.0f;

// varijable koje se upisu "Rucno"
float AltitudeSetPoint = 10.0f;
bool TrajectoryTracking = false;
float ObjectLat = 43.5;
float ObjectLong = 15.5;
float ObjectDepth = 20.0f;
float Doppler = 100.0f;
float SilenceAngle = 0.00f;
float VerbalAngle = 5.00f;
float NonLinearCoeff = 1.0f;
float NonLinearCoeffDist = 1.0f;
float TransformAngle = 0.00f;
float PathVelocity = 0.0f;
float Ki = 0.0f;
int Sound=0;
int GuidanceMode = 1;
int TaskMode = 1;
int JoyStickMode=1;
char SpeedProfile='C';
float velAtitude = 0.0f;
float lastvelAtitude = 0.0f;
float PathStartX = 10.0f;
float PathStartY = 10.0f;
float PathStartX1 = 0.0f;
float PathStartY1 = 0.0f;
float PathX = 0.0f;
float PathY = 0.0f;
float DistanceToPath = 0.0f;
float RabbitDistance = 0.0f;
float RabbitNorth = 0;
float RabbitNorth1 = 0;
float RabbitNorth2 = 0;
float RabbitNorthOld = 0;
float RabbitEast = 0;
float RabbitEastOld = 0;
float HeadingErr = 0.0f;
float TauX = 0.0f;
float TauY = 0.0f;
float TauZ = 0.0f;
float TauK = 0.0f;
float TauM = 0.0f;
float TauN = 0.0f;
float frwVelVector = 0.0f;
float latVelVector = 0.0f;
float zVelVector = 0.0f;

// definiranje Way Pointa
int NumberOfWP = 15;
const int MaxNumberOfWP = 15;
//int mode = 1;// za lawnmover mode, mode = 0
int WPindex = 1;
int ActualWPindex = 1;
int ActualWPindexOld = 1;
float WPnorth[MaxNumberOfWP];// ={43.823023f, 43.822942f, 43.823321f, 43.823023f};
float WPeast[MaxNumberOfWP];// = {15.570909f, 15.570321f, 15.570435f,15.570909f};
float WPz[MaxNumberOfWP];


// window size
int width = 500;
int height = 500;

// mouse control
bool doRotate = false;
int xMouse = 0;
int yMouse = 0;

// listener orientation
float xRotation = 0.0f; // Roll
float yRotation = 0.0f; //Heading
float zRotation = 0.0f; //Pitch
float lastyRotation = 0.0f;

// listener position
float xListenerPos = 0.0f;
//TONI ustvari su y i z zamjenjeni
float yListenerPos = 50.0f; //ovo je Z
float zListenerPos = 20.0f; //ovo je Y
float DirectionToTarget = 0.0f;
float zListenerPosRel = 0.0f;
float xListenerPosRel = 0.0f;

// keyboard control
bool moveForward    = false;
bool moveBackward   = false;
bool moveRotateClock    = false;
bool moveRotateAntiClock   = false;
bool moveLeft       = false;
bool moveRight      = false;
bool moveUp         = false;
bool moveDown       = false;
bool moveFast       = false;
bool ambientVolUp   = false;
bool ambientVolDown = false;
bool masterVolUp    = false;
bool masterVolDown  = false;

#undef PI
const float PI = 3.14159265f;

float accumulatedTime = 0.0f;

// textures
GLuint texture;
GLuint skyboxTexture[6];

// sounds placement
struct Object
{
	float xPos;
	float yPos;
	float zPos;
	float intensity;
	int sound;
    FMOD::Event *event;
};

const int NUM_OBJECTS = 8;
Object objects[NUM_OBJECTS] =
{
	//{  -11.0f,    10.0f,    0.0f,    1.0f,    0,    0 },
	{   0.0f,     10.0f,    0.0f,    1.0f,    0,    0 }, //Toni: IZVOR ZVUKA JE ISHODISTE KOORDINATNOG SUSTAVA
	{   0.0f,    10.0f,    0.0f,    1.0f,    1,    0 },
	{   45.0f,    10.0f,    0.0f,    1.0f,    3,    0 },
	{  -30.0f,    1.0f,   21.0f,    1.0f,    2,    0 },
	{  -30.0f,    1.0f,  -21.0f,    1.0f,    3,    0 },
	{   12.0f,    1.0f,  -27.0f,    1.0f,    0,    0 },
	{    0.0f,    10.0f,   0.0f,    1.0f,    0,    0 },
	{    0.0f,    10.0f,   0.0f,    1.0f,    1,    0 },
};

// geometry structers for loading and drawing
struct Polygon
{
	int numVertices;
	int indicesOffset;
	float directOcclusion;
	float reverbOcclusion;
	FMOD_VECTOR normal;
};

struct Mesh
{
	int numVertices;
	FMOD_VECTOR *vertices;
	float (*texcoords)[2];
	int numPolygons;
	Polygon* polygons;
	int numIndices;
	int *indices;
	FMOD::Geometry *geometry;
};

class GlutCloseClass
{
  public:
    GlutCloseClass() {};
   ~GlutCloseClass();
};


GlutCloseClass gCloseObject;

Mesh walls;
Mesh rotatingMesh;

// fmod sounds structures
FMOD::EventSystem     *fmodEventSystem    = 0;
FMOD::EventProject    *fmodEventProject   = 0;
FMOD::EventGroup      *fmodEventGroup     = 0;
FMOD::EventParameter  *fmodEventParameter = 0;
FMOD::System          *fmodSystem         = 0;
FMOD::Geometry	      *geometry           = 0;
FMOD::DSP             *global_lowpass     = 0;

float   ambientVolume   = 0.2f;
float   masterVolume;

/*
    Global stack of strings to render
*/
#ifdef SHOW_GUI_DEBUG_TEXT
std::stack<std::string> debugText;
std::stack<std::string> statusText;
#endif

GLenum  rendermode = GL_FILL;

bool showdebug = false;
bool showhelp = false;

void outputText(int x, int y, std::string text)
{
	int i;
	const char *txt = text.c_str();

	glRasterPos2f(x, y);
	for (i = 0; i < (int)text.length(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, txt[i]);
	}
}

#ifdef SHOW_GUI_DEBUG_TEXT
void renderText(int x, int y, std::stack<std::string> *text)
{
    glColor3f(1.0f, 1.0f, 1.0f);
    for (int count = 0; count < text->size(); count++)
    {
        outputText(x, y, text->top());
        text->pop();

        y -= 17;
    }
}
#endif

void renderUiText()
{
    /*
        Render help text
    */
    if (showhelp)
    {
        int x = 10;
        int y = height - 20;

        glColor3f(1.0f, 1.0f, 1.0f);
        outputText(x, y,     "F1   - Toggle help");
        outputText(x, y-=18, "F2   - Toggle fullscreen");
        outputText(x, y-=18, "F3   - Toggle wireframe rendering");
        outputText(x, y-=18, "F11  - Toggle debug info");
        outputText(x, y-=18, "--");
        outputText(x, y-=18, "w     - Move forward");
        outputText(x, y-=18, "s     - Move backward");
        outputText(x, y-=18, "a     - Move left");
        outputText(x, y-=18, "d     - Move right");
        outputText(x, y-=18, "space - Move up");
        outputText(x, y-=18, "c     - Move down");
        outputText(x, y-=18, "Mouse (hold left button) - look direction");
        outputText(x, y-=18, "--");
  //      outputText(x, y-=18, "V/v   - Master volume up/down");
        outputText(x, y-=18, "Z/z   - Ambient sound volume up/down");
    }
    else
    {
        glColor3f(1.0f, 1.0f, 1.0f);
        outputText(10, height - 20, "F1 - Help");
    }

    /*
        Render debug text
    */
#ifdef SHOW_GUI_DEBUG_TEXT
    if (showdebug)
    {
        renderText(width - (width/2), height - 20, &debugText);
    }
    else
    {
        /*
            Otherwise just pop everything off the stack
        */
        for (int count = 0; count < debugText.size(); count++)
        {
            debugText.pop();
        }
    }

    /*
        Render status text
    */
    renderText(10, 20, &statusText);
#endif
}

void initGeometry(const char* szFileName, Mesh& mesh, bool alter = false)
{
	FMOD_RESULT result;

	FILE* file = fopen(szFileName, "rb");
	if (file)
	{
		// read vertices
		fread(&mesh.numVertices, sizeof (mesh.numVertices), 1, file);
		mesh.vertices = new FMOD_VECTOR[mesh.numVertices];
		mesh.texcoords = new float[mesh.numVertices][2];
		fread(mesh.vertices, sizeof (float) * 3, mesh.numVertices, file);
		fread(mesh.texcoords, sizeof (float) * 2, mesh.numVertices, file);


		fread(&mesh.numIndices, sizeof (mesh.numIndices), 1, file);
		mesh.indices = new int[mesh.numIndices];
		fread(mesh.indices, sizeof (int), mesh.numIndices, file);


		fread(&mesh.numPolygons, sizeof (mesh.numPolygons), 1, file);
		mesh.polygons = new Polygon[mesh.numPolygons];

		// read polygons
		for (int poly = 0; poly < mesh.numPolygons; poly++)
		{
			Polygon* polygon = &mesh.polygons[poly];


			fread(&polygon->numVertices, sizeof (polygon->numVertices), 1, file);
			fread(&polygon->indicesOffset, sizeof (polygon->indicesOffset), 1, file);
			fread(&polygon->directOcclusion, sizeof (polygon->directOcclusion), 1, file);
			fread(&polygon->reverbOcclusion, sizeof (polygon->reverbOcclusion), 1, file);

			int* indices = &mesh.indices[polygon->indicesOffset];

			// calculate polygon normal
			float xN = 0.0f;
			float yN = 0.0f;
			float zN = 0.0f;
			// todo: return an error if a polygon has less then 3 vertices.
			for (int vertex = 0; vertex < polygon->numVertices - 2; vertex++)
			{
				float xA = mesh.vertices[indices[vertex + 1]].x -mesh.vertices[indices[0]].x;
				float yA = mesh.vertices[indices[vertex + 1]].y -mesh.vertices[indices[0]].y;
				float zA = mesh.vertices[indices[vertex + 1]].z -mesh.vertices[indices[0]].z;
				float xB = mesh.vertices[indices[vertex + 2]].x -mesh.vertices[indices[0]].x;
				float yB = mesh.vertices[indices[vertex + 2]].y -mesh.vertices[indices[0]].y;
				float zB = mesh.vertices[indices[vertex + 2]].z -mesh.vertices[indices[0]].z;
				// cross product
				xN += yA * zB - zA * yB;
				yN += zA * xB - xA * zB;
				zN += xA * yB - yA * xB;
			}
			float fMagnidued = (float)sqrt(xN * xN + yN * yN + zN * zN);
			if (fMagnidued > 0.0f) // a tollerance here might be called for
			{
				xN /= fMagnidued;
				yN /= fMagnidued;
				zN /= fMagnidued;
			}
			polygon->normal.x = xN;
			polygon->normal.y = yN;
			polygon->normal.z = zN;
		}
		fclose(file);
	}

	result = fmodSystem->createGeometry(mesh.numPolygons, mesh.numIndices, &mesh.geometry);
	ERRCHECK(result);

    /*
        Tell FMOD about the geometry
    */
	for (int poly = 0; poly < mesh.numPolygons; poly++)
	{
		Polygon* polygon = &mesh.polygons[poly];
		FMOD_VECTOR vertices[16];
		for (int i = 0; i < polygon->numVertices; i++)
			vertices[i] = mesh.vertices[mesh.indices[polygon->indicesOffset + i]];
		int polygonIndex = 0;

        if (alter && polygon->directOcclusion == 0.85f)
        {
     //       polygon->directOcclusion = 0.95f;
        }

		result = mesh.geometry->addPolygon(
			polygon->directOcclusion,
			polygon->reverbOcclusion,
			false, // single sided
			polygon->numVertices,
			vertices,
			&polygonIndex);
		ERRCHECK(result);
	}
}


void freeGeometry(Mesh& mesh)
{
    mesh.geometry->release();

    delete [] mesh.vertices;
	delete [] mesh.texcoords;
	delete [] mesh.polygons;
	delete [] mesh.indices;
}


void inWater()
{
    FMOD_RESULT result;
    FMOD_REVERB_PROPERTIES reverbprops;
    static bool inwater = false;

    if (xListenerPos > -14.75f && xListenerPos < -7.6f
        && zListenerPos > -10.85f && zListenerPos < -3.75f
        && yListenerPos < 5.0f)
    {
        /*
            Use opengl fog to make it look like we are in water
        */
        if (!inwater)
        {
    	    glEnable(GL_FOG);

            result = fmodEventSystem->getReverbPreset("UnderWater", &reverbprops);
            ERRCHECK(result);
            result = fmodEventSystem->setReverbProperties(&reverbprops);
            ERRCHECK(result);
            result = global_lowpass->setBypass(false);
            ERRCHECK(result);

            inwater = true;
        }
    }
    else
    {
        /*
            Disable fog (water)
        */
        if (inwater)
        {
            glDisable(GL_FOG);

            result = fmodEventSystem->getReverbPreset("StdReverb", &reverbprops);
            ERRCHECK(result);
            result = fmodEventSystem->setReverbProperties(&reverbprops);
            ERRCHECK(result);
            if (global_lowpass)
            {
                result = global_lowpass->setBypass(true);
                ERRCHECK(result);
            }

            inwater = false;
        }
    }
}


void drawSkyBox()
{
    glPushMatrix();
        glTranslatef(xListenerPos, 0.0f, yListenerPos);
        glDisable(GL_LIGHTING);
        /*
            Walls
        */
        glBindTexture(GL_TEXTURE_2D, skyboxTexture[0]);
        glBegin(GL_QUADS);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-150.0f, -150.0f, -150.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-150.0f, 150.0f, -150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(150.0f, 150.0f, -150.0f);
            glTexCoord2f(0.0f, 1.0f);  glVertex3f(150.0f, -150.0f, -150.0f);
        glEnd();

        glBindTexture(GL_TEXTURE_2D, skyboxTexture[1]);
        glBegin(GL_QUADS);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(150.0f, -150.0f, -150.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(150.0f, 150.0f,-150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(150.0f, 150.0f, 150.0f);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(150.0f, -150.0f, 150.0f);
        glEnd();

        glBindTexture(GL_TEXTURE_2D, skyboxTexture[2]);
        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-150.0f, -150.0f, 150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-150.0f, 150.0f, 150.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(150.0f, 150.0f, 150.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(150.0f, -150.0f, 150.0f);
        glEnd();

        glBindTexture(GL_TEXTURE_2D, skyboxTexture[3]);
        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-150.0f, -150.0f, -150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-150.0f, 150.0f, -150.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-150.0f, 150.0f, 150.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-150.0f, -150.0f, 150.0f);
        glEnd();

        /*
            Top
        */
        glBindTexture(GL_TEXTURE_2D, skyboxTexture[4]);
        glBegin(GL_QUADS);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-150.0f, 150.0f, -150.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(150.0f, 150.0f, -150.0f);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(150.0f, 150.0f, 150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-150.0f, 150.0f, 150.0f);
        glEnd();

        /*
            Bottom
        */
        glBindTexture(GL_TEXTURE_2D, skyboxTexture[5]);
        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-150.0f, -150.0f, -150.0f);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(150.0f, -150.0f, -150.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(150.0f, -150.0f, 150.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-150.0f, -150.0f, 150.0f);
        glEnd();

        glEnable(GL_LIGHTING);
    glPopMatrix();
}

void drawWaterRoom()
{
    glBlendFunc( GL_SRC_ALPHA, GL_ONE );
    glEnable(GL_BLEND);

    glPushMatrix();
        glColor4f(0.0f,0.1f,0.8f,1.0f);
        glDisable(GL_LIGHTING);

        glBegin(GL_QUADS);
            glVertex3f(-14.72f, 4.0f, -10.85f);
            glVertex3f(-7.68f, 4.0f, -10.85f);
            glVertex3f(-7.68f, 4.0f, -3.85f);
            glVertex3f(-14.72f, 4.0f, -3.85f);
        glEnd();

        glEnable(GL_LIGHTING);
    glPopMatrix();

    glDisable(GL_BLEND);
}

void drawGeometry(Mesh& mesh)
{
	FMOD_RESULT result;

	FMOD_VECTOR pos;
	result = mesh.geometry->getPosition(&pos);
	ERRCHECK(result);

	glPushMatrix();
	// create matrix and set gl transformation for geometry
	glTranslatef(pos.x, pos.y, pos.z);
	FMOD_VECTOR forward;
	FMOD_VECTOR up;
	result = mesh.geometry->getRotation(&forward, &up);
	ERRCHECK(result);
	float matrix[16] =
	{
		up.y * forward.z - up.z * forward.y,		up.x,		forward.x,		0.0f,
		up.z * forward.x - up.x * forward.z,		up.y,		forward.y,		0.0f,
		up.x * forward.y - up.y * forward.x,		up.z,		forward.z,		0.0f,
		0.0f,										0.0f,		0.0f,			1.0f,
	};
	glMultMatrixf(matrix);

	// draw all polygons in object
	glEnable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, rendermode);
	for (int poly = 0; poly < mesh.numPolygons; poly++)
	{
		Polygon* polygon = &mesh.polygons[poly];
		if (polygon->directOcclusion == 0.0f)
        {
			continue; // don't draw because it is an open door way
        }
		glBegin(GL_TRIANGLE_FAN);
		glNormal3fv(&polygon->normal.x);

		for (int i = 0; i < polygon->numVertices; i++)
		{
			int index = mesh.indices[polygon->indicesOffset + i];
			glTexCoord2f(mesh.texcoords[index][0], mesh.texcoords[index][1]);
			glVertex3fv(&mesh.vertices[index].x);
		}
		glEnd();
	}
	glPopMatrix();
}

void initObjects()
{
    FMOD_RESULT result;
	// TONI pozivanje zvukova
    if (GuidanceMode < 3)
		{if (Sound==0)
		{result = fmodEventGroup->getEvent("PinkNew 500m", FMOD_EVENT_DEFAULT, &objects[0].event);
    ERRCHECK(result);

    result = fmodEventGroup->getEvent("Click1 500m", FMOD_EVENT_DEFAULT, &objects[1].event);
	ERRCHECK(result);}
		else
		{{result = fmodEventGroup->getEvent("PinkNew 500m", FMOD_EVENT_DEFAULT, &objects[1].event);
    ERRCHECK(result);

    result = fmodEventGroup->getEvent("Click1 500m", FMOD_EVENT_DEFAULT, &objects[0].event);
	ERRCHECK(result);}}
	}

    if (GuidanceMode > 1 && GuidanceMode < 4)
		{
			result = fmodEventGroup->getEvent("up", FMOD_EVENT_DEFAULT, &objects[2].event);
    ERRCHECK(result);

    result = fmodEventGroup->getEvent("down", FMOD_EVENT_DEFAULT, &objects[3].event);
    ERRCHECK(result);
	}

    if (GuidanceMode > 2)
		{result = fmodEventGroup->getEvent("left", FMOD_EVENT_DEFAULT, &objects[4].event);
    ERRCHECK(result);

    result = fmodEventGroup->getEvent("right", FMOD_EVENT_DEFAULT, &objects[5].event);
    ERRCHECK(result);
	}

    if (TaskMode == 3)
		{//result = fmodEventGroup->getEvent("3DSoundEmit", FMOD_EVENT_DEFAULT, &objects[6].event);
			if (Sound==0)
			{result = fmodEventGroup->getEvent("PinkNew", FMOD_EVENT_DEFAULT, &objects[6].event);
		ERRCHECK(result);
		result = fmodEventGroup->getEvent("Click1", FMOD_EVENT_DEFAULT, &objects[7].event);
		ERRCHECK(result);}
			else
			{result = fmodEventGroup->getEvent("PinkNew", FMOD_EVENT_DEFAULT, &objects[7].event);
		ERRCHECK(result);
		result = fmodEventGroup->getEvent("Click1", FMOD_EVENT_DEFAULT, &objects[6].event);
		ERRCHECK(result);}
	}


	for (int i=0; i < NUM_OBJECTS-1; i++)
	{
		FMOD_VECTOR pos = { objects[i].xPos, objects[i].yPos, objects[i].zPos };
		FMOD_VECTOR vel = { 0.0f, 0.0f, 0.0f };

        if (objects[i].event)
        {
		    //TONI izgleda kao prerada zvuka u 3D na osnovu pozicije i velocity (brzina kretanja) izvora zvuka?
			result = objects[i].event->set3DAttributes(&pos, &vel);
		    ERRCHECK(result);

			//TONI izgleda kao pokretanje zvuka
			result = objects[i].event->start();
            ERRCHECK(result);
        }
	}
}


void updateObjectSoundPos(Object* object)
{
    FMOD_RESULT result;

    if (object->event)
    {

		//TONI nova pozicija izvora zvuka
		FMOD_VECTOR pos = { object->xPos, object->yPos, object->zPos };
	    FMOD_VECTOR oldPos;
     	object->event->get3DAttributes(&oldPos, 0);

		//TONI racunanje brzine kretanja izvora zvuka
	    FMOD_VECTOR vel;
	    vel.x = 0;//(pos.x - oldPos.x) *  (1000.0f / (float)INTERFACE_UPDATETIME);
	    vel.y = 0;//(pos.y - oldPos.y) *  (1000.0f / (float)INTERFACE_UPDATETIME);
	    vel.z = 0;//(pos.z - oldPos.z) *  (1000.0f / (float)INTERFACE_UPDATETIME);
    	result = object->event->set3DAttributes(&pos, &vel);

//    	ERRCHECK(result);
    }
}


void mouseFunc(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			doRotate = true;
			xMouse = x;
			yMouse = y;
		}
		else
		if (state == GLUT_UP)
		{
			doRotate = false;
		}
		break;

	default:
		break;
	}
}

void motionFunc(int x, int y)
{
	//TONI s misom se moze rotirati a ne kretati naprijed-nazad
	std::string SingleLog;
	int test = 0;

	int dx = x - xMouse;
	int dy = y - yMouse;

	// view rotation about y-axis
	yRotation += (float)dx * 0.5f;
	if (yRotation > 180.0f)
		yRotation -= 360.0f;
	else
	if (yRotation < -180.0f)
		yRotation += 360.0f;

	// view rotation about x-axis
	const float xExtent = 88.0f;
	//TONI izmjena u redu ispod, da se ne mice gore-dolje vec samo lijevo-desno
	xRotation += 0.0f;//(float)dy * 0.5f;
	if (xRotation > xExtent)
		xRotation = xExtent;
	else
	if (xRotation < -xExtent)
		xRotation = -xExtent;

	converter<<dx;
	converter>>SingleLog;
	converter.clear();
	file<<SingleLog+' ';
	converter<<dy;
	converter>>SingleLog;
	converter.clear();
	file<<SingleLog+' ';

	xMouse = x;
	yMouse = y;
}

void doGeometryMovement()
{
	FMOD_RESULT result;

	// example of moving individual polygon vertices
	int xGeometryWarpPos = -30.0f;
	int zGeometryWarpPos = -21.0f;
	int dx = xListenerPos - xGeometryWarpPos;
	int dz = zListenerPos - zGeometryWarpPos;
	if (dx * dx + dz * dz < 30.0f * 30.0f)
	{
		if (sin(accumulatedTime * 1.0f) > 0.0f)
		{
			static FMOD_VECTOR lastOffset = { 0.0f, 0.0f, 0.0f };
			FMOD_VECTOR offset = { sin(accumulatedTime * 2.0f), 0.0f, cos(accumulatedTime * 2.0f) };
			for (int poly = 0; poly < walls.numPolygons; poly++)
			{
				Polygon* polygon = &walls.polygons[poly];
				for (int i = 0; i < polygon->numVertices; i++)
				{
					FMOD_VECTOR& vertex = walls.vertices[walls.indices[polygon->indicesOffset + i]];

					dx = vertex.x - xGeometryWarpPos;
					dz = vertex.z - zGeometryWarpPos;
					if (dx * dx + dz * dz > 90.0f)
						continue;
					vertex.x -= lastOffset.x;
					vertex.y -= lastOffset.y;
					vertex.z -= lastOffset.z;

					vertex.x += offset.x;
					vertex.y += offset.y;
					vertex.z += offset.z;
					result = walls.geometry->setPolygonVertex(poly, i, &vertex);
					ERRCHECK(result);
				}
			}
			lastOffset = offset;
		}
	}

	// example of rotation and a geometry object
	FMOD_VECTOR up = { 0.0f, 1.0f, 0.0f };
	FMOD_VECTOR forward = { (float)sin(accumulatedTime * 0.5f), 0.0f, (float)cos(accumulatedTime * 0.5f) };
	result = rotatingMesh.geometry->setRotation(&forward, &up);
	ERRCHECK(result);
	FMOD_VECTOR pos;
	pos.x = 12.0f;
	pos.y = (float)sin(accumulatedTime) * 0.4f + 0.1f;
	pos.z = 0.0f;
	result = rotatingMesh.geometry->setPosition(&pos);
	ERRCHECK(result);
	drawGeometry(rotatingMesh);
}

void doSoundMovement()
{
	//TONI micanje izvora zvuka
	//NonLinearCoeffDist is nonlineariti coeff for distance cue
	float azim = 0, DistanceErr = 0;
	float distance = sqrt((xListenerPos-RabbitEast)*(xListenerPos-RabbitEast)+(zListenerPos-RabbitNorth)*(zListenerPos-RabbitNorth)); //distance from target(rabbit) to the listener
	float TransformDistance=1.0f, NewDist=distance; //NewDist is supernormal distance, TransformDistance is realtionship betwean supernormal and real distance.

	//Nova distanca uzrokovana uvodjenjem nelinearnosti na isti nacin kao i za azimut
	DistanceErr = distance - 10;//RabbitDistance;
	if (ActualWPindex==1)
	{Integral=0;}
	else
	{Integral += DistanceErr*INTERFACE_UPDATETIME/1000;}
	NewDist = 10/PI*atan2((2*NonLinearCoeffDist*sin(2*(PI/2)*DistanceErr/10)),(1-NonLinearCoeffDist*NonLinearCoeffDist+(1+NonLinearCoeffDist*NonLinearCoeffDist)*cos(2*(PI/2)*DistanceErr/10)))+10;
	NewDist += Ki*Integral;
	//Nova supernormalna distanca sa mrtvom zonom +/-0.5 i linearno (promjenjeni koeficijent nagiba) u ostalom dijelu
	/*if (fabs(distance - RabbitDistance)>0.5)
	{
		if (fabs(distance - RabbitDistance)<5)
		{
			if (distance<RabbitDistance)
			{NewDist=5/NonLinearCoeffDist+(9.5-5/NonLinearCoeffDist)/4.5*(distance-5);}
			else
			{NewDist=10.5+(9.5-5/NonLinearCoeffDist)/4.5*(distance-10.5);}
		}
		else
		{
			if (distance<RabbitDistance)
			{NewDist=distance/(5/NonLinearCoeffDist);}
			else
			{NewDist=distance+10.5+(9.5-5/NonLinearCoeffDist)-15;}
		}
	}*/
	if (ActualWPindex==1)
		{TransformDistance = RabbitDistance/distance;}
	else
		{TransformDistance = NewDist/distance;}

	//InputStoh1 = (0*sin(2*PI*accumulatedTime)+ sin(PI/8*accumulatedTime)+ 1.5*sin(4*PI/8*accumulatedTime)+ 1.5*sin(7*PI/8*accumulatedTime)+ sin(10*PI/8*accumulatedTime)+ 0*sin(13*PI/8*accumulatedTime))/3;
	//InputStoh = 12*(r-1);//InputStoh1;

	// Slip angle - Pomak zbog vodjenja po vektoru brzine a ne headingu, za male brzine nepouzdan
	std::cout<<latVelVector<<","<<frwVelVector<<std::endl;
	VelVec = atan2(latVelVector,frwVelVector);
	if 	(fabs(frwVelVector) < 0.3)
		{VelVec = fabs(frwVelVector)/0.3*VelVec;}
	HeadingErr = DirectionToTarget - yRotation - VelVec*180/PI;

	if (HeadingErr<-180.0f)
	{HeadingErr = HeadingErr + 360;
	}

	//uvodjenje azimut pomaka rabbita zbog nelinearnosti heading skale i vodjenja po brzine a ne headingu
	HE = HeadingErr;
	azim = HE*PI/180;
	if (fabs(HeadingErr) < 90)// || HeadingErr > 270)
		{
			// uvodjenje nelinearnosti u heading skalu
			TransformAngle = 0.5*atan2((2*NonLinearCoeff*sin(2*azim)),(1-NonLinearCoeff*NonLinearCoeff+(1+NonLinearCoeff*NonLinearCoeff)*cos(2*azim)));//-azim;// ovo nije novi kut vec kut degeneracije headingerrora zbog uvodjenja nelinearnosti
		}
	else
		{TransformAngle = azim;//0.0f;
		}

	//std::cout<<180/PI*TransformAngle<<" "<<DirectionToTarget<<" "<<yRotation<<" "<<HeadingErr<<" "<<fabs(HE)<<" "<<zListenerPosRel<<" "<<xListenerPosRel<<std::endl;



	if (TaskMode == 1)
	{
	if ((accumulatedTime > KDiterTime * tDiter[iDiter]) && (iDiter < 12))
	{iDiter++;}
		objects[0].xPos = xListenerPos*(1-cos(TransformAngle))+zListenerPos*sin(TransformAngle);//-25*(1-cos((FiDiter[iDiter]+AmpDiter*sin(accumulatedTime*2*PI*FreqDiter))*PI/180));
		objects[0].zPos = zListenerPos*(1-cos(TransformAngle))-xListenerPos*sin(TransformAngle);//+25*sin((FiDiter[iDiter]+AmpDiter*sin(accumulatedTime*2*PI*FreqDiter))*PI/180);
		objects[1].xPos = 0;
		objects[1].zPos = 0;
	}
	else
		{objects[0].xPos = xListenerPos + RabbitDistance*sin(TransformAngle + yRotation*PI/180 + VelVec);//RabbitEast+(xListenerPos-RabbitEast)*(1-cos(TransformAngle))+(zListenerPos-RabbitNorth)*sin(TransformAngle);
		objects[0].zPos = zListenerPos - RabbitDistance*cos(TransformAngle + yRotation*PI/180 + VelVec);//RabbitNorth+(zListenerPos-RabbitNorth)*(1-cos(TransformAngle))-(xListenerPos-RabbitEast)*sin(TransformAngle);
		objects[0].xPos = xListenerPos + (objects[0].xPos - xListenerPos)*TransformDistance;
		objects[0].zPos = zListenerPos + (objects[0].zPos - zListenerPos)*TransformDistance;
		objects[1].xPos = RabbitEast;
		objects[1].zPos = RabbitNorth;
		objects[6].xPos = objects[0].xPos;
		objects[6].zPos = objects[0].zPos;
	}



	if (fabs(HeadingErr) < SilenceAngle)// || HeadingErr > 360-SilenceAngle)
	{
		objects[0].yPos = yListenerPos + 5000;
		objects[1].yPos = yListenerPos + 5000;
	}
	else
	{
		if (TaskMode<3) // for target and path use sound source of long distance (500m)
		{
			objects[0].yPos = yListenerPos;// Toni: ovo znaci da je objekt (izvor zvuka) uvjek u ROV ravnini po z (dubini)
			objects[6].yPos = yListenerPos + 5000;
		}
		else
		{
			objects[6].yPos = yListenerPos;
			objects[0].yPos = yListenerPos + 5000;
		}
		objects[1].yPos = yListenerPos + 5000;
	}

	// target or last WP won
	if ((fabs(xListenerPos) < 2 && fabs(zListenerPos) < 2) || ((ActualWPindex == NumberOfWP + 1) && JoyStickMode==1))
		{
			objects[0].yPos = yListenerPos + 5000;
			objects[6].yPos = yListenerPos + 5000;
			objects[1].yPos = yListenerPos;
	}

	std::cout<<distance<<" "<<HE<<" "<<yRotation<<" "<<RabbitNorth<<" "<<RabbitEast<<" "<<ActualWPindex<<std::endl;
	std::cout<<std::endl;
	/*std::cout<<objects[0].xPos<<"  "<<xListenerPos<<std::endl;
	std::cout<<std::endl;
	std::cout<<objects[0].zPos<<"  "<<zListenerPos<<std::endl;
	std::cout<<std::endl;
	std::cout<<objects[0].yPos<<"  "<<yListenerPos<<std::endl;
	std::cout<<std::endl;*/


	updateObjectSoundPos(&objects[0]);
	updateObjectSoundPos(&objects[6]);
	updateObjectSoundPos(&objects[1]);

	//std::cout<<zListenerPosRel<<" "<<xListenerPosRel<<" "<<HE<<" "<<DirectionToTarget<<" "<<objects[0].zPos<<" "<<objects[0].xPos<<std::endl;

	// Upute gore-dolje
	if (Altitude < AltitudeSetPoint - 1.0f) //OVAJ RED UBACITI ZA TEREN UMJESTO REDA ISPOD
	//if (yListenerPos < AltitudeSetPoint - 0.5f)
	{
		objects[2].xPos = xListenerPos;
		objects[2].zPos = zListenerPos;
		objects[2].yPos = yListenerPos + 5;
	}
	else
	{
		objects[2].yPos = yListenerPos + 5000;
	}
	updateObjectSoundPos(&objects[2]);
	if (Altitude > AltitudeSetPoint + 1.0f) //OVAJ RED UBACITI ZA TEREN UMJESTO REDA ISPOD
	//if (yListenerPos > AltitudeSetPoint + 0.5f)
	{
		objects[3].xPos = xListenerPos;
		objects[3].zPos = zListenerPos;
		objects[3].yPos = yListenerPos - 5;//listenerVector.x;// = ;//-22 + 8.0f * sin(accumulatedTime)
	}
		else
	{
		objects[3].yPos = yListenerPos + 5000;
	}
	updateObjectSoundPos(&objects[3]);

	// Upute lijevo-desno


		if (HeadingErr > VerbalAngle && HeadingErr < 360-VerbalAngle && sin(HeadingErr*PI/180) < 0)
	{
		objects[4].xPos = xListenerPos;
		objects[4].zPos = zListenerPos;
		objects[4].yPos = yListenerPos;
	}
		else
	{
		objects[4].yPos = yListenerPos + 5000;
	}
	updateObjectSoundPos(&objects[4]);
		if (HeadingErr > VerbalAngle && HeadingErr < 360-VerbalAngle && sin(HeadingErr*PI/180) > 0)
	{
		objects[5].xPos = xListenerPos;
		objects[5].zPos = zListenerPos;
		objects[5].yPos = yListenerPos;
	}
		else
	{
		objects[5].yPos = yListenerPos + 5000;
	}
	updateObjectSoundPos(&objects[5]);

}

void doListenerMovement()
{
	// Update user movement

	std::string SingleLog;
	float step;
	const float MOVEMENT_SPEED = 0.1f;
	float forward = 0.0f;
	switch (SpeedProfile)
	{
		case 'V':
			step=(float)INTERFACE_UPDATETIME/1000*PathVelocity+0.3*(float)INTERFACE_UPDATETIME/1000*PathVelocity*(float)sin(accumulatedTime/3);
			break;
		case 'S':
			if (xListenerPos>0)
				{step=(float)INTERFACE_UPDATETIME/1000*PathVelocity*1.2;}
			else
				{step=(float)INTERFACE_UPDATETIME/1000*PathVelocity;}
			break;
		default:
			step=(float)INTERFACE_UPDATETIME/1000*PathVelocity;
	}


	//r = rand();
	//koristenje joysticka
	LABUST::JoystickData joystickData;
if (JoyStickMode==2)
	{
	joystickData = joystick->ReadJoystickData();
	std::cout<<joystickData.axes[0]<<" "<<joystickData.axes[1]<<std::endl;
}

  Head = 0.0f;

	std::vector<labust::xml::GyrosReader> receivedData;
	comms->Receive(receivedData);
	static std::map<std::string,double> latLonMap,TARGETStatus;
	static std::map<std::string,float> rovStatus,dvlStatus;
	static std::map<std::string,std::string> rovStringifiedStatus;

	if(!receivedData.empty())
		{
			for(std::vector<labust::xml::GyrosReader>::iterator gyros = receivedData.begin(); gyros!=receivedData.end(); gyros++)
			{
				std::string label = gyros->GetLabel();

				//std::cout<<"Labela:"<<label<<", "<<filterMessageLabel<<std::endl;

				try
				{
					if(boost::iequals(label,rovMessageLabel))
					{
						rovStatus.clear();
						gyros->dictionary(rovStatus);
						gyros->dictionary(rovStringifiedStatus);
					//	dvlStatus = rovStatus;
					//	latLonMap = rovStatus;
					}
					//za fieldwork
					if(boost::iequals(label,dvlMessageLabel))
					{
						dvlStatus.clear();
						gyros->dictionary(dvlStatus);
					}
					if(boost::iequals(label,filterMessageLabel))
					{
						latLonMap.clear();
						gyros->dictionary(latLonMap);
					}

					if(boost::iequals(label,TARGETMessageLabel))
					{
						TARGETStatus.clear();
						gyros->dictionary(TARGETStatus);
					}
				}
				catch (std::exception &e)
				{
					std::cout<<"Error: "<<e.what()<<" "<<label<<" "<<std::endl;
				}
			}

			receivedData.clear();
		}



// OVAJ DIO TREBA UBACITI ZA TEREN
	//Pozicija ROVa (Listener-a) u metrima, ishodiste je u izvoru zvuka (cilja)

	ObjectLong = TARGETStatus["Northing"];
	ObjectLat = TARGETStatus["Easting"];
	ObjectDepth = TARGETStatus["Depth"];//xListenerPos = (latLonMap["LAT"] - ObjectLat) * 110000.0f;
	zPos = latLonMap["Northing"];
	if (JoyStickMode==2)
		{yListenerPos = 20;}
	else
	{	zListenerPos = -(latLonMap["Northing"] - ObjectLong); //relative to the target
		xListenerPos = (latLonMap["Easting"] - ObjectLat); //relative to the target
		yListenerPos = rovStatus["Depth"];
	}
	zRotation = rovStatus["Roll"]; //sve je izmjesano z-naprijed, x-lateral, y-gore/dolje
	xRotation = rovStatus["Pitch"];
	if (JoyStickMode==1)
	{yRotation = rovStatus["Heading"];}
	//std::cout<<yRotation<<" "<<yListenerPos<<" "<<TaskMode<<" "<<GuidanceMode<<std::endl;
	TauX = rovStatus["TauX"];
	TauY = rovStatus["TauY"];
	TauZ = rovStatus["TauZ"];
	TauK = rovStatus["TauK"];
	TauM = rovStatus["TauM"];
	TauN = rovStatus["TauN"];
	frwVelVector = rovStatus["u"];
	latVelVector = rovStatus["v"];

	//Na pocetku upisivanje WPa u log file
	if (accumulatedTime < 0.01)
		{std::string sname,strWPindex;
		int WPindex;

		// Inicijalizacija WPova iz MOOS stringa
		for (WPindex=1; WPindex < NumberOfWP +2; WPindex++)
			{
			converter.clear();
			converter<<WPindex-1;
			converter>>strWPindex;
			sname = rovStringifiedStatus["Waypoint"+strWPindex];
			int index = 0;
			index = sname.find(",");
			sname = sname.substr(index + 1);//na pocetku WP poruke je greskom zarez pa ga moramo izbaciti
			index = sname.find(",");
			converter.clear();
			SingleLog = sname.substr(0, index);
			converter<<SingleLog;
			converter>>WPnorth[WPindex];
			if (WPnorth[WPindex]==0)
			{NumberOfWP = WPindex -1;}
			else
			{WPnorth[WPindex] = -(WPnorth[WPindex]-ObjectLong);
			sname = sname.substr(index + 1);
			index = sname.find(",");
			converter.clear();
			SingleLog = sname.substr(0, index);
			converter<<SingleLog;
			converter>>WPeast[WPindex];
			WPeast[WPindex] = (WPeast[WPindex]-ObjectLat);
			sname = sname.substr(index + 1);
			converter.clear();
			SingleLog = sname.substr(0, index);
			converter<<SingleLog;
			converter>>WPz[WPindex];
			}}

		// upis vrijednosti WPa u log file
		for (WPindex=1; WPindex < NumberOfWP +1; WPindex++)
		{file<<WPnorth[WPindex];
		file<<";";
		file<<WPeast[WPindex];
		file<<";";
		file<<WPz[WPindex]<<std::endl;
		file<<";";
		//std::cout<<WPnorth[WPindex]<<" "<<WPeast[WPindex]<<" "<<WPz[WPindex]<<" "<<ObjectLong<<" "<<ObjectLat<<std::endl;
		}
	}
	Altitude = dvlStatus["Altitude"];
	if (fabs(xListenerPos - WPeast[ActualWPindex]) < 3 && fabs(zListenerPos-WPnorth[ActualWPindex]) < 3)
	{ActualWPindex += 1;}
	if (TaskMode==3)
	{if (ActualWPindex == 1 && sqrt((xListenerPos - WPeast[ActualWPindex])*(xListenerPos - WPeast[ActualWPindex]) + (zListenerPos-WPnorth[ActualWPindex])*(zListenerPos-WPnorth[ActualWPindex])) < RabbitDistance)
		{ActualWPindex += 1;}
	}
	else
	{if (ActualWPindex == 1 && sqrt((xListenerPos - WPeast[ActualWPindex])*(xListenerPos - WPeast[ActualWPindex]) + (zListenerPos-WPnorth[ActualWPindex])*(zListenerPos-WPnorth[ActualWPindex])) < 2)
		{ActualWPindex += 1;}
	}
	// Set virtual target (rabbit) position
	/*
	PathX = PathStartX*(PathStartY * xListenerPos + PathStartX * zListenerPos)/(PathStartX * PathStartX + PathStartY * PathStartY);
	PathY = PathStartY*(PathStartY * xListenerPos + PathStartX * zListenerPos)/(PathStartX * PathStartX + PathStartY * PathStartY);
	DistanceToPath = sqrt((xListenerPos - PathY) * (xListenerPos - PathY) + (zListenerPos - PathX) * (zListenerPos - PathX));*/
	if (TaskMode>1 && ActualWPindex>1)
		{PathStartY = WPeast[ActualWPindex-1] - WPeast[ActualWPindex];
		PathStartX = WPnorth[ActualWPindex-1] - WPnorth[ActualWPindex];
		}
	float kLine = PathStartY / PathStartX;
	float kLineAngle = atan2(PathStartY,PathStartX);
	aSQR = 1 + kLine*kLine;

	// PathX i Path Y su tocka projekcije ROV pozicije na path (pravi kut)
	if (TaskMode>1)
		{if (PathStartX == 0)
		{PathX = WPnorth[ActualWPindex];
		PathY =xListenerPos;}
		else
		{PathX = (kLine*(xListenerPos - WPeast[ActualWPindex] + kLine*WPnorth[ActualWPindex]) + zListenerPos)/aSQR;//(1+kLine*kLine);
		PathY = WPeast[ActualWPindex] + kLine*PathX - kLine*WPnorth[ActualWPindex];
		//PathY = PathStartY + WPeast[ActualWPindex] + kLine*PathX - kLine*PathStartX - kLine*WPnorth[ActualWPindex];
		}
	}
	else
		{PathX = PathStartX*(PathStartY * xListenerPos + PathStartX * zListenerPos)/(PathStartX * PathStartX + PathStartY * PathStartY);
		PathY = PathStartY*(PathStartY * xListenerPos + PathStartX * zListenerPos)/(PathStartX * PathStartX + PathStartY * PathStartY);
		}

	DistanceToPath = sqrt((xListenerPos - PathY) * (xListenerPos - PathY) + (zListenerPos - PathX) * (zListenerPos - PathX));


	if ((TaskMode > 1 && sqrt((zListenerPos-WPnorth[NumberOfWP])*(zListenerPos-WPnorth[NumberOfWP]) + (xListenerPos-WPeast[NumberOfWP])*(xListenerPos-WPeast[NumberOfWP])) > RabbitDistance))
		//Da li smo dosli do targeta, odnosno zadnjeg WPa
	{
	if (DistanceToPath > RabbitDistance)
		{RabbitNorth = PathX;
		RabbitEast = PathY;
		RabbitNorthOld = RabbitNorth;
		RabbitEastOld = RabbitEast;
		}
	else
		{if (TaskMode==3) //meaning trajectory tracking
		{
		if (ActualWPindex>2 && ActualWPindex>ActualWPindexOld) //if new WP
		{
			RabbitNorth = WPnorth[ActualWPindex-1] - cos(kLineAngle)*RabbitDistance;
			RabbitEast = WPeast[ActualWPindex-1] - sin(kLineAngle)*RabbitDistance;
			wait = true;
		}
		else
		{
			if (wait==false || (wait==true && fabs(HE)<10)) // if WP is cahnged recently, wait until heading is cca. inline with path segment
			{
				RabbitNorth = RabbitNorthOld - cos(kLineAngle)*step;
				RabbitEast = RabbitEastOld - sin(kLineAngle)*step;
				wait=false;
			}
		}
		RabbitNorthOld = RabbitNorth;
		RabbitEastOld = RabbitEast;
		ActualWPindexOld=ActualWPindex;
		//std::cout<<kLineAngle*180/PI<<" "<<step<<" "<<sin(kLineAngle)*step<<" "<<RabbitEast<<" "<<ActualWPindex<<std::endl;
		}
		else
			{if (PathStartX == 0)
				{RabbitNorth = WPnorth[ActualWPindex];
				if (PathStartY<0)
					{RabbitEast = xListenerPos + sqrt(RabbitDistance*RabbitDistance-(zListenerPos - RabbitNorth)*(zListenerPos - RabbitNorth));}
				else
					{RabbitEast = xListenerPos - sqrt(RabbitDistance*RabbitDistance-(zListenerPos - RabbitNorth)*(zListenerPos - RabbitNorth));}
				}
			else
				{bSQR = -2*((PathX-WPnorth[ActualWPindex]) + kLine*(PathY-WPeast[ActualWPindex]));
				cSQR = (PathX-WPnorth[ActualWPindex])*(PathX-WPnorth[ActualWPindex])+(PathY-WPeast[ActualWPindex])*(PathY-WPeast[ActualWPindex])+DistanceToPath*DistanceToPath-RabbitDistance*RabbitDistance;
				RabbitNorth1 = (-bSQR+sqrt(bSQR*bSQR-4*aSQR*cSQR))/(2*aSQR);
				RabbitNorth2 = (-bSQR-sqrt(bSQR*bSQR-4*aSQR*cSQR))/(2*aSQR);
				if (abs(RabbitNorth1)<abs(RabbitNorth2))
					{RabbitNorth = RabbitNorth1 + WPnorth[ActualWPindex];
					RabbitEast = kLine * RabbitNorth1 + WPeast[ActualWPindex];}
				else
					{RabbitNorth= RabbitNorth2 + WPnorth[ActualWPindex];
					RabbitEast = kLine * RabbitNorth2 + WPeast[ActualWPindex];}
				}
			}
		}
	}
	else
	{RabbitNorth = WPnorth[ActualWPindex];
		RabbitEast = WPeast[ActualWPindex];
	}
	if (TaskMode == 1) // if guidance to Target, rabbit is target
		{RabbitNorth = 0.0f;
		RabbitEast = 0.0f;
	}
	if (ActualWPindex == 1) //guidance to the first WP of the path
		{RabbitNorth = WPnorth[1];
		RabbitEast = WPeast[1];
		RabbitNorthOld = RabbitNorth;
		RabbitEastOld = RabbitEast;
	}

	ti += (float)INTERFACE_UPDATETIME / 1000.0f;
	if (ti > 2)
	{r = rand()%3;
	ti = 0.0f;}
	//std::cout<<r-1<<" "<<ti<<std::endl;//<<RabbitNorth<<" "<<RabbitEast<<" "<<PathX<<" "<<PathY<<" "<<PathStartX<<" "<<PathStartY<<" "<<ActualWPindex<<" "<<DistanceToPath<<" "<<zListenerPos<<" "<< xListenerPos<<std::endl;

	//std::cout<<latLonMap["Northing"]<<" "<<latLonMap["Easting"]<<" "<<rovStatus["Depth"]<<" "<<TARGETStatus["Northing"]<<" "<<TARGETStatus["Easting"]<<" "<<TARGETStatus["Depth"]<<" "<<rovStatus["Heading"]<<" "<<dvlStatus["Altitude"]<<" "<<AltitudeSetPoint<<std::endl;
	if (JoyStickMode==2)
		{if (abs(joystickData.axes[1])>2000)
			{forw = -0.1*joystickData.axes[1]/intMax;}
		else
		{forw = 0;}
	}
	if (JoyStickMode==2)
		{
	if (abs(joystickData.axes[0])>2000)
		{
		yRotation += 1.0f*joystickData.axes[0]/intMax;
		if (yRotation < 0)
		{yRotation = yRotation + 360;}
		else
		{
				if (yRotation > 360)
			{yRotation = yRotation - 360;}
		}
	}

}
// OVAJ DIO TREBA IZBACITI ZA TEREN, OVO JE VOZNJA TIPKOVNICOM
	/*if (moveForward)
    {
		forward += (MOVEMENT_SPEED * (moveFast ? 2.0f : 1.0f));
    }*/

	/*if (moveBackward)
    {
		forward -= (MOVEMENT_SPEED * (moveFast ? 2.0f : 1.0f));
    }*/
	/*if (moveRotateClock)
    {
		yRotation += 0.3f;

    }
	if (moveRotateAntiClock)
    {
		yRotation += -0.3f;
    }*/

	/*
	float right = 0.0f;
	if (moveLeft)
    {
		right -= (MOVEMENT_SPEED * (moveFast ? 2.0f : 1.0f));
    }
	if (moveRight)
    {
		right += (MOVEMENT_SPEED * (moveFast ? 2.0f : 1.0f));
    }
	*/
	float up = 0.0f;
	/*
	if (moveUp)
    {
		up += (MOVEMENT_SPEED * 0.2f * (moveFast ? 2.0f : 1.0f));
    }
	if (moveDown)
    {
		up -= (MOVEMENT_SPEED * 0.2f * (moveFast ? 2.0f : 1.0f));
    }
	float right = 0.0f;
if (JoyStickMode==2)
	//Lateral movement, TONI: kada tipkama biras desno/lijevo, ovdje nije petljao sa pith-om i roll-om
	{float xRight = (float)cos(yRotation * (PI / 180.0f));
	float yRight = 0.0f;
	float zRight = (float)sin(yRotation * (PI / 180.0f));

	xListenerPos += xRight * right;
	yListenerPos += yRight * right;
	zListenerPos += zRight * right;

	//Forward-backward movement TONI: kada tipkama biras naprijed/nazad, ovdje je petljao sa pith-om i roll-om
	//float xForward = (float)sin(yRotation * (PI / 180.0f)) * cos(xRotation  * (PI / 180.0f));
	float xForward = (float)sin(yRotation * (PI / 180.0f)) * cos(zRotation  * (PI / 180.0f));
	float yForward = -(float)sin(xRotation  * (PI / 180.0f));
	float zForward = -(float)cos(yRotation * (PI / 180.0f)) * cos(xRotation  * (PI / 180.0f));

	xListenerPos += xForward * forward;
	yListenerPos += yForward * forward;
	zListenerPos += zForward * forward;}
	*/
// DO OVDJE TREBA IZBACITI ZA TEREN

// OVO OSTAJE I ZA TEREN I ZA SIMULACIJU
	lastyRotation=yRotation; //Toni spremi stvarni heading
	// simulacija

	//Lateral movement, TONI: Isto kao gore samo sa ukljucenim pokretom glave
	float xRight = (float)cos(yRotation * (PI / 180.0f));
	float yRight = 0.0f;
	float zRight = (float)sin(yRotation * (PI / 180.0f));

	//Forward-backward movement TONI: Isto kao gore samo sa ukljucenim pokretom glave
	float xForward = (float)sin(yRotation * (PI / 180.0f)) * cos(xRotation  * (PI / 180.0f));
	float yForward = -(float)sin(xRotation  * (PI / 180.0f));
	float zForward = -(float)cos(yRotation * (PI / 180.0f)) * cos(xRotation  * (PI / 180.0f));

	if (JoyStickMode==2)
		{xListenerPos += xForward * forw;
		yListenerPos += yForward * forw;
		zListenerPos += zForward * forw;}

	yRotation=lastyRotation; //Toni: vrati pravi heading

	//Up-Down movement simulacija
	/*yListenerPos += up;

	if (yListenerPos < 1.0f)
    {
		yListenerPos = 1.0f;
    }*/

	if (yListenerPos < 0.0f)
    {
		yListenerPos = 0.0f;
    }

	// cross product TONI????
	float xUp = yRight * zForward - zRight * yForward;
	float yUp = zRight * xForward - xRight * zForward;
	float zUp = xRight * yForward - yRight * xForward;

	/*std::cout<<xForward<<" "<<yForward<<" "<< zForward<<std::endl;
	std::cout<<std::endl;
	std::cout<<xUp<<" "<<yUp<<" "<< zUp<<std::endl;
	std::cout<<std::endl;*/

		// Racunanje heading-a prema target-u ili rabbit-u
	if (TaskMode ==1)
	{	if (JoyStickMode == 1)
			{zListenerPosRel = zListenerPos;
			xListenerPosRel = xListenerPos;}
		else
		{zListenerPosRel = 0;//20*cos(PI/180*InputStoh);
		xListenerPosRel = -25;//20*sin(PI/180*InputStoh);
	}
	}
	else
		{zListenerPosRel = zListenerPos - RabbitNorth;
		xListenerPosRel = xListenerPos - RabbitEast;}

	DirectionToTarget = atan2(-xListenerPosRel,zListenerPosRel)*180.0f/PI;

//OVAJ DIO SE KORISTI I ZA TEREN I ZA SIMULACIJU
	// Update listener
	{
		//TONI nova pozicija slusaca, koristi se i za Teren i simulaciju
		FMOD_VECTOR listenerVector;
		if (JoyStickMode==2)
		{listenerVector.x = xListenerPosRel;
		listenerVector.y = yListenerPos;
		listenerVector.z = zListenerPosRel;}
		else
		{listenerVector.x = xListenerPos;
		listenerVector.y = yListenerPos;
		listenerVector.z = zListenerPos;}

		static FMOD_VECTOR lastpos = { 0.0f, 0.0f, 0.0f };
		static bool bFirst = true;

		FMOD_VECTOR forward;
		FMOD_VECTOR up;
		FMOD_VECTOR vel;

		forward.x = xForward;
		forward.y = yForward;
		forward.z = zForward;
		up.x = xUp;
		up.y = yUp;
		up.z = zUp;

		// ********* NOTE ******* READ NEXT COMMENT!!!!!
		// vel = how far we moved last FRAME (m/f), then time compensate it to SECONDS (m/s).
		//TONI brzina slusaca
		vel.x = (listenerVector.x - lastpos.x) * (Doppler * 1000.0f / (float)INTERFACE_UPDATETIME); // BRZINA UVECANA RADI UVODJENJA POJACANOG DOPPLERA
		vel.y = (listenerVector.y - lastpos.y) * (Doppler * 1000.0f / (float)INTERFACE_UPDATETIME);
		vel.z = (listenerVector.z - lastpos.z) * (Doppler * 1000.0f / (float)INTERFACE_UPDATETIME);

		if (bFirst)
		{
			bFirst = false;
			vel.x = 0;
			vel.y = 0;
			vel.z = 0;
			HeadIni = Head;
		}
		if (vel.x==0 && vel.z==0)
					{velAtitude = lastvelAtitude;}
		else
		{
			if (vel.z < 0.0f)
				{velAtitude = atan(-vel.x/vel.z);}
			else
			{
				if (vel.z == 0.0f)
				{
					if (vel.x < 0.0f)
						{velAtitude = 1.5*PI;}
					else
						{velAtitude = 0.5*PI;}
				}
				else
				{velAtitude = atan(-vel.x/vel.z)+PI;}
			}
		velAtitude = fmod(180*velAtitude/PI+360,360);
		lastvelAtitude = velAtitude;
		}
		//std::cout<<yRotation<<" "<<forw<<" "<<zListenerPos<<" "<<xListenerPos<<" "<<std::endl;

		/*static FMOD_VECTOR lastVel = { 0.0f, 0.0f, 0.0f };
		// store pos and vel for next time

		if (lastVel.x != 0.0f || lastVel.y != 0.0f || lastVel.z != 0.0f)
		{
			if (vel.x == 0.0f && vel.y == 0.0f && vel.z == 0.0f)
			{
				int test = 0;
			}
		}
		lastVel = vel;*/
		lastpos = listenerVector;
		lastObjectLong = zPos;

		FMOD_RESULT result = fmodSystem->set3DListenerAttributes(0, &listenerVector, &vel, &forward, &up);
		ERRCHECK(result);

		file<<key1;
		file<<';';
		converter<<TauX;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<TauY;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<TauZ;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<TauN;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<listenerVector.z;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<listenerVector.x;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<listenerVector.y;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<yRotation;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<DistanceToPath;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<Altitude;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<objects[0].zPos;//ovo je ustvari x pozicija
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<objects[0].xPos; //ovo je ustvari y pozicija
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<objects[0].yPos;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<Head-HeadIni; //pomak glave
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<DirectionToTarget; //
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<RabbitNorth;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';
		converter<<RabbitEast;
		converter>>SingleLog;
		converter.clear();
		file<<SingleLog+';';

	}
}

void doUpdateVolume()
{
    //TONI pojacavanje i stisavanje zvuka tipke Z/z
	if (ambientVolUp)
    {
        char volumestring[64];

        ambientVolume += 0.025;
        if (ambientVolume > 1.0f)
        {
            ambientVolume = 1.0f;
        }

        sprintf(volumestring, "Ambient Volume: %0.3f", ambientVolume);

        #ifdef SHOW_GUI_DEBUG_TEXT
        statusText.push(volumestring);
        #endif
    }
    else if (ambientVolDown)
    {
        char volumestring[64];

        ambientVolume -= 0.025;
        if (ambientVolume < 0.0f)
        {
            ambientVolume = 0.0f;
        }

        sprintf(volumestring,"Ambient Volume: %0.3f", ambientVolume);

        #ifdef SHOW_GUI_DEBUG_TEXT
        statusText.push(volumestring);
        #endif
    }
}

void timerFunc(int nValue)
{
    std::string TimeLog;
	static bool firsttime = true;
	FMOD_RESULT result;

	//doGeometryMovement();

	doSoundMovement();
	doListenerMovement();
    doUpdateVolume();

    result = FMOD::NetEventSystem_Update();
	ERRCHECK(result);
    result = fmodEventSystem->update();
	ERRCHECK(result);

	accumulatedTime += (float)INTERFACE_UPDATETIME / 1000.0f;

	converter<<accumulatedTime;
	converter>>TimeLog;
	converter.clear();
	file<<TimeLog+';'<<std::endl;

	glutPostRedisplay();
	glutTimerFunc(INTERFACE_UPDATETIME, timerFunc, 0);

#if 0
    if (firsttime)
    {
        SetWindowPos(
          GetForegroundWindow(),    // handle to window
          HWND_TOPMOST,             // placement-order handle
          0,                        // horizontal position
          0,                        // vertical position
          width,                    // width
          height,                   // height
          SWP_NOMOVE
        );
        firsttime = false;
    }
#endif
}

void keyboardFunc(unsigned char key, int x, int y)
{
	key1 = key;
	switch (key)
	{
	case 'w':
		moveForward = true;
		break;
	case 's':
		moveBackward = true;
		break;
	case 'j':
		moveRotateAntiClock = true;
		break;
	case 'l':
		moveRotateClock = true;
		break;
	case 'a':
		moveLeft = true;
		break;
	case 'd':
		moveRight = true;
		break;
	case ' ':
		moveUp = true;
		break;
	case 'c':
		moveDown = true;
        break;
    case 'z' :
        ambientVolDown = true;
        break;
    case 'Z' :
        ambientVolUp = true;
        break;
    case 'v':
        masterVolDown = true;
        break;
    case 'V':
        masterVolUp = true;
        break;

    case 'p':
        {
            float value;

            fmodEventParameter->getValue(&value);

            fmodEventParameter->setValue(value - 20.0f);
        }
        break;

    case 'P':
        {
            float value;

            fmodEventParameter->getValue(&value);

            fmodEventParameter->setValue(value + 20.0f);
        }
        break;

    case 'f':
        moveFast = true;
        break;
	}
}

void keyboardUpFunc(unsigned char key, int x, int y)
{
	key1 = 0;
	switch (key)
	{
	case 'w':
		moveForward = false;
		break;
	case 's':
		moveBackward = false;
		break;
	case 'j':
		moveRotateAntiClock = false;
		break;
	case 'l':
		moveRotateClock = false;
	case 'a':
		moveLeft = false;
		break;
	case 'd':
		moveRight = false;
        break;
	case ' ':
		moveUp = false;
		break;
	case 'c':
		moveDown = false;
        break;
    case 'z' :
        ambientVolDown = false;
        break;
    case 'Z' :
        ambientVolUp = false;
        break;
    case 'v':
        masterVolDown = false;
        break;
    case 'V':
        masterVolUp = false;
        break;
    case 'f':
        moveFast = false;
        break;
    case 27:
        exit(1);
        break;
	}
}

void specialKeyFunc(int key, int x, int y)
{
  /*  switch (key)
    {

    }*/

    key = key;
}

void specialKeyUpFunc(int key, int x, int y)
{
    switch (key)
    {
    case GLUT_KEY_F1:
        showhelp = !showhelp;
        break;
    case GLUT_KEY_F2:
        if(fullscreen)
        {
            glutPositionWindow(20,40);
            glutReshapeWindow(500,500);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
        break;
    case GLUT_KEY_F3:
        rendermode = (rendermode == GL_LINE ? GL_FILL : GL_LINE);
        break;
    case GLUT_KEY_F11:
        showdebug = !showdebug;
        break;
    case GLUT_KEY_F8:
        rotatingMesh.geometry->setActive(false);
        walls.geometry->setActive(false);
        break;
    case GLUT_KEY_F9:
        rotatingMesh.geometry->setActive(true);
        walls.geometry->setActive(true);
        break;
    }
}

void display(void)
{
    // Show listener position
    {
        char text[64];

        sprintf(text, "Listener Pos: (%.2f, %.2f, %.2f)", xListenerPos, yListenerPos, zListenerPos);
        #ifdef SHOW_GUI_DEBUG_TEXT
        debugText.push(std::string(text));
        #endif
    }
    // Show cpu usage position
    {
        char text[64];
        float dsp, stream, geometry, update, total;

        fmodSystem->getCPUUsage(&dsp, &stream, &geometry, &update, &total);
        sprintf(text, "CPU Usage : (%.2f, %.2f, %.2f, %.2f, %.2f)", dsp, stream, geometry, update, total);
        #ifdef SHOW_GUI_DEBUG_TEXT
        debugText.push(std::string(text));
        #endif
    }

    /*
        3D RENDERING
    */

	// update view
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(
		60.0,								// fov
		(float)width / (float)height,		// aspect
		0.1,								// near
		500.0								// far
		);


	glRotatef(xRotation, 1.0f, 0.0f, 0.0f);
	glRotatef(yRotation, 0.0f, 1.0f, 0.0f);
	glTranslatef(-xListenerPosRel, -yListenerPos, -zListenerPosRel);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// clear
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.4, 0.6f, 1.0f, 0.0f);

	glEnable(GL_TEXTURE_2D);
	glBindTexture( GL_TEXTURE_2D, texture );

	// draw geometry
	drawGeometry(walls);
	drawGeometry(rotatingMesh);

    // draw skybox
    drawSkyBox();

	glDisable(GL_TEXTURE_2D);

	// draw sound objects
	int object;
	if (TaskMode == 3)
		{object=6;}
	else
		{object=0;}
	//for (object = 0; object < NUM_OBJECTS-6; object++)
	{
        char txt[256];


		float directOcclusion = 1.0f;
		float reverbOcclusion = 1.0f;

		// set colour baced on direct occlusion
//		objects[object].channel->getOcclusion(&directOcclusion, &reverbOcclusion);
		float intensity = 1.0f;// - directOcclusion;

		glPolygonMode(GL_FRONT_AND_BACK, rendermode);
		glPushMatrix();
		//glTranslatef(objects[object].xPos, objects[object].yPos, objects[object].zPos);
		glTranslatef(0, objects[object].yPos, 0);

        sprintf(txt, "Sound object (%d): %.2f, %.2f, %.2f", object, objects[object].xPos, objects[object].yPos, objects[object].zPos);
        #ifdef SHOW_GUI_DEBUG_TEXT
        debugText.push(std::string(txt));
        #endif

		glPushAttrib(GL_LIGHTING_BIT);

		intensity *= 0.75f;
		float color[4] = { intensity, intensity, 0.0f, 0.0f };
		if(object == 0)
		{//objekt broj 0 je ofarban crveno
			color[1] = 0;
			//std::cout<<objects[0].zPos<<"  "<<objects[0].xPos<<std::endl;
			//std::cout<<std::endl;
		}
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
		intensity *= 0.5f;
		float ambient[4] = { intensity, intensity, 0.0f, 0.0f };
		if(object == 0)
		{//objekt broj 0 je ofarban crveno
			ambient[1] = 0;
		}

		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);


		glRotatef(accumulatedTime * 200.0f, 0.0f, 1.0f, 0.0f);
        {
            FMOD_VECTOR soundorientation;
            float rad = (accumulatedTime * 200.0f);

            rad *= 3.14159f;
            rad /= 180.0f;

            soundorientation.x = 0;//sinf(rad);
            soundorientation.y = 0;
            soundorientation.z = 0;//cosf(rad);

            objects[object].event->set3DAttributes(0, 0, &soundorientation);
        }

		glutSolidTeapot(1.f);//SolidTorus(0.15f, 0.6f, 8, 16);
		glPopAttrib();
		glPopMatrix();
	}

    /*
        Draw blue transparent blue quads to entry to water room
    */
    drawWaterRoom();

    /*
        Do water effects if we are in the water room
    */
    inWater();


    /*
        2D RENDERING
    */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0f, (GLsizei)width, 0.0f, (GLsizei)height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
    glDisable(GL_DEPTH_TEST);

    /*
        Render text
    */
    renderUiText();


    glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
	glPolygonMode(GL_FRONT_AND_BACK, rendermode);

	// finish
	glutSwapBuffers();
}

void reshapeFunc(int w, int h)
{
	width = w;
	height = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
}

GLuint loadTexturePNG(const char *filename)
{
    GLuint texture = SOIL_load_OGL_texture(filename, 0, 0, 0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    return texture;
}

GLuint loadTexture(const char *filename)
{
    GLuint texture;
    int width;
	int height;
    unsigned char *data;
    FILE *file;

    // open texture data
    file = fopen( filename, "rb" );
    if ( file == NULL )
		return 0;

    width = 128;
    height = 128;
    data = (unsigned char*)malloc(width * height * 3);

    fread(data, width * height * 3, 1, file );
    fclose(file);

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    gluBuild2DMipmaps(
		GL_TEXTURE_2D,
		3,
		width,
		height,
        GL_RGB,
		GL_UNSIGNED_BYTE,
		data);

    free(data);

    return texture;
}


void init(void)
{
    FMOD_RESULT      result;
    bool             listenerflag = true;
    FMOD_VECTOR      listenerpos  = { 0.0f, 0.0f, 0.0f };
    FMOD_SPEAKERMODE speakermode;

    printf("==================================================================\n");
    printf("3D example.  Copyright (c) Firelight Technologies 2004-2011.\n");
    printf("==================================================================\n\n");

    result = FMOD::EventSystem_Create(&fmodEventSystem);
    ERRCHECK(result);
    result = FMOD::NetEventSystem_Init(fmodEventSystem);
    ERRCHECK(result);
    result = fmodEventSystem->getSystemObject(&fmodSystem);
    ERRCHECK(result);
    result = fmodSystem->getDriverCaps(0,0,0,&speakermode);
    ERRCHECK(result);
    result = fmodSystem->setSpeakerMode(speakermode);
    ERRCHECK(result);
    //result = fmodEventSystem->init(32, FMOD_INIT_3D_RIGHTHANDED | FMOD_INIT_SOFTWARE_OCCLUSION | FMOD_INIT_SOFTWARE_HRTF, 0, FMOD_EVENT_INIT_NORMAL);
    result = fmodEventSystem->init(32, FMOD_INIT_3D_RIGHTHANDED, 0, FMOD_EVENT_INIT_NORMAL);
	//result = fmodEventSystem->init(32, FMOD_INIT_3D_RIGHTHANDED | FMOD_INIT_SOFTWARE_OCCLUSION | FMOD_INIT_SOFTWARE_HRTF | FMOD_INIT_DSOUND_HRTFFULL, 0, FMOD_EVENT_INIT_NORMAL);
    ERRCHECK(result);
    //result = fmodEventSystem->setMediaPath("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media1/");
    ERRCHECK(result);
    result = fmodEventSystem->load("examples1.fev", 0, &fmodEventProject);
    ERRCHECK(result);
    result = fmodEventProject->getGroup("FeatureDemonstration/3D Events", true, &fmodEventGroup);
    ERRCHECK(result);

    /*
        Create a programmer created lowpass filter to apply to everything.
    */
    result = fmodSystem->createDSPByType(FMOD_DSP_TYPE_LOWPASS, &global_lowpass);
    ERRCHECK(result);

    result = global_lowpass->setParameter(FMOD_DSP_LOWPASS_CUTOFF, 1000);
    ERRCHECK(result);

    result = global_lowpass->setBypass(true);   // turn it off to start with.
    ERRCHECK(result);

    result = fmodSystem->addDSP(global_lowpass, 0);
    ERRCHECK(result);


	initObjects();

	result = fmodSystem->setGeometrySettings(200.0f);
    ERRCHECK(result);

    printf("Loading geometry...");

	// load objects
	//initGeometry("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/walls.bin", walls, true);
	//initGeometry("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/center.bin", rotatingMesh);
	initGeometry("files/walls.bin", walls, true);
	initGeometry("files/center.bin", rotatingMesh);

    printf("done.\n");

    /*
        Load textures
    */
    printf("Loading textures...\n");

	/*texture = loadTexture("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/texture.img");
    skyboxTexture[0] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/front.png");
    skyboxTexture[1] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/right.png");
    skyboxTexture[2] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/back.png");
    skyboxTexture[3] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/left.png");
    skyboxTexture[4] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/top.png");
    skyboxTexture[5] = loadTexturePNG("C:/Program Files/FMOD SoundSystem/FMOD Programmers API Win32/fmoddesignerapi/examples/media/skybox/bluesky/bottom.png");
	*/
	texture = loadTexture("files/texture.img");
    skyboxTexture[0] = loadTexturePNG("files/front.png");
    skyboxTexture[1] = loadTexturePNG("files/right.png");
    skyboxTexture[2] = loadTexturePNG("files/back.png");
    skyboxTexture[3] = loadTexturePNG("files/left.png");
    skyboxTexture[4] = loadTexturePNG("files/top.png");
    skyboxTexture[5] = loadTexturePNG("files/bottom.png");


    printf("done.\n");

	// setup lighting
	GLfloat lightDiffuse[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat lightPosition[] = {300.0, 1000.0, 400.0, 0.0};
	GLfloat lightAmbiant[] = {1.25, 1.25, 1.25, 1.0};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbiant);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0f);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

    // setup fog(water)
    GLfloat	fogColor[4] = {0.0f,0.1f,0.9f,1.0f};

	glFogi(GL_FOG_MODE, GL_EXP);	    // Fog Mode
	glFogfv(GL_FOG_COLOR, fogColor);    // Set Fog Color
	glFogf(GL_FOG_DENSITY, 0.15f);	    // How Dense Will The Fog Be
	glHint(GL_FOG_HINT, GL_DONT_CARE);	// Fog Hint Value
	glFogf(GL_FOG_START, 0.0f);			// Fog Start Depth
	glFogf(GL_FOG_END, 1.0f);			// Fog End Depth

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

GlutCloseClass::~GlutCloseClass()
{
    glDeleteTextures( 1, &texture );

	freeGeometry(walls);
	freeGeometry(rotatingMesh);

    fmodEventSystem->release();
    FMOD::NetEventSystem_Shutdown();

    //_CrtDumpMemoryLeaks();
}
}

using namespace FMOD;

int	main(int argc, char **argv)
{
	ros::init(argc,argv,"augmented_acoustics");

	try
	{
	//ako nema command line parametara, ppita se path do config fajla
	std::string path, configToUse;
		if(argc < 2)
		{
			std::cout<<"Please enter path to config file for Audio experiment"<<std::endl;
			std::cin>>path;
		}
		else
		{//inace se prvi param uzme kao path
			path = argv[1];
		}

		if(argc < 3)
		{//
			configToUse = "";
		}
		else
		{//A drugi kao ime configa
			configToUse = argv[2];
		}

		labust::xml::Reader reader(path, true);
		//kreiraj xml citac
		std::string configQuery;
		if (configToUse.empty())
		{
			configQuery = "/configurations/config[@type='program']";
		}
		else
		{
			configQuery = "/configurations/config[@type='program' and @name='"+configToUse+"']";
		}

		_xmlNode* configNode = NULL;
		int sampleParam;
		if (reader.try_value(configQuery, &configNode))
   		{//pozicioniraj se u cvor koji ima config za moj program i ucitaj varijable za PROGRAM
      		reader.useNode(configNode);
			reader.try_value("param[@name='FilterMessage']/@value", &filterMessageLabel);
			reader.try_value("param[@name='RovMessage']/@value", &rovMessageLabel);
			reader.try_value("param[@name='DvlMessage']/@value", &dvlMessageLabel);
			reader.try_value("param[@name='TARGETMessage']/@value", &TARGETMessageLabel);
			reader.try_value("param[@name='ObjectLat']/@value", &ObjectLat);
			reader.try_value("param[@name='ObjectLong']/@value", &ObjectLong);
			reader.try_value("param[@name='AltitudeSetPoint']/@value", &AltitudeSetPoint);
			reader.try_value("param[@name='Doppler']/@value", &Doppler);
			reader.try_value("param[@name='Silence_angle']/@value", &SilenceAngle);
			reader.try_value("param[@name='Verbal_angle']/@value", &VerbalAngle);
			reader.try_value("param[@name='NonLinear_coeff']/@value", &NonLinearCoeff);
			reader.try_value("param[@name='Guidance_Mode']/@value", &GuidanceMode);
			reader.try_value("param[@name='Task_Mode']/@value", &TaskMode);
			reader.try_value("param[@name='K_Diter_Time']/@value", &KDiterTime);
			reader.try_value("param[@name='AmpDiter']/@value", &AmpDiter);
			reader.try_value("param[@name='FreqDiter']/@value", &FreqDiter);
			reader.try_value("param[@name='JoyStick_Mode']/@value", &JoyStickMode);
			reader.try_value("param[@name='PathVelocity']/@value", &PathVelocity);
			reader.try_value("param[@name='SpeedProfile']/@value", &SpeedProfile);
			reader.try_value("param[@name='NonLinear_coeff_distance']/@value", &NonLinearCoeffDist);
			reader.try_value("param[@name='Kintegral']/@value", &Ki);
			reader.try_value("param[@name='Sound']/@value", &Sound);
			reader.try_value("param[@name='PathStartX']/@value", &PathStartX1);
			reader.try_value("param[@name='PathStartY']/@value", &PathStartY1);
			reader.try_value("param[@name='RabbitDistance']/@value", &RabbitDistance);
		}
		//vrati se u cvor i posalji config u moos comms interface
		reader.useNode(configNode);
		comms = new LABUST::COMMUNICATION::GyrosMoosCommsInterface(reader);

		//reader.useNode(configNode);//,"PalicaMS"); //ovo ti je citac xml fajla
		//LABUST::COMMUNICATION::GyrosMoosCommsInterface comms(reader, moosConfig);	


		if (JoyStickMode==2)
		{
			std::string joystickConfig("PalicaMS");
			reader.useNode(configNode);
			joystick = new LABUST::JoystickReader(reader,joystickConfig);
		}




		// Inicijalizacija joysticka

	srand((unsigned)time(0));

	//Upisivanje zadanih parametara u Log_Doc file 

	file<<"ImuCom ;Doppler ;SilenceAngle ;VerbalAngle ;NonLinearCoeff ;GuidanceMode ;TaskMode ;RabbitDistance ;PathVelocity ;NonLinearCoeffDistance; PathStartX ;PathStartY"<<std::endl;
	file<<Doppler<<" ;"<<SilenceAngle<<" ;"<<VerbalAngle<<" ;"<<NonLinearCoeff<<" ;"<<GuidanceMode<<" ;"<<TaskMode<<" ;"<<RabbitDistance<<" ;"<<PathVelocity<<" ;"<<NonLinearCoeffDist<<" ;"<<PathStartX<<" ;"<<PathStartY<<" ;"<<std::endl;
	file<<"WayPointx;WayPointy;WayPointz"<<std::endl;

	//Upisivanje podataka u Header Log_Doc file
	file<<"Control_signal_field ;TauX ;TauY ;TauZ ;TauYaw ;ROV_N ;ROV_E ;ROV_Z ;Azimuth ;DistanceToPath ;Altitude ;TargetN ;TargetE ;TargetZ ;Head ;DirToTarget ;RabbitN ;RabbitE ;Time ;"<<std::endl;


	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Audio guidance");
	glutDisplayFunc(display);	
	glutReshapeFunc(reshapeFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutKeyboardFunc(keyboardFunc);
	glutKeyboardUpFunc(keyboardUpFunc);
    glutSpecialFunc(specialKeyFunc);
    glutSpecialUpFunc(specialKeyUpFunc);

	glutTimerFunc(INTERFACE_UPDATETIME, timerFunc, 0);

	init();

    usleep(1000*1000);

	glutMainLoop();

	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
	return 0;
}
