/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  Created on: 13.06.2013.
*  Author: Dula Nad
*********************************************************************/
#include <labust/gui/AAGui.hpp>
#include <GL/glut.h>
#include <SOIL/SOIL.h>
#include <fmod/fmod.hpp>
#include <fmod/fmod_errors.h>
#include <fmod/fmod_event.hpp>
#include <fmod/fmod_event_net.hpp>
#include <iostream>
#include <fstream>
#include <cmath>

using labust::gui::AAGui;

//int width = 500;
//int height = 500;
//
//// textures
//GLuint texture;
//GLuint skyboxTexture[6];
//
//// geometry structers for loading and drawing
//struct Polygon
//{
//	int numVertices;
//	int indicesOffset;
//	float directOcclusion;
//	float reverbOcclusion;
//	FMOD_VECTOR normal;
//};
//
//struct Mesh
//{
//	int numVertices;
//	FMOD_VECTOR *vertices;
//	float (*texcoords)[2];
//	int numPolygons;
//	Polygon* polygons;
//	int numIndices;
//	int *indices;
//	FMOD::Geometry *geometry;
//};
//
//class GlutCloseClass
//{
//  public:
//    GlutCloseClass() {};
//   ~GlutCloseClass();
//};
//
//GlutCloseClass gCloseObject;
//
//Mesh walls;
//Mesh rotatingMesh;
//
//void display(void);
//void reshapeFunc(int w, int h);
//void mouseFunc(int button, int state, int x, int y);
//void motionFunc(int x, int y);
//void specialKeyUpFunc(int key, int x, int y);
void specialKeyFunc(int key, int x, int y);
//void keyboardUpFunc(unsigned char key, int x, int y);
//void keyboardFunc(unsigned char key, int x, int y);
//void timerFunc(int nValue);
GLuint loadTexture(const char *filename);
GLuint loadTexturePNG(const char *filename);
//void initGeometry(const char* szFileName, Mesh& mesh, bool alter = false);
//void ERRCHECK(FMOD_RESULT result);

int INTERFACE_UPDATETIME = 15;

AAGui::AAGui()
{
	this->init();
}

void AAGui::start()
{
	glutMainLoop();
}

AAGui::AAGui(const std::string& path)
{
	this->init();
	this->loadTextures(path);
}

int width = 500;
int height = 500;

void AAGui::init()
{
	int argc(0);
	char* argv[0];
	//No glut configuration is needed really
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Audio guidance");
//	glutDisplayFunc(display);
//	glutReshapeFunc(reshapeFunc);
//	glutMouseFunc(mouseFunc);
//	glutMotionFunc(motionFunc);
//	glutKeyboardFunc(keyboardFunc);
//	glutKeyboardUpFunc(keyboardUpFunc);
    glutSpecialFunc(specialKeyFunc);
//  glutSpecialUpFunc(specialKeyUpFunc);
}

void AAGui::loadTextures(const std::string& path)
{
  std::cout<<"Loading textures from "<<path<<std::endl;
	texture = loadTexture((path + "/texture.img").c_str());
  skyboxTexture[0] = loadTexturePNG((path + "/front.png").c_str());
  skyboxTexture[1] = loadTexturePNG((path + "/right.png").c_str());
  skyboxTexture[2] = loadTexturePNG((path + "/back.png").c_str());
  skyboxTexture[3] = loadTexturePNG((path + "/left.png").c_str());
  skyboxTexture[4] = loadTexturePNG((path + "/top.png").c_str());
  skyboxTexture[5] = loadTexturePNG((path + "/bottom.png").c_str());
  std::cout<<"done.\n";

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

void AAGui::drawSkyBox(float xListenerPos, float yListenerPos)
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

//
//void display(void)
//{
//    // Show listener position
//    {
//        char text[64];
//
//        sprintf(text, "Listener Pos: (%.2f, %.2f, %.2f)", xListenerPos, yListenerPos, zListenerPos);
//        #ifdef SHOW_GUI_DEBUG_TEXT
//        debugText.push(std::string(text));
//        #endif
//    }
//    // Show cpu usage position
//    {
//        char text[64];
//        float dsp, stream, geometry, update, total;
//
//        fmodSystem->getCPUUsage(&dsp, &stream, &geometry, &update, &total);
//        sprintf(text, "CPU Usage : (%.2f, %.2f, %.2f, %.2f, %.2f)", dsp, stream, geometry, update, total);
//        #ifdef SHOW_GUI_DEBUG_TEXT
//        debugText.push(std::string(text));
//        #endif
//    }
//
//    /*
//        3D RENDERING
//    */
//
//	// update view
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(
//		60.0,								// fov
//		(float)width / (float)height,		// aspect
//		0.1,								// near
//		500.0								// far
//		);
//
//
//	glRotatef(xRotation, 1.0f, 0.0f, 0.0f);
//	glRotatef(yRotation, 0.0f, 1.0f, 0.0f);
//	glTranslatef(-xListenerPosRel, -yListenerPos, -zListenerPosRel);
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//
//	// clear
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	glClearColor(0.4, 0.6f, 1.0f, 0.0f);
//
//	glEnable(GL_TEXTURE_2D);
//	glBindTexture( GL_TEXTURE_2D, texture );
//
//	// draw geometry
//	drawGeometry(walls);
//	drawGeometry(rotatingMesh);
//
//    // draw skybox
//    drawSkyBox();
//
//	glDisable(GL_TEXTURE_2D);
//
//	// draw sound objects
//	int object;
//	if (TaskMode == 3)
//		{object=6;}
//	else
//		{object=0;}
//	//for (object = 0; object < NUM_OBJECTS-6; object++)
//	{
//        char txt[256];
//
//
//		float directOcclusion = 1.0f;
//		float reverbOcclusion = 1.0f;
//
//		// set colour baced on direct occlusion
////		objects[object].channel->getOcclusion(&directOcclusion, &reverbOcclusion);
//		float intensity = 1.0f;// - directOcclusion;
//
//		glPolygonMode(GL_FRONT_AND_BACK, rendermode);
//		glPushMatrix();
//		//glTranslatef(objects[object].xPos, objects[object].yPos, objects[object].zPos);
//		glTranslatef(0, objects[object].yPos, 0);
//
//        sprintf(txt, "Sound object (%d): %.2f, %.2f, %.2f", object, objects[object].xPos, objects[object].yPos, objects[object].zPos);
//        #ifdef SHOW_GUI_DEBUG_TEXT
//        debugText.push(std::string(txt));
//        #endif
//
//		glPushAttrib(GL_LIGHTING_BIT);
//
//		intensity *= 0.75f;
//		float color[4] = { intensity, intensity, 0.0f, 0.0f };
//		if(object == 0)
//		{//objekt broj 0 je ofarban crveno
//			color[1] = 0;
//			//std::cout<<objects[0].zPos<<"  "<<objects[0].xPos<<std::endl;
//			//std::cout<<std::endl;
//		}
//		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
//		intensity *= 0.5f;
//		float ambient[4] = { intensity, intensity, 0.0f, 0.0f };
//		if(object == 0)
//		{//objekt broj 0 je ofarban crveno
//			ambient[1] = 0;
//		}
//
//		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
//
//
//		glRotatef(accumulatedTime * 200.0f, 0.0f, 1.0f, 0.0f);
//        {
//            FMOD_VECTOR soundorientation;
//            float rad = (accumulatedTime * 200.0f);
//
//            rad *= 3.14159f;
//            rad /= 180.0f;
//
//            soundorientation.x = 0;//sinf(rad);
//            soundorientation.y = 0;
//            soundorientation.z = 0;//cosf(rad);
//
//            objects[object].event->set3DAttributes(0, 0, &soundorientation);
//        }
//
//		glutSolidTeapot(1.f);//SolidTorus(0.15f, 0.6f, 8, 16);
//		glPopAttrib();
//		glPopMatrix();
//	}
//
//    /*
//        Draw blue transparent blue quads to entry to water room
//    */
//    drawWaterRoom();
//
//    /*
//        Do water effects if we are in the water room
//    */
//    inWater();
//
//
//    /*
//        2D RENDERING
//    */
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluOrtho2D(0.0f, (GLsizei)width, 0.0f, (GLsizei)height);
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//
//	glDisable(GL_LIGHTING);
//	glDisable(GL_NORMALIZE);
//    glDisable(GL_DEPTH_TEST);
//
//    /*
//        Render text
//    */
//    renderUiText();
//
//
//    glEnable(GL_DEPTH_TEST);
//	glEnable(GL_LIGHTING);
//	glEnable(GL_LIGHT0);
//	glEnable(GL_NORMALIZE);
//	glShadeModel(GL_SMOOTH);
//	glPolygonMode(GL_FRONT_AND_BACK, rendermode);
//
//	// finish
//	glutSwapBuffers();
//}
//
//void reshapeFunc(int w, int h)
//{
//	width = w;
//	height = h;
//	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
//}
//
//void mouseFunc(int button, int state, int x, int y)
//{
//	switch (button)
//	{
//	case GLUT_LEFT_BUTTON:
//		if (state == GLUT_DOWN)
//		{
//			doRotate = true;
//			xMouse = x;
//			yMouse = y;
//		}
//		else
//		if (state == GLUT_UP)
//		{
//			doRotate = false;
//		}
//		break;
//
//	default:
//		break;
//	}
//}
//
//void motionFunc(int x, int y)
//{
//	//TONI s misom se moze rotirati a ne kretati naprijed-nazad
//	std::string SingleLog;
//	int test = 0;
//
//	int dx = x - xMouse;
//	int dy = y - yMouse;
//
//	// view rotation about y-axis
//	yRotation += (float)dx * 0.5f;
//	if (yRotation > 180.0f)
//		yRotation -= 360.0f;
//	else
//	if (yRotation < -180.0f)
//		yRotation += 360.0f;
//
//	// view rotation about x-axis
//	const float xExtent = 88.0f;
//	//TONI izmjena u redu ispod, da se ne mice gore-dolje vec samo lijevo-desno
//	xRotation += 0.0f;//(float)dy * 0.5f;
//	if (xRotation > xExtent)
//		xRotation = xExtent;
//	else
//	if (xRotation < -xExtent)
//		xRotation = -xExtent;
//
//	xMouse = x;
//	yMouse = y;
//}
//
//void keyboardFunc(unsigned char key, int x, int y)
//{
//	key1 = key;
//	switch (key)
//	{
//	case 'w':
//		moveForward = true;
//		break;
//	case 's':
//		moveBackward = true;
//		break;
//	case 'j':
//		moveRotateAntiClock = true;
//		break;
//	case 'l':
//		moveRotateClock = true;
//		break;
//	case 'a':
//		moveLeft = true;
//		break;
//	case 'd':
//		moveRight = true;
//		break;
//	case ' ':
//		moveUp = true;
//		break;
//	case 'c':
//		moveDown = true;
//        break;
//    case 'z' :
//        ambientVolDown = true;
//        break;
//    case 'Z' :
//        ambientVolUp = true;
//        break;
//    case 'v':
//        masterVolDown = true;
//        break;
//    case 'V':
//        masterVolUp = true;
//        break;
//
//    case 'p':
//        {
//            float value;
//
//            fmodEventParameter->getValue(&value);
//
//            fmodEventParameter->setValue(value - 20.0f);
//        }
//        break;
//
//    case 'P':
//        {
//            float value;
//
//            fmodEventParameter->getValue(&value);
//
//            fmodEventParameter->setValue(value + 20.0f);
//        }
//        break;
//
//    case 'f':
//        moveFast = true;
//        break;
//	}
//}
//
//void keyboardUpFunc(unsigned char key, int x, int y)
//{
//	key1 = 0;
//	switch (key)
//	{
//	case 'w':
//		moveForward = false;
//		break;
//	case 's':
//		moveBackward = false;
//		break;
//	case 'j':
//		moveRotateAntiClock = false;
//		break;
//	case 'l':
//		moveRotateClock = false;
//	case 'a':
//		moveLeft = false;
//		break;
//	case 'd':
//		moveRight = false;
//        break;
//	case ' ':
//		moveUp = false;
//		break;
//	case 'c':
//		moveDown = false;
//        break;
//    case 'z' :
//        ambientVolDown = false;
//        break;
//    case 'Z' :
//        ambientVolUp = false;
//        break;
//    case 'v':
//        masterVolDown = false;
//        break;
//    case 'V':
//        masterVolUp = false;
//        break;
//    case 'f':
//        moveFast = false;
//        break;
//    case 27:
//        exit(1);
//        break;
//	}
//}
//
void specialKeyFunc(int key, int x, int y)
{
  /*  switch (key)
    {

    }*/

    //key = key;
}
//
//void specialKeyUpFunc(int key, int x, int y)
//{
//    switch (key)
//    {
//    case GLUT_KEY_F1:
//        showhelp = !showhelp;
//        break;
//    case GLUT_KEY_F2:
//        if(fullscreen)
//        {
//            glutPositionWindow(20,40);
//            glutReshapeWindow(500,500);
//            fullscreen = false;
//        }
//        else
//        {
//            glutFullScreen();
//            fullscreen = true;
//        }
//        break;
//    case GLUT_KEY_F3:
//        rendermode = (rendermode == GL_LINE ? GL_FILL : GL_LINE);
//        break;
//    case GLUT_KEY_F11:
//        showdebug = !showdebug;
//        break;
//    case GLUT_KEY_F8:
//        rotatingMesh.geometry->setActive(false);
//        walls.geometry->setActive(false);
//        break;
//    case GLUT_KEY_F9:
//        rotatingMesh.geometry->setActive(true);
//        walls.geometry->setActive(true);
//        break;
//    }
//}
//
//void timerFunc(int nValue)
//{
//    std::string TimeLog;
//	static bool firsttime = true;
//	FMOD_RESULT result;
//
//	//doGeometryMovement();
//
//	doSoundMovement();
//	doListenerMovement();
//    doUpdateVolume();
//
//    result = FMOD::NetEventSystem_Update();
//	ERRCHECK(result);
//    result = fmodEventSystem->update();
//	ERRCHECK(result);
//
//	accumulatedTime += (float)INTERFACE_UPDATETIME / 1000.0f;
//
//	glutPostRedisplay();
//	glutTimerFunc(INTERFACE_UPDATETIME, timerFunc, 0);
//
//#if 0
//    if (firsttime)
//    {
//        SetWindowPos(
//          GetForegroundWindow(),    // handle to window
//          HWND_TOPMOST,             // placement-order handle
//          0,                        // horizontal position
//          0,                        // vertical position
//          width,                    // width
//          height,                   // height
//          SWP_NOMOVE
//        );
//        firsttime = false;
//    }
//#endif
//}
//
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
//
//void initGeometry(const char* szFileName, Mesh& mesh, bool alter)
//{
//	FMOD_RESULT result;
//
//	FILE* file = fopen(szFileName, "rb");
//	if (file)
//	{
//		// read vertices
//		fread(&mesh.numVertices, sizeof (mesh.numVertices), 1, file);
//		mesh.vertices = new FMOD_VECTOR[mesh.numVertices];
//		mesh.texcoords = new float[mesh.numVertices][2];
//		fread(mesh.vertices, sizeof (float) * 3, mesh.numVertices, file);
//		fread(mesh.texcoords, sizeof (float) * 2, mesh.numVertices, file);
//
//
//		fread(&mesh.numIndices, sizeof (mesh.numIndices), 1, file);
//		mesh.indices = new int[mesh.numIndices];
//		fread(mesh.indices, sizeof (int), mesh.numIndices, file);
//
//
//		fread(&mesh.numPolygons, sizeof (mesh.numPolygons), 1, file);
//		mesh.polygons = new Polygon[mesh.numPolygons];
//
//		// read polygons
//		for (int poly = 0; poly < mesh.numPolygons; poly++)
//		{
//			Polygon* polygon = &mesh.polygons[poly];
//
//
//			fread(&polygon->numVertices, sizeof (polygon->numVertices), 1, file);
//			fread(&polygon->indicesOffset, sizeof (polygon->indicesOffset), 1, file);
//			fread(&polygon->directOcclusion, sizeof (polygon->directOcclusion), 1, file);
//			fread(&polygon->reverbOcclusion, sizeof (polygon->reverbOcclusion), 1, file);
//
//			int* indices = &mesh.indices[polygon->indicesOffset];
//
//			// calculate polygon normal
//			float xN = 0.0f;
//			float yN = 0.0f;
//			float zN = 0.0f;
//			// todo: return an error if a polygon has less then 3 vertices.
//			for (int vertex = 0; vertex < polygon->numVertices - 2; vertex++)
//			{
//				float xA = mesh.vertices[indices[vertex + 1]].x -mesh.vertices[indices[0]].x;
//				float yA = mesh.vertices[indices[vertex + 1]].y -mesh.vertices[indices[0]].y;
//				float zA = mesh.vertices[indices[vertex + 1]].z -mesh.vertices[indices[0]].z;
//				float xB = mesh.vertices[indices[vertex + 2]].x -mesh.vertices[indices[0]].x;
//				float yB = mesh.vertices[indices[vertex + 2]].y -mesh.vertices[indices[0]].y;
//				float zB = mesh.vertices[indices[vertex + 2]].z -mesh.vertices[indices[0]].z;
//				// cross product
//				xN += yA * zB - zA * yB;
//				yN += zA * xB - xA * zB;
//				zN += xA * yB - yA * xB;
//			}
//			float fMagnidued = (float)sqrt(xN * xN + yN * yN + zN * zN);
//			if (fMagnidued > 0.0f) // a tollerance here might be called for
//			{
//				xN /= fMagnidued;
//				yN /= fMagnidued;
//				zN /= fMagnidued;
//			}
//			polygon->normal.x = xN;
//			polygon->normal.y = yN;
//			polygon->normal.z = zN;
//		}
//		fclose(file);
//	}
//
//	result = fmodSystem->createGeometry(mesh.numPolygons, mesh.numIndices, &mesh.geometry);
//	ERRCHECK(result);
//
//    /*
//        Tell FMOD about the geometry
//    */
//	for (int poly = 0; poly < mesh.numPolygons; poly++)
//	{
//		Polygon* polygon = &mesh.polygons[poly];
//		FMOD_VECTOR vertices[16];
//		for (int i = 0; i < polygon->numVertices; i++)
//			vertices[i] = mesh.vertices[mesh.indices[polygon->indicesOffset + i]];
//		int polygonIndex = 0;
//
//        if (alter && polygon->directOcclusion == 0.85f)
//        {
//     //       polygon->directOcclusion = 0.95f;
//        }
//
//		result = mesh.geometry->addPolygon(
//			polygon->directOcclusion,
//			polygon->reverbOcclusion,
//			false, // single sided
//			polygon->numVertices,
//			vertices,
//			&polygonIndex);
//		ERRCHECK(result);
//	}
//}
//
//void ERRCHECK(FMOD_RESULT result)
//{
//    if (result != FMOD_OK)
//    {
//        printf("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
//        exit(-1);
//    }
//}
//
