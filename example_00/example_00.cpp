// CS184 Simple OpenGL Example
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265

using namespace std;

//****************************************************
// Some Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport    viewport;

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}


//****************************************************
// sets the window up
//****************************************************
void initScene(){
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent

  myReshape(viewport.w,viewport.h);
}


//***************************************************
// function that does the actual drawing
//***************************************************
void myDisplay() {


  //----------------------- ----------------------- -----------------------
  // This is a quick hack to add a little bit of animation.
  static float tip = 0.0f;
  const  float stp = 0.05f;
  static  float R = 0.0f;
  static  float G = 0.0f;
  static  float B = 0.0f;
  const  float c_beg = 1.0f;
  const  float c_stp = 0.003f;
  const  float beg = 0.0f;
  const  float end = float(2*PI);
  const float c_end = 0.0f;
  const float DEG2RAD=PI/180;
  tip += stp;
  if (tip>end) tip = beg;
if ( R > c_end) {
	R-=c_stp;
}
else if ( B > c_end) {
	B-=c_stp;
}
else if ( G > c_end) {
	G-=c_stp;
}
else {
	R= c_beg, G = c_beg, B = c_beg;
} 
  //----------------------- ----------------------- -----------------------


  glClear(GL_COLOR_BUFFER_BIT);                // clear the color buffer (sets everything to black)

  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                            // make sure transformation is "zero'd"

  //----------------------- code to draw objects --------------------------
  // Rectangle Code
  //glColor3f(red component, green component, blue component);

  glColor3f(R,G,B);                   // setting the color to pure red 90% for the rect
  glBegin(GL_LINE_STRIP);
float k = 30;
for (float j=0; j < 360*12 ; j++)
   {
      float degInRad = j*DEG2RAD;
      float x =degInRad/k * cosf(degInRad-tip);
      float y =degInRad/k * sinf(degInRad-tip);
      glVertex3f(x,y, 0.0f);
   }
  glEnd();

static float s=0;
const float s_end = 360*10;
const float s_beg = 0;
const float s_stp = 10;
if (s < s_end) {
      s+=s_stp;
      float degInRad = s*DEG2RAD;
      float x =degInRad/k * cosf(degInRad-tip);
      float y =degInRad/k * sinf(degInRad-tip);
  glColor3f(1.0f, 0.0f, 0.0f);

  glBegin(GL_POLYGON);
	for (float m = 0; m < 360; m ++){
		float theta = m * DEG2RAD;
		glVertex3f(2/k*cos(theta)+x, 2/k*sin(theta)+y, 0.0f);
	}

  glEnd();
}
else s = s_beg; 
  glFlush();
  glutSwapBuffers();                           // swap buffers (we earlier set double buffer)
}


//****************************************************
// called by glut when there are no messages to handle
//****************************************************
void myFrameMove() {
  //nothing here for now
#ifdef _WIN32
  Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif
  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  // Initalize theviewport size
  viewport.w = 400;
  viewport.h = 400;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("CS184! cs184-at Nan Tian");

  initScene();                                 // quick function to set up scene

  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else

  return 0;
}








