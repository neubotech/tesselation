// CS184 Simple OpenGL Example
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
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
//#undef Success
#ifdef  _WIN32
#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/Dense"
#else
#undef Success
#include <Eigen/Core>
#include <Eigen/Dense>
#endif //  _WIN32


#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265

using namespace std;
using namespace Eigen;

int num_patch;

//////////////////////////////////////////////////////////////////
//CPatch
//////////////////////////////////////////////////////////////////

class CPatch{
public:
  CPatch(){}

public:
  bool assign(int idx, int idy, Vector3f point){
    m_point[idx][idy] = point;
    return true;
  }

  void print(){
    for (int i = 0; i < 4; i++){

      for (int j=0; j < 4; j++){
        cout << m_point[j][i](0) <<",\t"<< m_point[j][i](1)<< ",\t"<< m_point[j][i](2) << ";\t";
      } 
      cout << endl<<endl;
    }
    
  }

public:
  // Mat4f P;
  Vector3f m_point[4][4];  
};



//////////////////////////////////////////////////////////////////
//parseBez function
//////////////////////////////////////////////////////////////////


CPatch*  parseBez(string file){
    printf("parse scene %s\n", file.c_str());
    CPatch * patches;

    ifstream fin(file.c_str());

    if(!fin.is_open()){
      cout <<"open file"<<file<<" failed"<<endl;
      exit(-1);
    }
    else{
      vector<string> splitline;
      string line, buf;

      getline(fin, line);
      num_patch = atoi(line.c_str());
      // cout << line <<endl;
      patches = (CPatch *)malloc(num_patch * sizeof(CPatch));
      
      // while(fin.good()){

        for(int k = 0; k < num_patch; k++){

          patches[k] = CPatch();

          for(int i=0; i< 4; i++){

            getline(fin, line);
            stringstream ss(line);

            // cout << k << "\t" << line << endl;

            while (ss >> buf){
              splitline.push_back(buf);
            }

            for(int j=0; j < 4; j++){
                patches[k].assign(j, i,  Vector3f(
                  atof(splitline[0+j*3].c_str()), 
                  atof(splitline[1+j*3].c_str()), 
                  atof(splitline[2 + j*3].c_str())) );
            }

            splitline.clear();
          }
           getline(fin, line);
          // patches[k] = CPatch(fin);


        }

      // }
      fin.close();
    }
    return patches;
  }


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

  CPatch * patches = parseBez("teapot.bez");

  glBegin(GL_LINE_STRIP);


for (int k=0; k < num_patch ; k++)
   {
    for(int i=0; i < 4; i++){
      for(int j =0; j< 4; j++){
          glVertex3f(patches[k].m_point[j][i](0),
              patches[k].m_point[j][i](1), 
              patches[k].m_point[j][i](2));
      }
    }
   }
  glEnd();

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

  // CPatch * patches = parseBez("teapot.bez");
  // CPatch * patches = parseBez("test.bez");
  // patches[0].print();



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
  // glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else

  return 0;
}








