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

#define DELETE_OBJECT(x)	if ((x)) { delete (x); (x) = NULL; }   // delete a class object
#define DELETE_ARRAY(x)		if ((x)) { delete [] (x); (x) = NULL; } // delete an array
#define FOR(i,n) for( int i=0; i<n; i++ )                           // for loop 
#define FOR_u(i, n) for (size_t i = 0; i < n; i++)                  // for loop 
#define SQUARE(x) ((x)*(x))
#define INF 1e10f
#define PI 3.1415926f
inline float sqr(float x) { return x*x; }
typedef unsigned char uchar; 
#define PRINT

using namespace std;
using namespace Eigen;
 
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
		FOR (i, 4) { 
			FOR (j, 4)
				printf("(%f, %f, %f) ", m_point[i][j](0), m_point[i][j](1), m_point[i][j](2)); // << ";\t";
			cout << endl;
      } 
  }

public:
  Vector3f m_point[4][4];  
};

vector<CPatch> g_patches; 

//////////////////////////////////////////////////////////////////
//parseBez function
//////////////////////////////////////////////////////////////////


vector<CPatch> parseBez(string file){
    vector<CPatch> patches;

    ifstream fin(file.c_str());

    if(!fin.is_open()){
	  printf("failed to open file (%s)\n", file.c_str()); 
      exit(-1);
    }
    else{
      vector<string> splitline;
      string line, buf;

      getline(fin, line);
      int num_patch = atoi(line.c_str());
	  printf("read (%d) patches from (%s)\n", num_patch, file.c_str());
	  patches.resize(num_patch);

        for(int k = 0; k < num_patch; k++){
          //patches[k] = CPatch();
		  CPatch patch; 
          for(int i=0; i< 4; i++){
            getline(fin, line);
            stringstream ss(line);
            // cout << k << "\t" << line << endl;

            while (ss >> buf){
              splitline.push_back(buf);
            }

            for(int j=0; j < 4; j++){
                patch.assign(i, j,  Vector3f(
                  atof(splitline[0+j*3].c_str()), 
                  atof(splitline[1+j*3].c_str()), 
                  atof(splitline[2+j*3].c_str())) );
            }

            splitline.clear();
          }
           getline(fin, line);
		   patches[k] = patch; 
        }

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
Viewport viewport;

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
  glClear(GL_COLOR_BUFFER_BIT);                // clear the color buffer (sets everything to black)

  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                            // make sure transformation is "zero'd"

  //----------------------- code to draw objects --------------------------
  // Rectangle Code
  //glColor3f(red component, green component, blue component);

  //glColor3f(R,G,B);                   // setting the color to pure red 90% for the rect

  //CPatch * patches = parseBez("teapot.bez");

  glBegin(GL_LINE_STRIP);


//for (int k=0; k < num_patch ; k++)
//   {
//    for(int i=0; i < 4; i++){
//      for(int j =0; j< 4; j++){
//          glVertex3f(patches[k].m_point[j][i](0),
//              patches[k].m_point[j][i](1), 
//              patches[k].m_point[j][i](2));
//      }
//    }
//   }
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
  g_patches = parseBez("test.bez");



  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  // Initialize the viewport size
  viewport.w = 400;
  viewport.h = 400;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Tesselation");

  initScene();                                 // quick function to set up scene

  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  // glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  
  g_patches.clear(); 
  return 0;
}








