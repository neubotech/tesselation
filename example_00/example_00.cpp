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

using namespace std;
using namespace Eigen;

#define DELETE_OBJECT(x)	if ((x)) { delete (x); (x) = NULL; }   // delete a class object
#define DELETE_ARRAY(x)		if ((x)) { delete [] (x); (x) = NULL; } // delete an array
#define FOR(i,n) for( int i=0; i<n; i++ )                           // for loop 
#define FOR_u(i, n) for (size_t i = 0; i < n; i++)                  // for loop 
#define SQUARE(x) ((x)*(x))
#define INF 1e10f
#define EPS 1e-7f
#define PI 3.1415926f
inline float sqr(float x) { return x*x; }
typedef unsigned char uchar; 
typedef Vector3f V3f; 
#define PRINT

enum Ttype { uniform, adaptive};
Ttype type = uniform;
 
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

  V3f Point(int _idx, int _idy) const { return m_point[_idx][_idy]; }
public:
  Vector3f m_point[4][4];  
};

class CCurve {
public: 
	CCurve() {
		m_points.clear();
	}

	~CCurve() {
		m_points.clear();
	}
	
	CCurve(vector<V3f>& _points) {
		if (_points.size() == 4) 
			m_points = _points; 
		else {
			printf("#points = %d\n", _points.size());
			exit(-2);
		}
	}

	void GetPoint(const V3f& _p) { m_points.push_back(_p);}
	V3f P0() const { return m_points[0];}
	V3f P1() const { return m_points[1];}
	V3f P2() const { return m_points[2]; }
	V3f P3() const { return m_points[3]; }

private: 
	vector<V3f> m_points; 
};

class CLocalGeo {
public:
	CLocalGeo(){};

	CLocalGeo(V3f _P){
		m_P = _P;
		m_n = Vector3f(0,0,0);
		m_uv = Vector2f(-1,-1);
	}

	CLocalGeo(const CPatch& _patch, V3f _P){
		m_P = _P;

		V3f U = _patch.m_point[3][0] - _patch.m_point[0][0];
		V3f V = _patch.m_point[0][3] - _patch.m_point[0][0];
		V3f P = m_P - _patch.m_point[0][0];

		U = U/U.norm();
		V = V/V.norm();
		P = P/P.norm();

		m_uv(0) = P.dot(U);
		m_uv(1) = P.dot(V);
		
		m_n = Vector3f(0, 0, 0);
	}
public:
	void print(){
		printf("position:\t%d\t%d\t%d\n", m_P.x(), m_P.y(), m_P.z());
		printf("Normal:\t%d\t%d\t%d\n", m_n.x(), m_n.y(), m_n.z());
		printf("u, v :\t%d\t%d\n", m_uv(0), m_uv(1));
	}

public: 
	V3f m_P; 
	V3f m_n; 
	Vector2f m_uv;


};

/////////////////////////////////////////////////////////////////////////////////
//adaptive code
/////////////////////////////////////////////////////////////////////
class CTriangle{
 	CTriangle( CPatch* _patch, V3f _v1, V3f _v2, V3f _v3 ):
 	m_v1(*_patch, _v1), m_v2(*_patch, _v2), m_v3(*_patch, _v3)
 	{
 		m_patch = _patch;
 	}

public:
	CLocalGeo m_v1, m_v2, m_v3;
	CPatch* m_patch;
};

typedef vector<vector<CLocalGeo> > PatchMesh; 

typedef vector<CLocalGeo> PatchVertex;

class CBezier {
public: 
	void UniformTessellate(const CPatch& _patch, float _step, vector<vector<CLocalGeo> >& _geos) {
		int numdiv = (1+EPS)/_step+1; 
		_geos.clear(); 
		_geos.resize(numdiv);

		FOR (i, numdiv)
			_geos[i] = vector<CLocalGeo>(numdiv);


		FOR (iu, numdiv) {
			float u = iu * _step; 
			FOR (iv, numdiv) {
				float v = iv * _step; 
				//V3f P, n; 
				CLocalGeo geo; 
				BezPatchInterp(_patch, u, v, geo); 
				_geos[iu][iv] = geo; 

				// if(iv==0 || iv ==1){
				// 	geo.print();
				// }
			}
		}
	}
	///////////////////////////////////////////////////////////
	//adaptive code
	///////////////////////////////////////////////////////
	void AdaptiveTriangulation(const CPatch& _patch, float _error, vector<CLocalGeo> & _geos){
		//initialize
		CLocalGeo v[2][2];

		for (int i = 0; i < 2; i++){
			for (int j=0; j< 2; j++){

				BezPatchInterp(_patch, i, j, v[i][j]);
				// v[i][j].print();
				// cout << i << j <<endl;
				// v[i][j].m_uv = Vector2f(i, j);
				_geos.push_back(v[i][j]);
				
			}
		}


	}

	////////////////////////////////////////////////////////

private: 
	void BezCurveInterp(const CCurve& _curve, float _u, CLocalGeo& _geo) {
		float _u1 = 1.0f - _u; 
		V3f A = _curve.P0() * _u1 + _curve.P1() * _u; 
		V3f B = _curve.P1() * _u1 + _curve.P2() * _u; 
		V3f C = _curve.P2() * _u1 + _curve.P3() * _u; 
		V3f D = A * _u1 + B * _u; 
		V3f E = B * _u1 + C * _u; 
		_geo.m_P = D * _u1 + E * _u; 
		_geo.m_n = 3 * (E - D);
	}

	void BezPatchInterp(const CPatch& _patch, float _u, float _v, CLocalGeo& _geo) {
		//vector<V3f> vpnts(4);
		CCurve vcurve, ucurve; 
		FOR (i, 4) {
			CCurve vTmp, uTmp; 
			FOR (j, 4)
				vTmp.GetPoint(_patch.Point(i, j));

			CLocalGeo vGeo, uGeo; //tmpdPdu; 
			BezCurveInterp(vTmp, _u, vGeo);
			vcurve.GetPoint(vGeo.m_P);

			FOR (j, 4)
				uTmp.GetPoint(_patch.Point(j, i));

			BezCurveInterp(uTmp, _v, uGeo);
			ucurve.GetPoint(uGeo.m_P);
		}

		//V3f dPdv, dPdu; 
		CLocalGeo Geo1, Geo2; 
		BezCurveInterp(vcurve, _v, Geo1);
		BezCurveInterp(ucurve, _u, Geo2);
		_geo.m_P = Geo1.m_P; 
		V3f _n = Geo2.m_n.cross(Geo1.m_n);
		_geo.m_n = _n / _n.norm(); 
	}
};


vector<CPatch> g_patches; 
vector<PatchMesh> g_meshes; 

vector<PatchVertex> g_triangles;



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
  //gluPerspective(45.0f, 1.0f, 1.0f, 500.0f);
  //glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );	
  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  //glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}


//****************************************************
// sets the window up
//****************************************************
void initScene(){
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent
  glMatrixMode(GL_MODELVIEW);													
  
  // specify the clear value for the depth buffer
  //glClearDepth( 1.0f );														
  gluLookAt(0, 0, 5, 0,0,0, 0,1,0);
  GLfloat amb_light[] = { 0, 0, 0.5, 1.0 };
  GLfloat diffuse[] = { 0, 0, 0.6, 1 };
  GLfloat specular[] = { 0, 0, 0.3, 1 };
  GLfloat lightpos[] = {1., 1., 1., 0.};
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, amb_light );
  glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuse );
  glLightfv( GL_LIGHT0, GL_SPECULAR, specular );
  glEnable( GL_LIGHT0 );
  glEnable( GL_COLOR_MATERIAL );
  //glShadeModel( GL_FLAT );
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
  glEnable( GL_DEPTH_TEST );
  glDepthFunc( GL_LEQUAL );
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0); 
  myReshape(viewport.w,viewport.h);
}

float g_transX = 0.0f; 
float g_transY = 0.0f; 
float g_scale = 1.0f; 
float g_rotateY = 0.0f; 
float g_rotateX = 0.0f;
bool g_smooth = true; 
bool g_filled = true; 

void keyPressed (unsigned char key, int x, int y) {  
	//printf("press key (%c)\n", key);
	switch (key) {
	case '=':
		g_scale *= 1.1;
		break; 
	case '-':
		g_scale /= 1.1; 
		break;
	case 's': 
		g_smooth = !g_smooth; 
		if (g_smooth) 
			printf("smooth shading\n");
		else 
			printf("flat shading\n");
		break;
	case 'w':
		g_filled = !g_filled;
		if (g_filled)
			printf("filled mode\n");
		else
			printf("wireframe mode\n");
		break;
	case 'q':
		printf("quit the program\n");
		exit(0);
		break; 
	default:
		printf("wrong keyboard input\n");
		break; 
	}
	//if (key == '+')	
} 

void arrowKeyPressed(int key, int x, int y) {
	int mod = glutGetModifiers();
	switch (key) {
	case GLUT_KEY_UP:
		if (mod == GLUT_ACTIVE_SHIFT)
			g_transY += 0.01f; 
		else
			g_rotateY += 5.0f; 
		break; 
	case GLUT_KEY_DOWN:
		if (mod == GLUT_ACTIVE_SHIFT)
			g_transY -= 0.01f;
		else
			g_rotateY -= 5.0f;
		break; 
	case GLUT_KEY_LEFT:
		if (mod == GLUT_ACTIVE_SHIFT)
			g_transX += 0.01f;
		else
			g_rotateX += 5.0f; 
		break; 
	case GLUT_KEY_RIGHT:
		if (mod == GLUT_ACTIVE_SHIFT)
			g_transX -= 0.01f;
		else
			g_rotateX -= 5.0f;
		break; 
	default:
		break;
	}
		
}


//***************************************************
// function that does the actual drawing
//***************************************************
float g_dx = 0.0f; 
void myDisplay() {
  //glClear(GL_COLOR_BUFFER_BIT);                // clear the color buffer (sets everything to black)
	//printf("my Display\n");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
	glLoadIdentity();                            // make sure transformation is "zero'd"
  

  //gluLookAt( 4,2,0, 0,0,0, 0,1,0);

	glPushMatrix();		
	
	glTranslatef(g_transX, g_transY, 0.0f);
	glRotatef(g_rotateY, 1.0f, 0.0f, 0.0f);
	glRotatef(g_rotateX, 0.0f, 1.0f, 0.0f);
	glScalef(g_scale, g_scale, g_scale);
  //----------------------- code to draw objects --------------------------

	if (g_smooth)
		glShadeModel( GL_SMOOTH );
	else
		glShadeModel(GL_FLAT);

	if (g_filled) 
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	else
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );



	int dx[] = {0, 0, 1, 1};
	int dy[] = {0, 1, 1, 0};
	glColor3f(0.0f, 0.0f, 1.0f);

	switch (type){
		case uniform:
			FOR_u (k, g_meshes.size()) {
				PatchMesh mesh = g_meshes[k];
				FOR (i, mesh.size()-1) {
					FOR (j, mesh[i].size()-1) {
						glBegin(GL_QUADS);
						FOR (l, 4) {
							int w = i + dx[l];
							int h = j + dy[l];
							V3f P = mesh[w][h].m_P;
							V3f n = mesh[w][h].m_n;
							glNormal3f(n.x(), n.y(), n.z());
							glVertex3f(P.x(),P.y(),P.z());
						}

						glEnd();	
					}
				}
			}
		case adaptive:
			// cout<< type <<endl;
			for(int k =0; k < g_triangles.size(); k++){
				// cout<<k << endl;
				PatchVertex vertexes = g_triangles[k];
				for (int s = 0; s < vertexes.size(); s++){
					V3f P = vertexes[s].m_P;
					V3f n = vertexes[s].m_n;
					glNormal3f(n.x(), n.y(), n.z());
					glVertex3f(P.x(),P.y(),P.z());
				}
			}

		}

		glPopMatrix();		
		glutSwapBuffers();
	                           // swap buffers (we earlier set double buffer)
	
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



void ProcessGeometry(const vector<CPatch>& _patches, Ttype type, const float parameter) {
	CBezier* bezier = new CBezier(); 
	
	switch (type) {
		case uniform:
			FOR (k, (int)_patches.size()) {
				PatchMesh mesh; 
				bezier->UniformTessellate(_patches[k], parameter, mesh);
				g_meshes.push_back(mesh);
			}
		case adaptive:
			FOR (k, (int)_patches.size()) {
				cout<<k<<endl;
				PatchVertex vertexes; 
				bezier->AdaptiveTriangulation(_patches[k], parameter, vertexes);
				cout << vertexes.size()<<endl;
				vertexes[0].print();
				g_triangles.push_back(vertexes);
			}
	}

	DELETE_OBJECT(bezier);
}



//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  float parameter = 0.1f;
  string filename = "teapot.bez";

  if (argc > 1){
  	filename = argv[1];
  	cout << "opening:\t" << argv[1] << endl;
  }
  if (argc > 2){
  	parameter = atof(argv[2]);

  	cout << "parameter:\t"<<parameter << endl;
  }

  if (argc > 3){
  	if (strcmp(argv[3], "-a") == 0){
  		type = adaptive;
  		cout<< "type:\tAdaptive Triangulation"<<endl;
  	}
  	else{
  		cout<< "type:\tUniform Tessellation"<<endl;
  	}
  }

  


  g_patches = parseBez(filename);

  ProcessGeometry(g_patches, type, parameter);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  // Initialize the viewport size
  viewport.w = 800;
  viewport.h = 800;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Tessellation/Triangulation");
 
  initScene();                                 // quick function to set up scene
  glutKeyboardFunc(keyPressed);
  glutSpecialFunc(arrowKeyPressed);
  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  
  g_patches.clear(); 
  //DELETE_OBJECT(bezier);
  return 0;
}