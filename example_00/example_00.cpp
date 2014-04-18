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
#include "Eigen/Geometry"
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
// #define N 1000  //Number of triangles per half patch


inline float sqr(float x) { return x*x; }
typedef unsigned char uchar; 
typedef Vector3f V3f; 
#define PRINT

enum Ttype { uniform, adaptive};
Ttype type = uniform;

int n_triangle=0;
 
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

  void print() const{
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

	CLocalGeo(const CPatch& _patch, V3f _P, float _u, float _v){
		m_P = _P;
		m_n = Vector3f(0, 0, 0);
		m_u = _u;
		m_v = _v;

	}

	CLocalGeo(const CPatch& _patch, V3f _P){
		m_P = _P;
		m_n = Vector3f(0, 0, 0);
		m_u = -1;
		m_v = -1;

	}
public:
	void assignUV(float _u, float _v){
		m_u = _u;
		m_v = _v;
	}

	void print(){
		printf("pos:\t%f\t%f\t%f\n", m_P(0), m_P(1), m_P(2));
		printf("Normal:\t%f\t%f\t%f\n", m_n.x(), m_n.y(), m_n.z());
		printf("u, v :\t%f\t%f\n", m_u, m_v);
	}

public: 
	V3f m_P; 
	V3f m_n; 
	float m_u;
	float m_v; 

};


/////////////////////////////////////////////////////////////////////////////////
//adaptive code
/////////////////////////////////////////////////////////////////////
class CTriangle{
public:
	// CTriangle(){};

 	CTriangle(const CPatch& _patch, CLocalGeo _v1, CLocalGeo _v2, CLocalGeo _v3):
 	m_v1(_patch, _v1.m_P, _v1.m_u, _v1.m_v), m_v2(_patch, _v2.m_P, _v2.m_u, _v2.m_v), m_v3(_patch, _v3.m_P, _v3.m_u, _v3.m_v)
 	{
 		// m_patch = &_patch;

 		n_triangle++;

 		cout<<"triangle#: "<< n_triangle << endl;
 		m_v1.print();
 		m_v2.print();
 		m_v3.print();
 	}

public:
	CLocalGeo m_v1, m_v2, m_v3;
	// const CPatch *m_patch;
};

typedef vector<vector<CLocalGeo> > PatchMesh; 

typedef vector<CTriangle> PatchTriangle;

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
	void AdaptiveTriangulation(const CPatch& _patch, float _error, vector<CTriangle> & _geos){
		//initialize 4 points of the patch
		_geos.clear(); 

		CLocalGeo v00(_patch, _patch.m_point[0][0], 0, 0);
		CLocalGeo v01(_patch, _patch.m_point[0][1], 0, 1);
		CLocalGeo v10(_patch, _patch.m_point[1][0], 1, 0);
		CLocalGeo v11(_patch, _patch.m_point[1][1], 1, 1);

		// v11.print();


		CTriangle bl( _patch, v00, v01, v10);

		

		CTriangle tr( _patch, v10, v01, v11); //bl = bottom left; tr = top right;

		subdivide(_patch, bl, _error, _geos);
		subdivide(_patch, tr, _error, _geos);


	}

private: 
	bool subdivide( const CPatch& _patch, CTriangle t, float _error, vector<CTriangle> & _geos){
		CLocalGeo v12(_patch, (t.m_v1.m_P + t.m_v2.m_P)/2.0f, (t.m_v1.m_u + t.m_v2.m_u)/2.0f, (t.m_v1.m_v + t.m_v2.m_v)/2.0f);
		CLocalGeo v23(_patch, (t.m_v2.m_P + t.m_v3.m_P)/2.0f, (t.m_v2.m_u + t.m_v3.m_u)/2.0f, (t.m_v2.m_v + t.m_v3.m_v)/2.0f);
		CLocalGeo v31(_patch, (t.m_v3.m_P + t.m_v1.m_P)/2.0f, (t.m_v3.m_u + t.m_v1.m_u)/2.0f, (t.m_v3.m_v + t.m_v1.m_v)/2.0f);

		bool e12=edge_test(_patch, t.m_v1, t.m_v2, v12, _error);
		bool e23=edge_test(_patch, t.m_v2, t.m_v3, v23, _error);
		bool e31=edge_test(_patch, t.m_v3, t.m_v1, v31, _error);

		t.m_v1.print();
		t.m_v2.print();
		v12.print();

		// cout<< e12 << e23 << e31 << endl;
		// _patch.print();

		if(e12 & e23 & e31){
			_geos.push_back(t);

			// t.m_v1.print();
			// t.m_v2.print();
			// t.m_v3.print();
		}
		else if (!e12 & e23 & e31){
			CTriangle t1(_patch, t.m_v1, v12, t.m_v3);
			CTriangle t2(_patch, v12, t.m_v2, t.m_v3);
			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
		}
		else if (e12 & !e23 & e31){
			CTriangle t1(_patch, t.m_v2, v23, t.m_v1);
			CTriangle t2(_patch, t.m_v3, t.m_v1, v23);
			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
		}
		else if (e12 & e23 & !e31){
			CTriangle t1(_patch, t.m_v1, t.m_v2, v31);
			CTriangle t2(_patch, t.m_v3, v31, t.m_v2);
			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
		}
		else if (!e12 & !e23 & e31){
			CTriangle t1(_patch, t.m_v1, v12, v23);
			CTriangle t2(_patch, t.m_v2, v23, v12);
			CTriangle t3(_patch, t.m_v3, t.m_v1, v23);

			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
			subdivide(_patch, t3, _error, _geos);
		}
		else if (e12 & !e23 & !e31){
			CTriangle t1(_patch, t.m_v2, v23, v31);
			CTriangle t2(_patch, t.m_v3, v31, v23);
			CTriangle t3(_patch, t.m_v1, t.m_v2, v31);

			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
			subdivide(_patch, t3, _error, _geos);
		}
		else if (!e12 & e23 & !e31){
			CTriangle t1(_patch, t.m_v3, v31, v12);
			CTriangle t2(_patch, t.m_v1, v12, v31);
			CTriangle t3(_patch, t.m_v2, t.m_v3, v12);

			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
			subdivide(_patch, t3, _error, _geos);
		}
		else if (!e12 & !e23 & !e31){
			CTriangle t1(_patch, t.m_v1, v12, v31);
			CTriangle t2(_patch, t.m_v2, v23, v12);
			CTriangle t3(_patch, t.m_v3, v31, v23);
			CTriangle t4(_patch, v12, v23, v31);

			subdivide(_patch, t1, _error, _geos);
			subdivide(_patch, t2, _error, _geos);
			subdivide(_patch, t3, _error, _geos);
			subdivide(_patch, t4, _error, _geos);
		}
	}

	//need to test out
	bool edge_test(const CPatch& _patch, CLocalGeo _v1, CLocalGeo _v2, CLocalGeo & _v12, float _error){
		CLocalGeo mid_point(_patch, (_v1.m_P+_v2.m_P)/2.0f);

		_v12.m_u = (_v1.m_u + _v2.m_u)/2.0f;
		_v12.m_v = (_v1.m_v + _v2.m_v)/2.0f;


		_v1.print();
		BezPatchInterp(_patch, _v12.m_u, _v12.m_v, _v12);
		// BezPatchInterp(_patch, mid_point.m_uv(0), mid_point.m_uv(1), _v12);



		float error=(_v12.m_P - mid_point.m_P).norm();
		// _v12.print();
		// mid_point.print();
		// cout << error << endl << endl;
		return error < _error;
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

vector<PatchTriangle> g_triangle_meshes;



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

if (type == uniform){
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
			// cout<< "type: " << type<< endl;
		}
if(type == adaptive){
			// cout<< type <<endl;
			for(int k =0; k < g_triangle_meshes.size(); k++){
				// cout<<k << endl;
				PatchTriangle Triangles = g_triangle_meshes[k];
				for (int s = 0; s < Triangles.size(); s++){
					glBegin(GL_TRIANGLES);
					V3f P, n;

					P = Triangles[s].m_v1.m_P;
					n = Triangles[s].m_v1.m_n;
					glNormal3f(n.x(), n.y(), n.z());
					glVertex3f(P.x(),P.y(),P.z());

					P = Triangles[s].m_v2.m_P;
					n = Triangles[s].m_v2.m_n;
					glNormal3f(n.x(), n.y(), n.z());
					glVertex3f(P.x(),P.y(),P.z());

					P = Triangles[s].m_v3.m_P;
					n = Triangles[s].m_v3.m_n;
					glNormal3f(n.x(), n.y(), n.z());
					glVertex3f(P.x(),P.y(),P.z());
					glEnd();
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


	
if (type == uniform){
			FOR (k, (int)_patches.size()) {
				PatchMesh mesh; 
				bezier->UniformTessellate(_patches[k], parameter, mesh);
				g_meshes.push_back(mesh);
			}
			cout<< "type: uniform " << type<< endl;
}
if (type == adaptive){
			FOR (k, (int)_patches.size()) {
				// cout<<k<<endl;
				PatchTriangle T; 
				bezier->AdaptiveTriangulation(_patches[k], parameter, T);
				// cout << vertexes.size()<<endl;
				// vertexes[0].print();
				if (true){
					g_triangle_meshes.push_back(T);
				}
				else{
					cout<<"reach maximum triangles per patch for patch "<< k <<endl;
				}
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
  type = uniform;
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

  cout<< type << endl;

  


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