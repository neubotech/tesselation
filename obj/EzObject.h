#ifndef EZ_OBJECT_H
#define EZ_OBJECT_H

#include "EzMath.h"
#include <vector>
#include <string>
using namespace std;

typedef struct kd_tree_node
{
	int direction;
	int depth;
	float dx, dy, dz;
	Vector3f center;
	vector<int> faceID;
	struct kd_tree_node *left, *right;
} KdTreeNode;

class EzObject
{
// Object
public:
	const static unsigned int DRAW_NORMAL_ARRAY = 1;
	const static unsigned int DRAW_COLOR_ARRAY = 2;
public:
	vector<Vector3f> vertexPosition;
	vector<Vector3f> vertexNormal;
	vector<Vector3f> vertexColor;
	vector<Vector3i> faceList;
	vector<Vector3f> faceNormal;
	int vertexCount;
	int faceCount;

	Vector3f kd, ke, ks, tf;
	float area;
	int brdfParam;
	string renderMode;
public:
	void read(char* fileName);
	void process();
	void write(const char *fileName);

// KdTree
public:
	KdTreeNode *root;
	Vector3f xDirection;
	Vector3f yDirection;
	Vector3f zDirection;
public:
	void build(KdTreeNode *node);
	bool triangleIntersectSurface(int faceType, float cx, float cy, float cz, float dx, float dy, float dz,
		float p1x, float p1y, float p1z,
		float p2x, float p2y, float p2z,
		float p3x, float p3y, float p3z);
	bool triangleIntersectBox(float cx, float cy, float cz, float dx, float dy, float dz,
		float p1x, float p1y, float p1z,
		float p2x, float p2y, float p2z,
		float p3x, float p3y, float p3z);
	bool rayIntersectBox(Vector3f &start, Vector3f &direction, KdTreeNode *node);
	bool rayIntersectTriangle(Vector3f &start, Vector3f &direction, int faceID, Vector3f &normal, float &t);
	bool rayIntersectTriangle(Vector3f &start, Vector3f &direction, int faceID);
public:
	EzObject();
	void buildKdTree();
	bool rayIntersect(Vector3f &start, Vector3f &direction, KdTreeNode *root, Vector3f &normal, float &t, int &faceId);
	bool rayIntersect(Vector3f &start, Vector3f &direction, KdTreeNode *root);

};

#endif
