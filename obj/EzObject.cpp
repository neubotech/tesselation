#include "EzObject.h"
#include <cstdio>
#include <iostream>
#include <string>
using namespace std;

bool beginWith(const char* str, const char* pattern) {
	int i = 0;
	while (pattern[i] != '\0' && str[i] == pattern[i]) i++;
	return pattern[i] == '\0';
}

void EzObject::write(const char *fileName) {
	FILE *fp = fopen(fileName, "w");
	for (int i = 0; i < vertexCount; i++) {
		fprintf(fp, "v %f %f %f\n", vertexPosition[i].x, vertexPosition[i].y, vertexPosition[i].z);
	}
	for (int i = 0; i < faceCount; i++) {
		fprintf(fp, "f %d %d %d\n", faceList[i].x + 1, faceList[i].y + 1, faceList[i].z + 1);
	}

	fclose(fp);
}

void EzObject::read(char* fileName) {
	printf("Loading object file...");

	vertexPosition.clear();
	vertexNormal.clear();
	vertexColor.clear();
	faceList.clear();
	faceNormal.clear();

	kd = Vector3f(0.0f, 0.0f, 0.0f);
	ke = Vector3f(0.0f, 0.0f, 0.0f);
	ks = Vector3f(0.0f, 0.0f, 0.0f);
	tf = Vector3f(0.0f, 0.0f, 0.0f);
	area = 0.0f;
	brdfParam = 1;
	renderMode = "flat";

	FILE *fp = fopen(fileName, "r");
	char line[1000];
	while (fscanf(fp, " %[^\n]", line) != EOF) {
		string l(line);
		if (beginWith(line, "v ")) {
			double vx, vy, vz;
			sscanf(line, " %*s %lf %lf %lf%*[^\n]", &vx, &vy, &vz);
			vertexPosition.push_back(Vector3f(vx, vy+5, vz));
			vertexNormal.push_back(Vector3f(0.0, 0.0, 0.0));
			vertexColor.push_back(Vector3f(0.0, 0.0, 0.0));
		} else if (beginWith(line, "f ")) {
			int vi1, vi2, vi3;
			if (l.find("//") != std::string::npos)
  				sscanf(line, " %*s %d//%*d %d//%*d %d//%*d%*[^\n]", &vi1, &vi2, &vi3);
  			else
  				sscanf(line, " %*s %d %d %d%*[^\n]", &vi1, &vi2, &vi3);
			faceList.push_back(Vector3i(vi1 - 1, vi2 - 1, vi3 - 1));		
			Vector3f faceNormalX = TriangleNormal(vertexPosition[vi1 - 1], vertexPosition[vi2 - 1], vertexPosition[vi3 - 1]);
			faceNormal.push_back(faceNormalX);
			vertexNormal[vi1 - 1] += faceNormalX;
			vertexNormal[vi2 - 1] += faceNormalX;
			vertexNormal[vi3 - 1] += faceNormalX;
			area += TriangleArea(vertexPosition[vi1 - 1], vertexPosition[vi2 - 1], vertexPosition[vi3 - 1]);
		}
	}
	fclose(fp);

	vertexCount = vertexPosition.size();
	faceCount = faceList.size();

	for (int i = 0; i < vertexCount; i++)
		vertexNormal[i].normalize();

	printf("OK!\n");
	printf("    Vertices: %d    Faces: %d\n\n", vertexCount, faceCount);

	buildKdTree();
}


EzObject::EzObject()
{
	xDirection = Vector3f(1.0, 0.0, 0.0);
	yDirection = Vector3f(0.0, 1.0, 0.0);
	zDirection = Vector3f(0.0, 0.0, 1.0);
}

void EzObject::buildKdTree()
{
	printf("KD Tree Reading Object...");
	root = new KdTreeNode;
	float min_x, min_y, min_z, max_x, max_y, max_z;
	min_x = min_y = min_z = INF;
	max_x = max_y = max_z = -INF;
	for (int i = 0; i < vertexPosition.size(); i++)
	{
		min_x = min(min_x, vertexPosition[i].x);
		min_y = min(min_y, vertexPosition[i].y);
		min_z = min(min_z, vertexPosition[i].z);
		max_x = max(max_x, vertexPosition[i].x);
		max_y = max(max_y, vertexPosition[i].y);
		max_z = max(max_z, vertexPosition[i].z);
	}
	min_x -= EPS;
	max_x += EPS;
	min_y -= EPS;
	max_y += EPS;
	min_z -= EPS;
	max_z += EPS;

	if (max_x - min_x <= EPS * 200.0f) {
		max_x += EPS * 200.0f;
		min_x -= EPS * 200.0f;
	}
	if (max_y - min_y <= EPS * 200.0f) {
		max_y += EPS * 200.0f;
		min_y -= EPS * 200.0f;
	}
	if (max_z - min_z <= EPS * 200.0f) {
		max_z += EPS * 200.0f;
		min_z -= EPS * 200.0f;
	}


	root->center = Vector3f((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2);
	root->dx = (max_x - min_x) / 2;
	root->dy = (max_y - min_y) / 2;
	root->dz = (max_z - min_z) / 2;
	root->left = root->right = NULL;
	root->direction = 0;
	root->depth = 0;
	for (int i = 0; i < faceList.size(); i++)
	{
		root->faceID.push_back(i);
	}
	printf("OK!\n");
	printf("KD Tree Building...");
	build(root);
	printf("OK!\n\n");
}

void EzObject::build(KdTreeNode* node)
{
	if (node->faceID.size() <= 15 || node->depth >= 20) return;

	KdTreeNode *left = new KdTreeNode, *right = new KdTreeNode;
	node->left = left;
	node->right = right;
	left->left = left->right = right->left = right->right = NULL;
	left->center = right->center = node->center;
	left->depth = right->depth = node->depth + 1;
	left->direction = right->direction = (node->direction + 1) % 3;
	left->dx = right->dx = node->dx;
	left->dy = right->dy = node->dy;
	left->dz = right->dz = node->dz;
	switch (node->direction)
	{
	case 0:
		left->center.x -= node->dx / 2.0;
		right->center.x += node->dx / 2.0;
		left->dx = right->dx = node->dx / 2.0;
		break;
	case 1:
		left->center.y -= node->dy / 2.0;
		right->center.y += node->dy / 2.0;
		left->dy = right->dy = node->dy / 2.0;
		break;
	case 2:
		left->center.z -= node->dz / 2.0;
		right->center.z += node->dz / 2.0;
		left->dz = right->dz = node->dz / 2.0;
		break;
	}

	for (int i = 0; i < node->faceID.size(); i++)
	{
		if (triangleIntersectBox(left->center.x, left->center.y, left->center.z,
			left->dx, left->dy, left->dz,
			vertexPosition[faceList[node->faceID[i]].x].x, vertexPosition[faceList[node->faceID[i]].x].y, vertexPosition[faceList[node->faceID[i]].x].z,
			vertexPosition[faceList[node->faceID[i]].y].x, vertexPosition[faceList[node->faceID[i]].y].y, vertexPosition[faceList[node->faceID[i]].y].z,
			vertexPosition[faceList[node->faceID[i]].z].x, vertexPosition[faceList[node->faceID[i]].z].y, vertexPosition[faceList[node->faceID[i]].z].z))
		{
			left->faceID.push_back(node->faceID[i]);
		}
		if (triangleIntersectBox(right->center.x, right->center.y, right->center.z,
			right->dx, right->dy, right->dz,
			vertexPosition[faceList[node->faceID[i]].x].x, vertexPosition[faceList[node->faceID[i]].x].y, vertexPosition[faceList[node->faceID[i]].x].z,
			vertexPosition[faceList[node->faceID[i]].y].x, vertexPosition[faceList[node->faceID[i]].y].y, vertexPosition[faceList[node->faceID[i]].y].z,
			vertexPosition[faceList[node->faceID[i]].z].x, vertexPosition[faceList[node->faceID[i]].z].y, vertexPosition[faceList[node->faceID[i]].z].z))
		{
			right->faceID.push_back(node->faceID[i]);
		}
	}
	if (right->faceID.size() == 0)
	{
		node->center = left->center;
		node->direction = left->direction;
		node->dx = left->dx;
		node->dy = left->dy;
		node->dz = left->dz;
		node->left = node->right = NULL;
		delete left;
		delete right;
		build(node);
	}
	else if (left->faceID.size() == 0)
	{
		node->center = right->center;
		node->direction = right->direction;
		node->dx = right->dx;
		node->dy = right->dy;
		node->dz = right->dz;
		node->left = node->right = NULL;
		delete left;
		delete right;
		build(node);
	}
	else
	{
		build(left);
		build(right);
	}
}

bool EzObject::triangleIntersectSurface(int faceType, float cx, float cy, float cz, float dx, float dy, float dz,
	float p1x, float p1y, float p1z,
	float p2x, float p2y, float p2z,
	float p3x, float p3y, float p3z)
{
	int positive = 0;
	int negative = 0;
	switch (faceType)
	{
	case 0:
		// X
		if (p1x - cx >= 0) positive++;
		if (p1x - cx <= 0) negative++;
		if (p2x - cx >= 0) positive++;
		if (p2x - cx <= 0) negative++;
		if (p3x - cx >= 0) positive++;
		if (p3x - cx <= 0) negative++;
		if (positive * negative == 0) return false;
		if ((p1y < cy - dy && p2y < cy - dy && p3y < cy - dy)) return false;
		if ((p1y > cy + dy && p2y > cy + dy && p3y > cy + dy)) return false;
		if ((p1z < cz - dz && p2z < cz - dz && p3z < cz - dz)) return false;
		if ((p1z > cz + dz && p2z > cz + dz && p3z > cz + dz)) return false;
		break;
	case 1:
		// Y
		if (p1y - cy >= 0) positive++;
		if (p1y - cy <= 0) negative++;
		if (p2y - cy >= 0) positive++;
		if (p2y - cy <= 0) negative++;
		if (p3y - cy >= 0) positive++;
		if (p3y - cy <= 0) negative++;
		if (positive * negative == 0) return false;
		if ((p1x < cx - dx && p2x < cx - dx && p3x < cx - dx)) return false;
		if ((p1x > cx + dx && p2x > cx + dx && p3x > cx + dx)) return false;
		if ((p1z < cz - dz && p2z < cz - dz && p3z < cz - dz)) return false;
		if ((p1z > cz + dz && p2z > cz + dz && p3z > cz + dz)) return false;
		break;
	case 2:
		// Z
		if (p1z - cz >= 0) positive++;
		if (p1z - cz <= 0) negative++;
		if (p2z - cz >= 0) positive++;
		if (p2z - cz <= 0) negative++;
		if (p3z - cz >= 0) positive++;
		if (p3z - cz <= 0) negative++;
		if (positive * negative == 0) return false;
		if ((p1x < cx - dx && p2x < cx - dx && p3x < cx - dx)) return false;
		if ((p1x > cx + dx && p2x > cx + dx && p3x > cx + dx)) return false;
		if ((p1y < cy - dy && p2y < cy - dy && p3y < cy - dy)) return false;
		if ((p1y > cy + dy && p2y > cy + dy && p3y > cy + dy)) return false;
		break;
	}
	return true;
}

bool EzObject::triangleIntersectBox(float cx, float cy, float cz, float dx, float dy, float dz,
	float p1x, float p1y, float p1z,
	float p2x, float p2y, float p2z,
	float p3x, float p3y, float p3z)
{
	if (triangleIntersectSurface(0, cx + dx, cy, cz, 0.0, dy, dz, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (triangleIntersectSurface(0, cx - dx, cy, cz, 0.0, dy, dz, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (triangleIntersectSurface(1, cx, cy + dy, cz, dx, 0.0, dz, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (triangleIntersectSurface(1, cx, cy - dy, cz, dx, 0.0, dz, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (triangleIntersectSurface(2, cx, cy, cz + dz, dx, dy, 0.0, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (triangleIntersectSurface(2, cx, cy, cz - dz, dx, dy, 0.0, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z)) return true;
	if (p1x >= cx - dx && p1x <= cx + dx && p1y >= cy - dy && p1y <= cy + dy && p1z >= cz - dz && p1z <= cz + dz) return true;
	if (p2x >= cx - dx && p2x <= cx + dx && p2y >= cy - dy && p2y <= cy + dy && p2z >= cz - dz && p2z <= cz + dz) return true;
	if (p3x >= cx - dx && p3x <= cx + dx && p3y >= cy - dy && p3y <= cy + dy && p3z >= cz - dz && p3z <= cz + dz) return true;
	return false;
}

bool EzObject::rayIntersectBox(Vector3f &start, Vector3f &direction, KdTreeNode *node)
{
	Vector3f &c = node->center;
	float &dx = node->dx;
	float &dy = node->dy;
	float &dz = node->dz;
	Vector3f p = c - start;
	float e, f, k, t1, t2;
	float t_min = -INF;
	float t_max = INF;

	e = xDirection * p;
	f = xDirection * direction;
	t1 = (e + dx) / f;
	t2 = (e - dx) / f;
	if (t1 > t2) {k = t1; t1 = t2; t2 = k;}
	t_min = max(t_min, t1);
	t_max = min(t_max, t2);

	e = yDirection * p;
	f = yDirection * direction;
	t1 = (e + dy) / f;
	t2 = (e - dy) / f;
	if (t1 > t2) {k = t1; t1 = t2; t2 = k;}
	t_min = max(t_min, t1);
	t_max = min(t_max, t2);

	e = zDirection * p;
	f = zDirection * direction;
	t1 = (e + dz) / f;
	t2 = (e - dz) / f;
	if (t1 > t2) {k = t1; t1 = t2; t2 = k;}
	t_min = max(t_min, t1);
	t_max = min(t_max, t2);

	return t_max > t_min && t_max > 0.0;
}

bool EzObject::rayIntersect(Vector3f &start, Vector3f &direction, KdTreeNode *root, Vector3f &normal, float &t, int &faceId)
{
	if (rayIntersectBox(start, direction, root) == false) return false;
	if (root->left == NULL)
	{
		Vector3f tn, nn;
		float min_t = INF;
		for (int i = 0; i < root->faceID.size(); i++)
		{
			float tt;
			if (rayIntersectTriangle(start, direction, root->faceID[i], tn, tt) && tt < min_t) {min_t = tt; nn = tn; faceId = root->faceID[i];}
		}
		if (min_t == INF) return false;
		t = min_t;
		normal = nn;
		return true;
	}
	Vector3f n1, n2;
	float t1, t2;
	bool intersectLeft = rayIntersect(start, direction, root->left, n1, t1, faceId);
	bool intersectRight = rayIntersect(start, direction, root->right, n2, t2, faceId);
	if (!intersectLeft && !intersectRight) return false;
	if (intersectLeft && intersectRight)
	{
		if (t1 < t2) {normal = n1; t = t1;}
		else {normal = n2; t = t2;}
	}
	else if (intersectLeft)
	{
		normal = n1; t = t1;
	}
	else
	{
		normal = n2; t = t2;
	}

	return true;
}

bool EzObject::rayIntersectTriangle(Vector3f &start, Vector3f &direction, int faceID, Vector3f &normal, float &t)
{
	Vector3f& v1 = vertexPosition[faceList[faceID].x];
	Vector3f& v2 = vertexPosition[faceList[faceID].y];
	Vector3f& v3 = vertexPosition[faceList[faceID].z];
	Vector3f e1 = v1 - v2;
	Vector3f e2 = v1 - v3;
	Vector3f s = v1 - start;
	float denominator = Det(direction, e1, e2);
	t = Det(s, e1, e2) / denominator;
	float beta = Det(direction, s, e2) / denominator;
	float gamma = Det(direction, e1, s) / denominator;
	if (t > EPS && beta >= 0 && gamma >= 0 && beta + gamma <= 1)
	{
		if (renderMode == "smooth") {
			Vector3f &n1 = vertexNormal[faceList[faceID].x];
			Vector3f &n2 = vertexNormal[faceList[faceID].y];
			Vector3f &n3 = vertexNormal[faceList[faceID].z];
			Vector3f p = start + direction * t;
			Vector3f ne1 = v1 - p;
			Vector3f ne2 = v2 - p;
			Vector3f ne3 = v3 - p;
			float area = TriangleArea(e1, e2);
			float w1 = TriangleArea(ne2, ne3) / area;
			float w2 = TriangleArea(ne3, ne1) / area;
			float w3 = TriangleArea(ne1, ne2) / area;
			normal = (n1 * w1 + n2 * w2 + n3 * w3).normalize();
		} else {
			normal = faceNormal[faceID];
		}
		return true;
	}
	return false;
}

bool EzObject::rayIntersectTriangle(Vector3f &start, Vector3f &direction, int faceID)
{
	Vector3f& v1 = vertexPosition[faceList[faceID].x];
	Vector3f& v2 = vertexPosition[faceList[faceID].y];
	Vector3f& v3 = vertexPosition[faceList[faceID].z];
	Vector3f e1 = v1 - v2;
	Vector3f e2 = v1 - v3;
	Vector3f s = v1 - start;
	float denominator = Det(direction, e1, e2);
	float t = Det(s, e1, e2) / denominator;
	float beta = Det(direction, s, e2) / denominator;
	float gamma = Det(direction, e1, s) / denominator;
	if (t > EPS && beta >= 0 && gamma >= 0 && beta + gamma <= 1) return true;
	return false;
}

bool EzObject::rayIntersect(Vector3f &start, Vector3f &direction, KdTreeNode *root)
{
	if (rayIntersectBox(start, direction, root) == false) return false;
	if (root->left == NULL)
	{
		for (int i = 0; i < root->faceID.size(); i++)
			if (rayIntersectTriangle(start, direction, root->faceID[i]))
				return true;
		return false;
	}
	return rayIntersect(start, direction, root->left) || rayIntersect(start, direction, root->right);
}
