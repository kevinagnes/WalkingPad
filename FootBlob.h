#pragma once
#include "ofMain.h"
using namespace glm;


class FootBlob {
public:
	float minX = 0, minY = 0, maxX = 0, maxY = 0;
	int id = 0;
	bool taken = false;
	void NewBlob(float _x, float _y);
	bool IsNear(float px, float py, float Thr);
	float distSqr(float x1, float y1, float x2, float y2);
	void add(float px, float py);
	vec4 draw(float startingX, float startingY, float spacing);
	void become(FootBlob other);
	vec2 getCenter();
	float size();

private:

};