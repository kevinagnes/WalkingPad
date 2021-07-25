#include "FootBlob.h"


void FootBlob::NewBlob(float _x, float _y) {
	minX = _x;
	minY = _y;
	maxX = _x;
	maxY = _y;

}

bool FootBlob::IsNear(float px, float py, float Thr) {
	float cx = MAX(MIN(px, maxX), minX);
	float cy = MAX(MIN(py, maxY), minY);
	float d = distSqr(cx, cy, px, py);
	if (d < Thr*Thr) return true;
	else return false;
}


float FootBlob::distSqr(float x1, float y1, float x2, float y2) {
	float d = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1); 
	return d;
}

void FootBlob::add(float px, float py) {
	minX = MIN(minX, px);
	minY = MIN(minY, py);
	maxX = MAX(maxX, px);
	maxY = MAX(maxY, py);
}

vec4 FootBlob::draw(float startingX,float startingY,float spacing) {
	ofPushStyle();
	ofSetLineWidth(15);
	vec2 start = vec2(startingX, startingY);
	vec2 min = vec2(minX, minY);
	vec2 max = vec2(maxX, maxY);
	vec2 p1 = start + (min * spacing);
	vec2 p2 = start + (max * spacing);
	ofSetRectMode(OF_RECTMODE_CORNER);
	ofRectangle r(p2, p1);
	ofNoFill();
	ofSetColor(ofColor::yellow);
	ofDrawLine(p2, p1);
	ofDrawRectangle(r);
	ofFill();
	ofSetColor(ofColor::yellow,127);
	ofDrawRectangle(r);
	ofSetColor(ofColor::red);
	ofDrawCircle(p1,20);
	ofSetColor(ofColor::green);
	ofDrawCircle(p2, 10);
	ofPopStyle();
	return vec4(p1, p2);

}

vec2 FootBlob::getCenter() {
	float x = (maxX - minX) * 0.5f + minX;
	float y = (maxY - minY) * 0.5f + minY;
	return vec2(x, y);
}

float FootBlob::size() {
	return (maxX - minX) * (maxY - minY);
}

void FootBlob::become(FootBlob other) {
	minX = other.minX;
	maxY = other.maxY;
	minX = other.minX;
	maxY = other.maxY;
}