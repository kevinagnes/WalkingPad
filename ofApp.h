#pragma once

#include "ofMain.h"
#include "ofxCsv.h"
#include "ofxOSC.h"
#include "rapidStream.h"
#include "ofxGui.h"
#include "ofxLPF.h"
#include "ofConstants.h"

using namespace std;
using namespace rapidlib;
using namespace glm;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void setupDevices();
		bool setupCompleted = false;

		void keyPressed(int key);
		void keyReleased(int key);
		void drawMatrix(int startingX, int startingY, int size);
		void drawGraph(float sX, float sY, float avgX, float avgY, int sN, float speed = 0);
		ofVec2f calculateCentroid();
		ofVec2f calculateOrientation();

		// gui
		ofxPanel gui;
		ofxPanel set;
		ofParameterGroup par;
		ofParameter<int> COMPORT, baud;
		ofParameter<bool> baudSelect;
		ofParameter<bool> confirm, simulating;
		ofParameterGroup parameters;
		ofParameter<int> T1,T2,T3,T4,T5;
		ofParameter<bool> showNas, showSpeed, showSpeedMul, showAvg, showX,showY, bigMatrix;
		
		ofColor col[2] = { ofColor(45,45,45), ofColor::red };
		ofxOscSender osc;

		ofxCsv csv;
		bool recording;
		bool toggle;
		string filename = "rec_" + 
			ofToString(ofGetDay()) + "_" +
			ofToString(ofGetMonth()) + "_" +
			ofToString(ofGetYear()) + "_" +
			ofToString(ofGetHours()) + "_" +
			ofToString(ofGetMinutes()) + "_" +
			ofToString(ofGetSeconds()) + "_" +
			".csv";
		const static int sensorsBase = 8;
		const static int sensorNumber = sensorsBase*sensorsBase;
		ofSerial serial;
		bool connected = false;
		int counter = 0;
		float reading[3];
		int readingArray[sensorNumber];
		string comPort = "COM";
		int baudTypes[2] = { 57600,115200 };
		
		rapidStream<float> cogArrayX;
		rapidStream<float> cogArrayY;
		rapidStream<float> nasArray;
		rapidStream<float> velArrayX;
		rapidStream<float> velArrayY;
		rapidStream<float> crossingArrayX;
		rapidStream<float> crossingArrayY;
		rapidStream<float> smoothStep1, smoothStep2;
		rapidStream<float> teta;
		vec2 cogOnStep[4];
		//rapidStream<float> speedX;
		//rapidStream<float> speedY;
		bool jump = false;
		vec2 foot1;
		vec2 foot2;
		ofxLPF lpf,smooth;

		float xPos = 0;
		ofPolyline lineX;
		ofPolyline lineAvgX;
		ofPolyline lineY;
		ofPolyline lineAvgY;
		ofPolyline lineN;
		ofPolyline lineSpeed,lineSpeedMul;

		float X = 0;
		float Y = 0;
		int N = 0;
		float sX = 0;
		float sY = 0;
		float sN = 0;
		float avX = 0;
		float avY = 0;
		float speed = 0;
		int timer1, timer2, timer3,timer4;
		vec2 orientationAngle;
		float Angle;
		bool dontDraw = false;
		int steps = 0;
		int freqX = 0, freqY = 0, oldFreqX = 0, oldFreqY = 0;
		bool changedX = false;
		bool changedY = false;
		float speedmul = 0;
		int stepTimer1, stepTimer2;
		int deltaStep = 0;

		bool leftDown = false, rightDown = false, upDown = false, downDown = false;
		bool firstContact = false;
		void tryFirstConnection();
		int timeOut = 0, timeFromPreviousCall = 0;
		int timeToCalibrate = 0, timeWithoutFeet = 0;
		deque<ofVec2f>points;
		deque<ofVec2f> blobs;
		vector<ofVec2f>blob;
		ofVec2f centroid1,centroid2;
		int timerrr=0;
};

