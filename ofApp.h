#pragma once

#include "ofMain.h"
#include "ofxCsv.h"
#include "ofxOSC.h"
#include "rapidStream.h"
#include "ofxGui.h"
#include "ofxLPF.h"
#include "ofConstants.h"
#include "ofxEasing.h"
#include "ofxOpenCv.h"

using namespace std;
using namespace rapidlib;
using namespace glm;
using namespace cv;


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void setupDevices(string COM = 0, int Baud = 0);
		void keyPressed(int key);
		void keyReleased(int key);
		void drawMatrix(int startingX, int startingY, int size);
		void drawGraph(float sX, float sY, float avgX, float avgY, int sN, float speed = 0);
		void MovementDetector();
		void SpeedCalculation();
		void FinalSpeedDecision();
		void OpenCV();
		void oscSend();
		void csvRecord();

		void calculateOrientation();
		//bool calculateOrientation();
		bool setupCompleted = false;
		bool _calibrateNow = false;
		bool drawGui = false;
		bool skipSetup = true; // opens in Unity with auto settings
		// gui
		ofxPanel gui;
		ofxPanel set;
		ofParameterGroup par;
		ofParameter<int> COMPORT, baud;
		ofParameter<bool> baudSelect;
		ofParameter<bool> confirm, simulating;
		ofParameterGroup parameters;
		ofParameter<int> T2,T3,T4,T5;
		ofParameter<bool> _debugOpticalFlow, _debugNewMethod, _debug, showNas, showSpeed, showSpeedMul, showAvg, showX,showY, bigMatrix;
		
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
		const static int sensorsBase = 16;
		const static int sensorNumber = sensorsBase*sensorsBase;
		ofSerial serial;
		bool connected = false;
		int counter = 0;
		float reading[3];
		unsigned char readingArray[sensorNumber];
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
		bool stopped = false;
		vec2 foot1;
		vec2 foot2;
		ofxLPF lpf,smooth, finalS;

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
		int timer1, timer2, timer3,timer4, timer5,timer6;
		vec2 orientationAngle;
		float Angle;
		bool dontDraw = false;
		int steps = 0;
		int freqX = 0, freqY = 0, oldFreqX = 0, oldFreqY = 0;
		bool changedX = false;
		bool changedY = false;
		float movementDetection = 0;
		int stepTimer1, stepTimer2;
		int deltaStep = 0, oldDeltaStep = 0;
		bool leftDown = false, rightDown = false, upDown = false, downDown = false;
		bool firstContact = false;
		void tryFirstConnection();
		int timeOut = 0, timeFromPreviousCall = 0;
		int timeToCalibrate = 0, timeWithoutFeet = 0;
		deque<vec2>points, _points, blobs;
		vector<vec3> distances;
		vector<ofVec2f>blob;
		ofVec2f centroid1, centroid2, _centroid1, _centroid2;
		//rapidStream<float> _centroid1_x, _centroid1_y, _centroid2_x, _centroid2_y;
		int timerrr=0;
		bool crossed = false;
		bool switcher = false;

		// variables for the speed
		const vec2 initialCoG = vec2(8, 8);
		vec2 currentCentroid = initialCoG, previousCentroid = initialCoG, avgCentroid = initialCoG, previousStep = initialCoG, rateOfChange = vec2(0,0), finalSpeed = vec2(0, 0);
		deque<vec2> tempCurrentCentroid,avgRateOfChange, avgSpeed;
		deque<float> lastFewSpeeds;
		ofParameter<float> _constant;
		const float constant = (float)1/60;
		float elapsedTime = constant;
		int _Timer1 = 0, _Timer2 = 0, _Timer3 = 0, coounter =0;
		float _stepDist = 0,stepDistance = 0, theSpeed = 0, oldSpeed = 0;
		bool wait = false, initial = true, stepDetected = false;


		int T6 = 50;

		// OpenCV OpticalFlow Idea
		bool calculatedFlow;
		int flowTime=0;
		ofParameter<float> T1;
		ofxCvColorImage currentColor;		//First and second original images
		ofxCvFloatImage flowX, flowY;		//Resulted optical flow in x and y axes
		ofxCvGrayscaleImage gray1, gray2;
};