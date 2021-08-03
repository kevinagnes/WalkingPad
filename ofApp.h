#pragma once

#include "ofMain.h"
#include "ofxCsv.h"
#include "ofxOSC.h"
#include "rapidStream.h"
#include "ofxGui.h"
#include "ofxLPF.h"
#include "ofConstants.h"
#include "ofxEasing.h"
#include "ofxCv.h"
#include "FootBlob.h"
//#include "ofxOpenCv.h"

using namespace std;
using namespace rapidlib;
using namespace glm;
using namespace cv;

class ofApp : public ofBaseApp{

	public:
		void setup();
		//void update();
		void draw();
		void setupDevices(string COM = 0, int Baud = 0);
		void keyPressed(int key);
		void keyReleased(int key);
		void drawMatrix(int startingX, int startingY, int size);
		void drawGraph(float sX, float sY, float avgX, float avgY, int sN, float speed = 0);
		void MovementDetector();
		void MovementDetector2();
		void SpeedCalculation();
		void FinalSpeedDecision();
		void OpenCV();
		void oscSend();
		void csvRecord();
		void blobDetection();
		void calculateOrientation();
		void drawMiniCoG(int rectSize);
		void drawGrid();
		void drawRecordingSquare();
		void drawOrientation();
		//bool calculateOrientation();
		bool setupCompleted = false;
		bool _calibrateNow = false;
		bool drawGui = false;
		bool skipSetup = true; // opens in Unity with auto settings
		long int currentTime =0;
		// gui
		ofxPanel gui;
		ofxPanel set;
		ofParameterGroup par;
		ofParameter<int> COMPORT, baud;
		ofParameter<bool> baudSelect;
		ofParameter<bool> confirm, simulating;
		ofParameterGroup parameters;
		ofParameter<int> T2, T3, T4,T5;
		ofParameter<float> dT;
		ofParameter<bool> _showContourFinder, _debugNewMethod, _debug, showNas, showSpeed, showSpeedMul, showAvg, showX,showY, bigMatrix, showPainting, useBlob;
		
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
		//rapidStream<float> crossingArrayX;
		//rapidStream<float> crossingArrayY;
		//rapidStream<float> smoothStep1, smoothStep2;
		deque<float> teta;
		vec2 cogOnStep[4];
		//rapidStream<float> speedX;
		//rapidStream<float> speedY;
		bool jump = false;
		bool stopped = false;
		vec2 foot1;
		vec2 foot2;
		ofxLPF lpf,smooth, finalS;

		ofxLPF lpfX, lpfY, lpfN;

		float xPos = 0;
		ofPolyline lineX;
		ofPolyline lineAvgX;
		ofPolyline lineY;
		ofPolyline lineAvgY;
		ofPolyline lineN;
		ofPolyline lineSpeed,lineSpeedMul;
		ofPolyline centroidPainting;
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
		deque<vec2>points, _points;
		vector<vec3> distances;
		vec2 centroid1, centroid2, _centroid1, _centroid2;
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
		ofxCv::ContourFinder contourFinder;
		ofImage img;
		ofParameter<float> minArea, maxArea, threshold;
		ofParameter<bool> holes;
		deque<vec2> movingAvgDir, movingAvgDirSecondary;
		int avgDirWin = 1;
		vec2 direction1 = vec2(0, 0), direction2 = vec2(0, 0), oldDirection = vec2(0, 0), finalDirection = vec2(0,0), preDirection = vec2(0, 0) ;
		vec2 Foot1a = vec2(0, 0), Foot2a = vec2(0, 0), Foot1b = vec2(0, 0), Foot2b = vec2(0, 0);
		vec2 oldDirection1 = vec2(0, 0), oldDirection2 = vec2(0, 0);
		vec2 primary = vec2(0, 0);
		vec2 secondary = vec2(0, 0); 
		vec2 contours[4];
		vector<vector<cv::Point>> quads;
		vector<vector<vector<vec2>>> Foot;
		float _angleBetweenFeet=0, _biggestDistance = 0;
		float ThresholdAngle = HALF_PI;
		float minThreshold = 40;
		bool _turning = false;
		bool useTWO = true , useFOUR = false;
		vec2 CENTER1,CENTER2;
		int cvTimer = 0, cvDebounce = 25;
		ofxLPF dirX, dirY;

		

		// orientation
		deque<vec2> stepLocations;
		vector<FootBlob> blobs;
		vec2 prevShort, prevLong1, prevLong2,previousCentroid1,previousCentroid2;
		ofxLPF a1, a2, a3;
		bool xWin = false;
		bool oldXwin = false;
		bool turned = false;
		ofColor cXcYColor = ofColor::black;
		float numOfFoot = 1;
		ofTrueTypeFont text;
		ofVec2f centroid1_, centroid2_;
		deque<vec2> C, B;
		vec2 _midC, _avgB;
		int windowSizeC = 150, windowSizeB = 60;
		//movement detector 2
		deque<float> _velocity;
		deque<float> _avgDis;
		int velWindow = 10;
		int disWindow = 10;
		vec2 _old;
		int _state = 0, _oldState = 0;
		float _s = 0, _prevS = 0, _oldVel = 0, _dif = 0;
		deque<float> _vel;
		int _timer1, _timer2, _timer3, _timer4, _timer5, _timer6, timeToStop = 2000;
		float _fs = 0, __fs;
		ofxLPF LowPass,LowPass2;
};
