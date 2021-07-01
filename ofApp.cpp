#include "ofApp.h"
#include "opencv2/opencv.hpp"
//------------------    --------------------------------------------
void ofApp::setup(){
    dontDraw = false;
    ofBackground(col[0]);
    ofSetBackgroundAuto(true);
    ofSetFrameRate(60);

    ofxGuiSetDefaultHeight(20);
    ofxGuiSetDefaultWidth(150); 
    parameters.setName("Thres & Delay");
    parameters.add(T1.set("OpticalFlow: ", 0.1f, 0, 1));
    parameters.add(T2.set("Blob Thresh: ", 50, 1, 255));
    parameters.add(T3.set("avgZeroCrossing: ", 20, 0, 500));
    parameters.add(T4.set("stepDebounce: ", 150, 0, 500));
    parameters.add(T5.set("ZeroCrossing Thresh: ", 25, 0, 400));
    parameters.add(_constant.set("CONSTANT: ", 1, 0, 3));
    parameters.add(_debug.set("showDebug", false));
    parameters.add(_debugOpticalFlow.set("showOpticalFlow", false));
    parameters.add(_debugNewMethod.set("showCentroid", true));
    parameters.add(bigMatrix.set("show Big Matrix", false));
    parameters.add(showX.set("X CoG", true));
    parameters.add(showY.set("Y CoG", true));
    parameters.add(showSpeed.set("Speed",true));
    parameters.add(showSpeedMul.set("SpeedMultiplier", false));
    parameters.add(showAvg.set("ZeroCrossingLine", false));
    parameters.add(showNas.set("NumberOfActiveSensors", false));
    gui.setup(parameters);
    gui.setPosition(ofGetWidth() - gui.getWidth(), 0);

    set.setName("Serial Com: ");
    par.add(baud.set("BAUD ", baud, baudTypes[0], baudTypes[1]));
    par.add(baudSelect.set("BAUD SET", false));
    par.add(COMPORT.set("COM ", 12, 1, 15));
    par.add(simulating.set("Keyboard Simulating: ", false));
    par.add(confirm.set("OK", false));
    set.setup(par);
    smooth.initialize(60, 1.2f);
    finalS.initialize(60, 2);
    
    calculatedFlow = false;
}
//--------------------------------------------------------------
void ofApp::update(){}
//--------------------------------------------------------------
void ofApp::draw(){
    // setup COM ports or keyboard simulation
    if (!setupCompleted) {
        set.draw();
        baud = baudTypes[baudSelect];
        if (confirm) {
            comPort += COMPORT.toString();
            if (!simulating) setupDevices();
            setupCompleted = true;
        }
        return;
    }

    // OSC
    osc.setup("localhost", 6969);

    if (!setupCompleted) return set.draw();

    if (dontDraw) return;

    // Get transmition of CoG and Array from WalkingPad
    if (!simulating) {

        // setup arduino connection
        if (serial.available() < 0) {
            //serial.writeByte('A');
            connected = false;
        }
        else {
            //if (!firstContact) tryFirstConnection();
            connected = true;

            while (serial.available() > 0) {
                // ask microcontroller to calibrate
                if (_calibrateNow) {
                    serial.writeByte('C');
                    ofSetColor(ofColor::green);
                    ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
                    _calibrateNow = false;
                }
                else {
                    // Pressure-sensitive MATRIX values
                    char byteReturned = serial.readByte();
                    if (counter < sensorNumber - 1)readingArray[counter] = byteReturned;
                    counter++;

                    if (counter > sensorNumber) {
                        if (_debug) std::cout << "Time from previous reading: " << ofGetElapsedTimeMillis() - timeFromPreviousCall << " ms" << endl;
                        timeFromPreviousCall = ofGetElapsedTimeMillis();
                        serial.writeByte('A');
                        counter = 0;
                        if (gray1.bAllocated) {
                            gray2 = gray1;
                            calculatedFlow = true;
                        }
                    }
                }
            }
        }

        // converts readingArray to an openCV image and calculates optical flow
        OpenCV();

        // Live Centroid calculation
        vec2 sum = vec2(0, 0);
        int total = 0;
        for (int i = 0; i < sensorsBase; i++) {
            for (int j = 0; j < sensorsBase; j++) {
                if (readingArray[(i * sensorsBase) + j] > T2) {
                    sum += vec2(i, j);
                    total++;
                }
            }
        }
        if (total != 0) currentCentroid = sum / total;

        // reading is multiplied to this range for visualisation purposes
        // before, the values originaly vary from 0 to 15
        // after,  the values vary from -400 to +400 (800 range)
        X = 1 * (currentCentroid.x * 50 - 400);
        Y = 1 * (currentCentroid.y * 50 - 400);
        N = total;

    }
    // Or get Keyboard Simulation
    else {
        if (leftDown && rightDown) {
            X = 0; Y = 0; N = 26;
        }
        else {
            if (leftDown) {
                X = -200; Y = 0; N = 13;
            }
            else if (rightDown) {
                X = 200; Y = 0; N = 13;
            }
        }
        if (upDown && downDown) {
            X = 0; Y = 0; N = 26;
        }
        else {
            if (upDown) {
                X = 0; Y = -200; N = 13;
            }
            else if (downDown) {
                X = 0; Y = 200; N = 13;
            }
        }
        if (!upDown && !downDown && !leftDown && !rightDown) {
            X = 0; Y = 0; N = 26;
        }
    }


    Speed(); 
    Processing();
    oscSend();

    // draw grid
    ofPushStyle();
    for (float y = 0; y < ofGetHeight(); y += ofGetHeight()/16) {
        ofSetColor(80,100);
        ofDrawLine(0, y, ofGetWidth(), y);
    }
    ofPopStyle();



    // draw the graph
    drawGraph(sX, sY, avX, avY, sN, speed);

    // draw if jump
    if (jump)    ofDrawCircle(ofGetWidth(), ofGetHeight(), 50);

    // draw if zero crossed
    if (crossed) ofDrawCircle(ofGetWidth(), 0, 50);

    //draw CoG
    ofPushMatrix();
    ofPushStyle();
    ofTranslate(140, 40);
    ofSetColor(127);
    ofNoFill();
    int rectSize = 70;
    ofDrawRectangle(-rectSize / 2, -rectSize / 2, rectSize, rectSize);
    ofDrawCircle(sX / 12, sY / 12, 7);
    ofFill();
    ofDrawCircle(X / 12, Y / 12, 4);
    ofPopStyle();
    ofPopMatrix();

    //draw OpenCV  
    //gray1.mirror(true, false);
    if (_debugOpticalFlow) gray1.draw(ofGetWidth() *.5f - ofGetHeight() * .5f, ofGetHeight() * .5f - ofGetHeight() * .5f, ofGetHeight(), ofGetHeight());

    //draw Matrix
    if (bigMatrix) drawMatrix(ofGetWidth() * .5f - ofGetHeight() * .5f, ofGetHeight() * .5f - ofGetHeight() * .5f, ofGetHeight());
    else drawMatrix(20, 10, rectSize);

    // draw recording guide
    ofPushStyle();
    ofSetColor(col[toggle]);
    ofSetLineWidth(10);
    ofNoFill();
    ofDrawRectangle(0, 0, ofGetWidth() - 3, ofGetHeight() - 3);
    ofPopStyle();

    // draw GUI
    gui.draw();

}

void ofApp::drawMatrix(int startingX, int startingY, int size) {
    ofPushStyle();
    //calculateCentroid();
    // draw matrix
    int spacing = size / sensorsBase;
    //Optical flow
    float* flowXPixels = flowX.getPixelsAsFloats();
    float* flowYPixels = flowY.getPixelsAsFloats();
    float sumX=0, sumY = 0, avgX = 0, avgY = 0;
    int numOfEntries;
    numOfEntries = 0;

    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j ++) {
            ofSetColor(255);
            ofPushMatrix();
            float ReadingMapped = ofMap(readingArray[(i * sensorsBase) + j], 0, 80, 0, 255, true);
            ofTranslate(startingX+(i*spacing),startingY+(j*spacing));
            ofNoFill();
            ofDrawRectangle(0, 0, spacing, spacing);
            ofColor pressureColor = ReadingMapped;
            if (ReadingMapped < 100 && _debugNewMethod) pressureColor.a = 50;
            ofSetColor(pressureColor);
            ofFill();
            ofDrawRectangle(0, 0, spacing, spacing);
            if (_debugOpticalFlow) {
                if (calculatedFlow) {
                    //optical Flow
                    ofPushStyle();
                    ofSetLineWidth(3);
                    ofSetColor(255, 255, 0);
                    float fx = flowXPixels[(i * sensorsBase) + j];
                    float fy = flowYPixels[(i * sensorsBase) + j];
                    //Draw only long vectors
                    if (fabs(fx) + fabs(fy) > T1) {
                        ofDrawCircle(-0.5, -0.5, spacing / 10);
                        ofDrawLine(0, 0, i + fx, j + fy);
                        numOfEntries += 1;
                        sumX += fx;
                        sumY += fy;
                    }
                    ofPopStyle();
                }
            }
            ofPopMatrix();
        }
    }
    
    if (numOfEntries > 0) {
        avgX = sumX / numOfEntries;
        avgY = sumY / numOfEntries;
    }

    if (_debug) std::cout << "avgX: " << avgX << endl;
    if (_debug) std::cout << "avgY: " << avgY << endl;
    if (_debug) std::cout << "numOfEntries: " << numOfEntries << endl;

    // draw points
    //ofSetColor(255, 255, 0);
    //for (int i = 0; i < points.size(); i++) {
    //    ofDrawCircle(startingX + (points[i].x * spacing),
    //                 startingY + (points[i].y * spacing),
    //                 spacing/3);
    //}

    // draw centroids
    ofSetColor(255, 0, 0);
    ofDrawCircle(startingX + (centroid1.x * spacing),
                 startingY + (centroid1.y * spacing),
                 spacing / 4);
    ofSetColor(0, 0, 255);
    ofDrawCircle(startingX + (centroid2.x * spacing),
        startingY + (centroid2.y * spacing),
        spacing / 4);
    // draw connecting lines
    ofSetColor(0,255,0);  
    ofSetLineWidth(3);
    ofPolyline stepsLine;
    float hipotenuse = abs(centroid1.distance(centroid2));
    if (hipotenuse > 3) {
        stepsLine.addVertex(startingX + centroid1.x * spacing, startingY + centroid1.y * spacing);
        stepsLine.addVertex(startingX + centroid2.x * spacing, startingY + centroid2.y * spacing);  
        if (_debug) std::cout << teta.mean() << endl;
    }
    stepsLine.draw();
    
    if (_debugNewMethod) {
        ofNoFill();
        ofSetColor(ofColor::white,75);
        ofDrawCircle(startingX + (previousStep.x * spacing),
            startingY + (previousStep.y * spacing),
            spacing * _constant * 3.5f);
        ofFill();
        ofDrawCircle(startingX + (avgCentroid.x * spacing),
            startingY + (avgCentroid.y * spacing),
            spacing * _constant);
        ofSetColor(ofColor::black);
        ofDrawCircle(startingX + (avgCentroid.x * spacing),
            startingY + (avgCentroid.y * spacing),
            spacing / 3);
        ofSetColor(ofColor::red);
        ofDrawCircle(startingX + (currentCentroid.x * spacing),
            startingY + (currentCentroid.y * spacing),
            spacing / 8);
        ofDrawCircle(startingX + (previousCentroid.x * spacing),
            startingY + (currentCentroid.y * spacing),
            spacing / 8);
        ofPolyline speedLine;
        speedLine.addVertex(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing);
        speedLine.addVertex(startingX + previousCentroid.x * spacing, startingY + previousCentroid.y * spacing);
        speedLine.draw();
    }
    ofPopStyle();
}

void ofApp::drawGraph(float sX, float sY, float avgX ,float avgY,int sN, float speed ) {
    ofPoint xpt, ypt;
    ofPoint avx, avy;
    ofPoint npt;
    ofPoint spt,smpt;
    xpt.set(xPos, ofMap(sX, -400, 400, ofGetHeight(), 0, true), 2);
    avx.set(xPos, ofMap(avgX, -400, 400, ofGetHeight(), 0, true), 2);
    ypt.set(xPos, ofMap(sY, -400, 400, ofGetHeight(), 0, true), 2);
    avy.set(xPos, ofMap(avgY, -400, 400, ofGetHeight(), 0, true), 2);
    npt.set(xPos, ofMap(sN, 0, sensorNumber, ofGetHeight(), 0, true), 2);
    spt.set(xPos, ofMap(theSpeed, 0,20, ofGetHeight(), 0, true), 2);
    smpt.set(xPos, ofMap(speedmul, 0, 9, ofGetHeight(), 0, true), 2);
    lineX.addVertex(xpt);
    lineAvgX.addVertex(avx);
    lineY.addVertex(ypt);
    lineAvgY.addVertex(avy);
    lineN.addVertex(npt);
    lineSpeed.addVertex(spt);
    lineSpeedMul.addVertex(smpt);

    int alphaX = 255, alphaY = 255;
    if (switcher) alphaY = 127;
    else alphaX = 127;

    ofSetColor(255, 100, 100,alphaX);
    if (showX) lineX.draw();
    int txtXpos = 190;
    ofDrawBitmapString("X:   " + ofToString(sX), txtXpos, 15);
    ofDrawBitmapString("freqX:   " + ofToString(freqX), txtXpos, 25);
    ofSetColor(155, 100, 100, alphaX);
    if (showAvg) lineAvgX.draw();
    ofSetColor(150, 150, 255, alphaY);
    if (showY)lineY.draw();
    ofDrawBitmapString("Y:   " + ofToString(sY), txtXpos, 35);
    ofDrawBitmapString("freqY:   " + ofToString(freqY), txtXpos, 45);
    ofSetColor(100, 100, 155, alphaY);
    if (showAvg) lineAvgY.draw();
    ofSetColor(100, 255, 100);
    if (showNas) lineN.draw();
    ofDrawBitmapString("N: " + ofToString(sN), txtXpos, 55);
    ofSetColor(ofColor::white);
    ofDrawBitmapString("The Speed: " + ofToString(theSpeed), txtXpos, 65);
    ofDrawBitmapString("SpeedMul: " + ofToString(speedmul), txtXpos, 75);
    if (showSpeed) lineSpeed.draw();
    ofSetColor(ofColor::yellow);
    if (showSpeedMul) lineSpeedMul.draw();


    xPos++;
    if (xPos > ofGetWidth()) {
        xPos = 0;
        lineX.clear();
        lineAvgX.clear();
        lineY.clear();
        lineAvgY.clear();
        lineN.clear();
        lineSpeed.clear();
        lineSpeedMul.clear();
        ofBackground(col[0]);
    }

}

void ofApp::setupDevices() {
    // SERIAL COM 
    serial.setup(comPort, baud);
    if (_debug) std::cout << serial.available() << endl;
    if (serial.available() >= 0) {
        serial.writeByte('A');
    }
}

void ofApp::tryFirstConnection() {
    if (ofGetElapsedTimeMillis() - timeOut > 100) {
        int inByte = serial.readByte();

        if (inByte == '1' || ofGetElapsedTimeMillis() > 2000) {
            serial.writeByte('1');
            firstContact = true;
        }
        timeOut = ofGetElapsedTimeMillis();
    }
    
}

ofVec2f ofApp::calculateCentroid() {
    // ORIENTATION ////////
    int maxVectorSize = 64;
    // Selecting the high pressure points
    //int t = 30;
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > T2) {
                points.push_back(ofVec2f(i, j));
                if (points.size() > maxVectorSize) points.pop_front();
            }
        }
    }
    
    ofVec2f average;
    for (int i = 0; i < points.size(); i++) {
        average += points[i];
    }
    average /= points.size();
    return average;
}

bool ofApp::calculateOrientation() {
    //if (sN < 30) return false;
   // ORIENTATION ////////
    int maxVectorSize = 64;
    // Selecting the high pressure points
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > T2) {
                _points.push_back(ofVec2f(i, j));
                if (_points.size() > maxVectorSize) _points.pop_front();
            }
        }
    }
    // Group Points that are close together
    int tt = 3;
    if (_points.size() < 2) return false;
    for (int i = 0; i < _points.size() - 1; i += 2) {
        if (_points[i].distance(_points[i + 1]) < tt) {
            blobs.push_back(_points[i]);
            blobs.push_back(_points[i + 1]);
            if (blobs.size() > maxVectorSize) blobs.pop_front();

        }
    }
    // Group the groups until there is only two groups.
    int timeToLeave = 0;
    while (blobs.size() > 4 || timeToLeave > 200) {
        for (int i = 0; i < blobs.size() - 1; i += 2) {
            if (blobs[i].distance(blobs[i + 1]) < tt) {
                blobs.push_back((blobs[i] + blobs[i + 1]) * 0.5f);
            }
            blobs.pop_front();
            blobs.pop_front();
            timeToLeave++;
        }
    }
    if (blobs.size() == 4) {
        if (blobs[0].distance(blobs[1]) > tt*4) {
            //_centroid1_x.pushToWindow(blobs[0].x);
            //_centroid1_y.pushToWindow(blobs[0].y);
            //_centroid2_x.pushToWindow(blobs[1].x);
            //_centroid2_y.pushToWindow(blobs[1].y);
            _centroid1 = (blobs[0] + blobs[1]) * 0.5f;
            _centroid2 = (blobs[2] + blobs[3]) * 0.5f;
            return true;
        }
        return false;
    }
    

    //if (blobs.size() >= 2) {
    //    if (blobs[0].distance(blobs[1]) > 2) {
    //        blob.push_back(blobs[0]);
    //        blob.push_back(blobs[1]);
    //    }
    //}
}

void ofApp::OpenCV() {
    ofPixels pixel;
    pixel.setFromPixels(readingArray, 16, 16, OF_IMAGE_GRAYSCALE);
    gray1 = pixel;
    if (gray2.bAllocated) {
        Mat img1 = cvarrToMat(gray1.getCvImage());
        Mat img2 = cvarrToMat(gray2.getCvImage());
        Mat flow;                        //Image for flow
        //Computing optical flow (visit https://goo.gl/jm1Vfr for explanation of parameters)
        calcOpticalFlowFarneback(img1, img2, flow, 0.5, 1, 3, 5, 5, 1.1, 0);
        //Split flow into separate images
        vector<Mat> flowPlanes;
        split(flow, flowPlanes);
        //Copy float planes to ofxCv images flowX and flowY
        //we call this to convert back from native openCV to ofxOpenCV data types
        IplImage iplX(flowPlanes[0]);
        //cvConvert(flowX, iplX);
        flowX = &iplX;
        IplImage iplY(flowPlanes[1]);
        flowY = &iplY;
    }
}

void ofApp::Speed() {
    

    /********************* STEP DETECTION *********************/
    // if stepDetected wait for current and previous to be bigger than Threshold
    if (stepDetected) {
        elapsedTime = constant;
        avgCentroid = currentCentroid;
        float Threshold = 3.5f * _constant;
        if (distance(currentCentroid, previousStep) < Threshold) {
            if (_debug) std::cout << "4 - prevening new comparisons" << endl;
            wait = true;
        }
        else {
            if (_debug) std::cout << "6 - threshold reached" << endl;
            wait = false;
            stepDetected = false;  
        }
        /********************* SPEED CALCULATION *********************/
        if (abs(ofGetElapsedTimeMillis() - _Timer1) < 2000) {
            if (_debug) std::cout << "5 - calculating speed" << endl;
            // calculate distance between centroids
            rateOfChange = abs(currentCentroid - previousCentroid) / constant;
            previousCentroid = currentCentroid;
            // average rate of change and speed
            avgRateOfChange.push_back(rateOfChange);
            if (avgRateOfChange.size() > 9) avgRateOfChange.pop_front();
            vec2 avgR = vec2(0, 0);
            for (int i = 0; i < avgRateOfChange.size(); i++) avgR += avgRateOfChange[i];
            avgR /= avgRateOfChange.size();
            avgSpeed.push_back(avgR);
            if (avgSpeed.size() > 5) avgSpeed.pop_front();
            vec2 avgS = vec2(0, 0);
            for (int i = 0; i < avgSpeed.size(); i++) avgS += avgSpeed[i];
            avgS /= avgSpeed.size();
            // final speed
            finalSpeed = avgS / stepDistance;
            theSpeed = MAX(finalSpeed.x,finalSpeed.y);
            if (theSpeed < 2 || abs(_Timer2 - _Timer1) > 1200) theSpeed = 0;
            theSpeed = finalS.process(theSpeed);
            if (theSpeed < 0.01f) theSpeed = 0;
        }
        
    } 
    // comparing currentCentroid and avgCentroid
    if (!wait) { 
        if (_debug) std::cout << "1 - comparing currentCentroid and avgCentroid" << endl;
        if (distance(currentCentroid, avgCentroid) < _constant) {
            avgCentroid = ((avgCentroid * elapsedTime) + currentCentroid * constant) / (elapsedTime + constant);
            elapsedTime += constant; 
            if (_debug) std::cout << "1 - elapsedTime: " << elapsedTime << "s" <<  endl;
        }
        else {
            elapsedTime = constant;
            avgCentroid = currentCentroid;
        }
    }
    // step was deteced
    float StepTimeThreshold = 0.075f;
    if (elapsedTime >= StepTimeThreshold) {
        if (_debug) std::cout << "2 - step was deteced" << endl;
        _stepDist = distance(currentCentroid, previousStep);
        stepDetected = true;
        _Timer2 = _Timer1;
        _Timer1 = ofGetElapsedTimeMillis();
        if (_stepDist > 4)stepDistance = _stepDist;
        ofSetColor(ofColor::gold);
        ofDrawRectangle(0, 0, 300, 300);
        centroid1 = currentCentroid;
        centroid2 = previousStep;
        teta.pushToWindow(ofRadToDeg(atan2(centroid1.y - centroid2.y, centroid1.x - centroid2.x)));
        previousStep = currentCentroid;      
    }
    


}

void ofApp::Processing() {
    // SPEED /////////////
    // add values to array
    cogArrayX.pushToWindow(X);
    cogArrayY.pushToWindow(Y);
    nasArray.pushToWindow(N);

    // average over 20 readings (change readings in rapidStream.cpp)
    sX = cogArrayX.mean();
    sY = cogArrayY.mean();
    sN = nasArray.mean();

    // detects jump
    if (_debug) std::cout << "NaS, highest value is jump: " << abs(nasArray.maxVelocity()) << endl;
    if (abs(nasArray.maxVelocity()) >= 20) jump = true;
    else jump = false;

    // this timer smooths the zero crossing lines for X and Y
    if (ofGetElapsedTimeMillis() - timer2 > T3) {
        crossingArrayX.pushToWindow(sX);
        crossingArrayY.pushToWindow(sY);
        timer2 = ofGetElapsedTimeMillis();
    }

    // stores the zero crossing value
    avX = crossingArrayX.mean();
    avY = crossingArrayY.mean();

    // zero cross algorithm based on variable line
    float varT5 = ofMap(speedmul, 1, 6, T5, T5 - 10);
    int zCrossX = abs(sX - avX);
    int zCrossY = abs(sY - avY);
    if (zCrossX > varT5 || zCrossY > varT5) {
        // debounce step
        if (ofGetElapsedTimeMillis() - timer3 > T4) {
            if (zCrossX > zCrossY) {
                switcher = true;
                if (sX > avX && !changedX) {
                    freqX++;
                    //centroid1 = calculateCentroid();
                    changedX = true;
                    crossed = true;
                    stepTimer1 = ofGetElapsedTimeMillis();
                    timer3 = ofGetElapsedTimeMillis();

                }
                else if (sX < avX && changedX) {
                    freqX++;
                    //centroid2 = calculateCentroid();
                    changedX = false;
                    crossed = true;
                    stepTimer2 = ofGetElapsedTimeMillis();
                    timer3 = ofGetElapsedTimeMillis();
                }
            }
            else {
                switcher = false;
                if (sY > avY && !changedY) {
                    freqY++;
                    //centroid1 = calculateCentroid();
                    changedY = true;
                    crossed = true;
                    stepTimer1 = ofGetElapsedTimeMillis();
                    timer3 = ofGetElapsedTimeMillis();
                }
                else if (sY < avY && changedY) {
                    freqY++;
                    //centroid2 = calculateCentroid();
                    changedY = false;
                    crossed = true;
                    stepTimer2 = ofGetElapsedTimeMillis();
                    timer3 = ofGetElapsedTimeMillis();
                }
            }
            //teta.pushToWindow(ofRadToDeg(atan2(centroid1.y - centroid2.y, centroid1.x - centroid2.x)));
        }
    }
    else crossed = false;

    int minDeltaStep = 600;
    deltaStep = abs(stepTimer1 - stepTimer2);
    //if (deltaStep > 200) smoothStep1.pushToWindow(deltaStep);
    if (deltaStep < 120) deltaStep = minDeltaStep;
    float process = smooth.process(deltaStep);
    speedmul = ofMap(process, 150, minDeltaStep, 6, 1, true);

}

void ofApp::oscSend() {
    // OSC send
    ofxOscMessage m;
    m.setAddress("/walkingpad");
    m.addFloatArg(theSpeed);
    //m.addFloatArg(speedmul);
    //m.addBoolArg(jump);
    m.addFloatArg(teta.mean());
    //m.addFloatArg(sX);
    //m.addFloatArg(sY);
    //m.addIntArg(sN);
    osc.sendMessage(m, false);
}

void ofApp::csvRecord() {
    // Log values in CSV file
    if (recording) {
        ofxCsvRow row;
        row.setFloat(0, sX); // CoG X
        row.setFloat(1, sY); // CoG Y
        row.setInt(2, sN);   // NaS mean
        row.setBool(3, jump);
        csv.addRow(row);
    }
}

void ofApp::keyPressed(int key) {
    if (simulating) {
        if (key == OF_KEY_LEFT) {
            leftDown = true;
        }
        if (key == OF_KEY_RIGHT) {
            rightDown = true;
        }
        if (key == OF_KEY_UP) {
            upDown = true;
        }
        if (key == OF_KEY_DOWN) {
            downDown = true;
        }
    }
}

void ofApp::keyReleased(int key) {

    if (key == 32) {
        if (toggle && recording) {
            // CSV
            csv.createFile(filename);
            bool exitAfterSave = false;
            exitAfterSave = csv.save(filename);
            if (exitAfterSave) ofExit();
        }
        toggle = !toggle;
        recording = toggle;
    }
    if (key == '0') _calibrateNow = true;
    if (simulating) {
        if (key == OF_KEY_LEFT) {
            leftDown = false;
        }
        if (key == OF_KEY_RIGHT) {
            rightDown = false;
        }
        if (key == OF_KEY_UP) {
            upDown = false;
        }
        if (key == OF_KEY_DOWN) {
            downDown = false;
        }
    }

}