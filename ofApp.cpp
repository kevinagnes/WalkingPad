#include "ofApp.h"

//------------------    --------------------------------------------
void ofApp::setup(){
    dontDraw = false;
    ofBackground(col[0]);
    ofSetBackgroundAuto(true);
    ofSetFrameRate(60);

    ofxGuiSetDefaultHeight(20);
    ofxGuiSetDefaultWidth(150); 
    parameters.setName("Thres & Delay");
    parameters.add(T1.set("Clamp: ", 180, 0, 400));
    parameters.add(T2.set("Blob Thresh: ", 10, 1, 255));
    parameters.add(T3.set("avgZeroCrossing: ", 60, 0, 500));
    parameters.add(T4.set("stepDebounce: ", 100, 0, 500));
    //parameters.add(T5.set("stepDebounce: ", 500, 0, 1000));
    parameters.add(bigMatrix.set("show Big Matrix", false));
    parameters.add(showX.set("X CoG", true));
    parameters.add(showY.set("Y CoG", true));
    parameters.add(showSpeed.set("Speed",false));
    parameters.add(showSpeedMul.set("SpeedMultiplier", false));
    parameters.add(showAvg.set("ZeroCrossingLine", false));
    parameters.add(showNas.set("NumberOfActiveSensors", false));
    gui.setup(parameters);
    gui.setPosition(ofGetWidth() - gui.getWidth(), 0);

    set.setName("Serial Com: ");
    par.add(baud.set("BAUD ", baud, baudTypes[0], baudTypes[1]));
    par.add(baudSelect.set("BAUD SET", false));
    par.add(COMPORT.set("COM ", 10, 0, 15));
    par.add(simulating.set("Keyboard Simulating: ", false));
    par.add(confirm.set("OK", false));
    set.setup(par);

    
}

//--------------------------------------------------------------
void ofApp::update(){

    
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

    // CSV
    csv.createFile(filename);

    // ignore when simulating
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
                char byteReturned = serial.readByte();
                switch (counter) {
                case 64: // CoG X
                    reading[0] = byteReturned;
                    break;
                case 65: // CoG Y
                    reading[1] = byteReturned;
                    break;
                case 66: // NaS
                    reading[2] = byteReturned;
                    break;
                default: // Pressure-sensitive MATRIX values
                    readingArray[counter] = byteReturned;
                    break;
                }
                counter++;
                if (counter > 66) {
                    //cout << ofGetElapsedTimeMillis() - timeFromPreviousCall << endl;
                    timeFromPreviousCall = ofGetElapsedTimeMillis();
                    serial.writeByte('A');
                    counter = 0;
                }
            }
        }
        // multiply by 1.20f to compensate the rectangular shape
        X = 1.25f * (reading[0] * 10 - 400);
        Y = 1     * (reading[1] * 10 - 400);
        N = reading[2];

        /*// ORIENTATION ////////
    int maxVectorSize = 32;
    // Selecting the high pressure points
    int t = 45;
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > t) {
                points.push_back(ofVec2f(i, j));
                if (points.size() > maxVectorSize) points.pop_front();
            }
        }
    }
    // Group Points that are close together
    int tt = 2;
    for (int i = 0; i < points.size() - 1; i += 2) {
        if (points.size() < 1) return;
        if (points[i].distance(points[i + 1]) < tt) {
            blobs.push_back(points[i]);
            blobs.push_back(points[i + 1]);
            if (blobs.size() > maxVectorSize) blobs.pop_front();

        }
    }
    // Group the groups until there is only two groups.
    while (blobs.size() > 2) {
        for (int i = 0; i < blobs.size() - 1; i += 2) {
            if (blobs[i].distance(blobs[i + 1]) < tt) {
                blobs.push_back((blobs[i] + blobs[i + 1]) * 0.5f);
            }
            blobs.pop_front();
            blobs.pop_front();
        }
    }
    if (blobs.size() == 2) {
        //cout << blobs.size() << endl;
        for (int i = 0; i < blobs.size(); i++) {
            blob.push_back(blobs[i]);
        }
    }*/
      
        //if (blobs.size() >= 2) {
        //    if (blobs[0].distance(blobs[1]) > 2) {
        //        blob.push_back(blobs[0]);
        //        blob.push_back(blobs[1]);
        //    }
        //}
    }
    // Keyboard Simulation
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
    if (abs(nasArray.maxVelocity()) >=12) jump = true;
    else jump = false;

    // max and min velocity
    vec2 max = vec2(cogArrayX.maxVelocity(), cogArrayY.maxVelocity());
    vec2 min = vec2(cogArrayX.minVelocity(), cogArrayY.minVelocity());

    // pushing values to velArray
    velArrayX.pushToWindow(max.x - min.x);
    velArrayY.pushToWindow(max.y - min.y);

    // average velocity (smoothing the velocity) 
    // smoothing, prevents it from reaching zero 
    // when in the middle of a cycle
    float cX = velArrayX.mean();
    float cY = velArrayY.mean();

    // this timer smooths the zero crossing lines for X and Y
    if (ofGetElapsedTimeMillis() - timer2 > T3) {
        crossingArrayX.pushToWindow(sX);
        crossingArrayY.pushToWindow(sY);
        timer2 = ofGetElapsedTimeMillis();  
    }

    // stores the zero crossing value
    avX = crossingArrayX.mean();
    avY = crossingArrayY.mean();

    // speed = highest value
    speed = MAX(cX, cY);

    // applies a low pass filter in the speed
    speed = lpf.process(speed);

    //

    // remove a variable value from speed
    // this helps low stopping latency
    // and smooths low-frequency continuous speed
    float varT1 = ofMap(speedmul, 1, 3, T1 - 100, T1 + 100);
    speed -= varT1;
    

    // Clamp negative speed to 0
    if (speed < 0) speed = 0;

    // zero cross detection based on variable line
    // stores the number of steps
    if (speed > 0) { 
        // debounce step
        if (ofGetElapsedTimeMillis() - timer3 > T4) {
            if (cX > cY) {
                if (sX > avX && !changedX) {
                    freqX++;
                    centroid1 = calculateCentroid();
                    changedX = true;
                    stepTimer1 = ofGetElapsedTimeMillis();
                    deltaStep += abs(stepTimer1 - stepTimer2);
                }
                else if (sX < avX && changedX) {
                    freqX++;
                    centroid2 = calculateCentroid();
                    changedX = false;
                    stepTimer2 = ofGetElapsedTimeMillis();
                    deltaStep += abs(stepTimer1 - stepTimer2);
                }   
            }
            else {
                if (sY > avY && !changedY) {
                    freqY++;
                    //centroid1.average(blob.data(), blob.size());
                    centroid1 = calculateCentroid();
                    changedY = true;
                    stepTimer1 = ofGetElapsedTimeMillis();
                    deltaStep += abs(stepTimer2 - stepTimer1);
                }
                else if (sY < avY && changedY) {
                    freqY++;
                    centroid2 = calculateCentroid();
                    changedY = false;
                    stepTimer2 = ofGetElapsedTimeMillis();
                    deltaStep += abs(stepTimer2 - stepTimer1);
                }   
            }
            //deltaStep = abs (stepTimer2 - stepTimer1);   
            
            
            
            //if (deltaStep > 100) smoothStep1.pushToWindow(deltaStep);
            teta.pushToWindow(ofRadToDeg(atan2(centroid1.y - centroid2.y, centroid1.x - centroid2.x)));
            smoothStep1.pushToWindow(1);
            timer3 = ofGetElapsedTimeMillis();


        }
        //smoothStep2.pushToWindow(smoothStep1.mean()); 
        //speedmul = smoothStep2.mean();
        //speedmul = smoothStep1.mean();
    }
    else {
        smoothStep1.pushToWindow(1);
        //smoothStep1.pushToWindow(1000);
        //smoothStep2.pushToWindow(3000);
        //speedmul = 1000;
        
    }
    if (deltaStep != 0) smoothStep1.pushToWindow(ofClamp(1000 * (freqX + freqY) / deltaStep, 1, 6));
    //else smoothStep1.pushToWindow(1);
    speedmul = smoothStep1.mean();
    //if (deltaStep != 0) cout << 1000 * (freqX + freqY) / deltaStep << endl;
    deltaStep = 0;
    freqX = 0;
    freqY = 0;
    
    //speedmul = ofMap(speedmul, 100, 1000, 6, 1, true);

    // OSC send
    ofxOscMessage m;
    m.setAddress("/xyn");
    m.addFloatArg(sX);
    m.addFloatArg(sY);
    m.addIntArg(sN);
    m.addFloatArg(speed);
    m.addIntArg(speedmul);
    m.addBoolArg(jump);
    m.addFloatArg(teta.mean());
    osc.sendMessage(m, false);

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

//--------------------------------------------------------------
void ofApp::draw(){

    if (!setupCompleted) return set.draw();

    if (dontDraw) return;

    // draw grid
    ofPushStyle();
    for (int y = 0; y < ofGetHeight(); y += 10) {
        ofSetColor(80,100);
        ofDrawLine(0, y, ofGetWidth(), y);
    }

    // draw recording guide
    ofPopStyle();
    ofPushStyle();
    ofSetColor(col[toggle]);
    ofSetLineWidth(10);
    ofNoFill();
    ofDrawRectangle(0, 0, ofGetWidth() - 3, ofGetHeight() - 3);
    ofPopStyle();

    // draw the graph
    drawGraph(sX, sY, avX, avY, sN, speed);

    // draw if jump
    if (jump) ofDrawCircle(ofGetWidth(), ofGetHeight(), 50);

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

    //draw Matrix
    if (bigMatrix) drawMatrix(ofGetWidth()/2- ofGetHeight() / 2, ofGetHeight()/2- ofGetHeight()/2, ofGetHeight());
    else drawMatrix(20, 10, rectSize);

    // draw GUI
    gui.draw();

}

void ofApp::drawMatrix(int startingX, int startingY, int size) {
    ofPushStyle();
    ofPushMatrix();
    calculateCentroid();
    int spacing = size / sensorsBase;
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j ++) {
            ofSetColor(255);
            ofPushMatrix();
            float ReadingMapped = ofMap(readingArray[(i * sensorsBase) + j], 0, 80, 0, 255, true);
            ofTranslate(startingX+(i*spacing),startingY+(j*spacing));
            
            ofNoFill();
            ofDrawRectangle(0, 0, spacing, spacing);
            ofColor pressureColor = ofColor::red;
            pressureColor.setHsb(ReadingMapped, 255, ReadingMapped);
            ofSetColor(pressureColor);
            ofFill();
            ofDrawRectangle(0, 0, spacing, spacing);
            //ofSetColor(ofColor::red);
            //ofDrawBitmapString(ReadingMapped, -10, 0);
            ofPopMatrix();
        }
    }
    ofSetColor(255, 255, 0);
    for (int i = 0; i < points.size(); i++) {
        ofDrawCircle(startingX + (points[i].x * spacing),
                     startingY + (points[i].y * spacing),
                     spacing/4);
    }
    ofSetColor(255, 0, 0);
    ofDrawCircle(startingX + (centroid1.x * spacing),
                 startingY + (centroid1.y * spacing),
                 spacing / 4);
    ofSetColor(0, 0, 255);
    ofDrawCircle(startingX + (centroid2.x * spacing),
        startingY + (centroid2.y * spacing),
        spacing / 4);
    ofSetColor(0,255,0);
    
    ofSetLineWidth(3);
    ofPolyline stepsLine;
    float hipotenuse = abs(centroid1.distance(centroid2));
    if (hipotenuse > 3) {
        stepsLine.addVertex(startingX + centroid1.x * spacing, startingY + centroid1.y * spacing);
        stepsLine.addVertex(startingX + centroid2.x * spacing, startingY + centroid2.y * spacing);
        
        //cout << teta.mean() << endl;
    }
    stepsLine.draw();

    //points.clear();
    //blobs.clear();
    //blob.clear();
    //float YawRotation = steps.get
    //ofDrawLine(step1, step2);
    ofPopMatrix();
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
    npt.set(xPos, ofMap(sN, 0, 64, ofGetHeight(), 0, true), 2);
    spt.set(xPos, ofMap(speed, 0,800, ofGetHeight(), 0, true), 2);
    smpt.set(xPos, ofMap(speedmul, 1, 8, ofGetHeight(), 0, true), 2);
    lineX.addVertex(xpt);
    lineAvgX.addVertex(avx);
    lineY.addVertex(ypt);
    lineAvgY.addVertex(avy);
    lineN.addVertex(npt);
    lineSpeed.addVertex(spt);
    lineSpeedMul.addVertex(smpt);
    //lineX.getSmoothed(4, 1);
    //lineY.getSmoothed(4, 1);
    //lineN.getSmoothed(4, 1);

    //ofSetColor(col[0]);
    //ofDrawRectangle(0, 0, ofGetWidth(), 65);

    ofSetColor(255, 100, 100);
    if (showX) lineX.draw();
    int txtXpos = 190;
    ofDrawBitmapString("X:   " + ofToString(sX), txtXpos, 15);
    ofDrawBitmapString("freqX:   " + ofToString(freqX), txtXpos, 25);
    ofSetColor(155, 50, 50);
    if (showAvg) lineAvgX.draw();
    ofSetColor(150, 150, 255);
    if (showY)lineY.draw();
    ofDrawBitmapString("Y:   " + ofToString(sY), txtXpos, 35);
    ofDrawBitmapString("freqY:   " + ofToString(freqY), txtXpos, 45);
    ofSetColor(50, 50, 155);
    if (showAvg) lineAvgY.draw();
    ofSetColor(100, 255, 100);
    if (showNas) lineN.draw();
    ofDrawBitmapString("N: " + ofToString(sN), txtXpos, 55);
    ofSetColor(ofColor::white);
    ofDrawBitmapString("Speed: " + ofToString(speed), txtXpos, 65);
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
            bool exitAfterSave = false;
            exitAfterSave = csv.save(filename);
            if (exitAfterSave) ofExit();
        }
        toggle = !toggle;
        recording = toggle;
    }
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

void ofApp::setupDevices() {
    // SERIAL COM 
    //serial.listDevices();
    //vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
    serial.setup(comPort, baud);
    cout << serial.available() << endl;
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
    int maxVectorSize = 16;
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
/*
ofVec2f ofApp::calculateOrientation() {
    // ORIENTATION ////////
    int maxVectorSize = 32;
    ofVec2f zero = ofVec2f(4, 4);
    int tt = 4;
    // Selecting the high pressure points
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > T2) {
                blobs.push_back(ofVec2f(i, j));
                if (blobs.size() > maxVectorSize) blobs.pop_front();
            }
        }
    }
    vector <vector<ofVec2f>> feet;
    vector<ofVec2f> foot1, foot2;
    if (blobs.size() < 2) return zero;

    for (int i = 0; i < blobs.size() - 1; i += 2) {
        if (blobs[i].distance(blobs[i + 1]) < tt) {
            foot1.push_back((blobs[i]+blobs[i+1])*0.5f);
        }
        else {
            foot2.push_back(blobs[i]);
            foot2.push_back(blobs[i + 1]);
        }
        blobs.pop_front();
        blobs.pop_front();
    }
    for (int i = 0; i < blobs.size(); i++) {
        for (int j = 1; j < blobs.size() - 1; j++) {
            if (blobs[i].distance(blobs[j]) < tt) {
                foot1.push_back((blobs[i] + blobs[j]) * 0.5f);
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


*/