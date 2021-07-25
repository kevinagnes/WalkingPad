#include "ofApp.h"
#include "opencv2/opencv.hpp"

//------------------    --------------------------------------------
void ofApp::setup() {
    ofBackground(col[0]);
    ofSetBackgroundAuto(true);
    ofSetFrameRate(60);
    // Low-pass filter for movoementDetection and finalSpeed
    lpf.initialize(60, 3);
    finalS.initialize(60, 2);
    LowPass.initialize(60, 4);
    LowPass2.initialize(60, 2);
    lpfX.initialize(60, 3); lpfY.initialize(60, 3); lpfN.initialize(60, 3);
    text.load("Sono-Bold.ttf",64);

#pragma region GUI_&_InitialVariables
    ofxGuiSetDefaultHeight(20);
    ofxGuiSetDefaultWidth(150);
    parameters.setName("WalkingPad");
    parameters.add(T1.set("OpticalFlow: ", 0.1f, 0, 1));
    parameters.add(T2.set("Blob Thresh: ", 50, 1, 255));
    //parameters.add(T3.set("avgZeroCrossing: ", 20, 0, 500));
    //parameters.add(T4.set("stepDebounce: ", 150, 0, 500));
    //parameters.add(T5.set("ZeroCrossing Thresh: ", 25, 0, 400));
    parameters.add(T5.set("blobDistance: ", 4, 1, 30));
    parameters.add(T4.set("turningDelay: ", 500, 0, 1000));
    parameters.add(_constant.set("CONSTANT: ", 1, 0, 3));
    parameters.add(_debug.set("showDebug", false));
    parameters.add(_debugOpticalFlow.set("showOpticalFlow", false));
    parameters.add(_debugNewMethod.set("showCentroid", false));
    parameters.add(bigMatrix.set("show Big Matrix", false));
    parameters.add(showPainting.set("showPainting", false));
    parameters.add(showX.set("X CoG", true));
    parameters.add(showY.set("Y CoG", true));
    parameters.add(showSpeed.set("Speed", true));
    parameters.add(showSpeedMul.set("SpeedMultiplier", true));
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

    currentCentroid = initialCoG;
    dontDraw = false;
    calculatedFlow = false;
#pragma endregion 
    
}
//--------------------------------------------------------------
void ofApp::update() {}
//--------------------------------------------------------------
void ofApp::draw() {

    currentTime = ofGetElapsedTimeMillis();

#pragma region InitialSetup
    // setup COM ports or keyboard simulation
    if (!setupCompleted) {
        if (skipSetup) {
            setupDevices("COM12", 57600);
            setupCompleted = true;
        }
        else {
            set.draw();
            baud = baudTypes[baudSelect];
            if (confirm) {
                comPort += COMPORT.toString();
                if (!simulating) setupDevices(comPort, baud);
                setupCompleted = true;
            }
            return;
        }
    }

    // OSC
    osc.setup("localhost", 6969);

    if (!setupCompleted) return set.draw();

    if (dontDraw) return;
#pragma endregion

#pragma region GetValues
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
                        if (_debug) std::cout << "Time from previous reading: " << currentTime - timeFromPreviousCall << " ms" << endl;
                        timeFromPreviousCall = currentTime;
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
        if (_debugOpticalFlow) OpenCV(); 

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
        int smoothingCentroid = 10;
        if (tempCurrentCentroid.size() > smoothingCentroid - 1) tempCurrentCentroid.pop_back();
        if (total != 0) tempCurrentCentroid.push_front(sum / total);
        vec2 curr;
        vec2 HighestCentroid;
        for (int i = 0; i < tempCurrentCentroid.size(); i++) {
            if (length(tempCurrentCentroid[i]) > length(HighestCentroid))
                HighestCentroid = tempCurrentCentroid[i];
            curr += tempCurrentCentroid[i];
        }
        if (tempCurrentCentroid.size() == smoothingCentroid)
            currentCentroid = (curr - HighestCentroid) / (tempCurrentCentroid.size() - 1);


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
            //X = 0; Y = 0; N = 26;
            currentCentroid = vec2(lpfX.process(8), lpfY.process(8)); N = lpfN.process(26);
        }
        else {
            if (leftDown) {
               //currentCentroid.x = 4; currentCentroid.y = 8; N = 13;
                currentCentroid = vec2(lpfX.process(4),lpfY.process(8)); N = lpfN.process(13);
            }
            else if (rightDown) {
                //currentCentroid.x = 12; currentCentroid.y = 8; N = 13;
                currentCentroid = vec2(lpfX.process(12), lpfY.process(8)); N = lpfN.process(13);
            }
        }
        if (upDown && downDown) {
            //currentCentroid.x = 8; currentCentroid.y = 8; N = 26;
            currentCentroid = vec2(lpfX.process(8), lpfY.process(8)); N = lpfN.process(26);
        }
        else {
            if (upDown) {
                //currentCentroid.x = 8; currentCentroid.y = 4; N = 13;
                currentCentroid = vec2(lpfX.process(8), lpfY.process(4)); N = lpfN.process(13);
            }
            else if (downDown) {
                //currentCentroid.x = 8; currentCentroid.y = 12; N = 13;
                currentCentroid = vec2(lpfX.process(8), lpfY.process(12)); N = lpfN.process(13);
            }
        }
        if (!upDown && !downDown && !leftDown && !rightDown) {
            //currentCentroid.x = 8; currentCentroid.y = 8;  N = 26;
            currentCentroid = vec2(lpfX.process(8), lpfY.process(8)); N = lpfN.process(26);
        }
        
        X = 1 * (currentCentroid.x * 50 - 400);
        Y = 1 * (currentCentroid.y * 50 - 400);
    }
#pragma endregion

#pragma region ProcessData
    //MovementDetector();
    MovementDetector2();
    //blobDetection();
    //SpeedCalculation();
    //FinalSpeedDecision();
    
    //calculateOrientation();
#pragma endregion

#pragma region SendData
    oscSend();
#pragma endregion

#pragma region DrawFunctions
    // draw grid
    ofPushStyle();
    for (float y = 0; y < ofGetHeight(); y += ofGetHeight() / 16) {
        ofSetColor(80, 100);
        ofDrawLine(0, y, ofGetWidth(), y);
    }
    ofPopStyle();

    // draw the graph
    //drawGraph(sX, sY, avX, avY, sN, speed);
    ofPushStyle();
    ofSetLineWidth(ofGetHeight()*0.002);
    drawGraph(X, Y, 0, 0, N, speed);
    ofPopStyle();

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
    if (_debugOpticalFlow && !simulating) {  
        gray1.rotate(90, gray1.getWidth() * 0.5f, gray1.getHeight() * 0.5f);
        gray1.mirror(true,false);
        gray1.draw(ofGetWidth() * .5f - ofGetHeight() * .5f, ofGetHeight() * .5f - ofGetHeight() * .5f, ofGetHeight(), ofGetHeight());
        contourFinder.draw(ofGetWidth() * .5f - ofGetHeight() * .5f, ofGetHeight() * .5f - ofGetHeight() * .5f, ofGetHeight(), ofGetHeight());
    }
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

    

    // draw blobs
    if (blobs.size() > 0) {
        for (int i = 0; i < blobs.size(); i++) {
            vec4 pos = blobs[i].draw(ofGetWidth() * .5f - ofGetHeight() * .5f, ofGetHeight() * .5f - ofGetHeight() * .5f, ofGetHeight() / sensorsBase);
            ofSetColor(ofColor::darkGreen);
            text.drawString(ofToString(blobs[i].id), (pos.r+pos.b)*.5f,(pos.g + pos.a) * .5f);
        }
    }

    // draw step winning
    ofSetColor(cXcYColor);
    ofDrawRectangle(0 + ofGetHeight() * 0.05f, ofGetHeight() * 0.5f - ofGetHeight() * 0.05f, ofGetHeight() * 0.1f, ofGetHeight() * 0.1f);

    // draw GUI
    string info = "[0] Calibrate \n[.] GUI \n[1] CoG \n[2] Speed \n[3] Matrix \n[4] Optical \n[5] Simulate";
    ofPushStyle();
    ofSetColor(ofColor::orange, 127);
    if (drawGui) gui.draw();
    else ofDrawBitmapString(info, vec2(ofGetWidth() - 110, ofGetHeight() -110));
    ofPopStyle();
#pragma endregion

    csvRecord();
}

#pragma region SpeedMethods
void ofApp::MovementDetector() {
    // SPEED /////////////
    // add values to array
    cogArrayX.pushToWindow(X);
    cogArrayY.pushToWindow(Y);
    nasArray.pushToWindow(N);

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

    if (cX > cY) xWin = true;
    else xWin = false;

    string txt = "max: " + ofToString(max) + '\n' +
        "min: " + ofToString(min) + '\n' +
        "cX: " + ofToString(cX) + '\n' +
        "cY: " + ofToString(cY) + '\n';
    text.drawString(txt, ofGetWidth() / 2, 100);

    // speed = highest value
    movementDetection = MAX(cX, cY) / 50;

    // applies a low pass filter in the speed
    //movementDetection = lpf.process(movementDetection);
    movementDetection -= 0.3f;
    // Clamp negative speed to 0
    if (movementDetection < 0) {
        movementDetection = 0;
        if (!stopped) {
            timer4 = currentTime;
            stopped = !stopped;
        }
    }

    /*
    * // average over 20 readings (change readings in rapidStream.cpp)
    sX = cogArrayX.mean();
    sY = cogArrayY.mean();
    sN = nasArray.mean();

    // detects jump
    if (_debug) std::cout << "NaS, highest value is jump: " << abs(nasArray.maxVelocity()) << endl;
    if (abs(nasArray.maxVelocity()) >= 20) jump = true;
    else jump = false;

    // this timer smooths the zero crossing lines for X and Y
    if (currentTime - timer2 > T3) {
        crossingArrayX.pushToWindow(sX);
        crossingArrayY.pushToWindow(sY);
        timer2 = currentTime;
    }

    // stores the zero crossing value
    avX = crossingArrayX.mean();
    avY = crossingArrayY.mean();

    // zero cross algorithm based on variable line
    float varT5 = ofMap(movementDetection, 1, 6, T5, T5 - 10);
    int zCrossX = abs(sX - avX);
    int zCrossY = abs(sY - avY);
    if (zCrossX > varT5 || zCrossY > varT5) {
        // debounce step
        if (currentTime - timer3 > T4) {
            if (zCrossX > zCrossY) {
                switcher = true;
                if (sX > avX && !changedX) {
                    freqX++;
                    //centroid1 = calculateCentroid();
                    changedX = true;
                    crossed = true;
                    stepTimer1 = currentTime;
                    timer3 = currentTime;

                }
                else if (sX < avX && changedX) {
                    freqX++;
                    //centroid2 = calculateCentroid();
                    changedX = false;
                    crossed = true;
                    stepTimer2 = currentTime;
                    timer3 = currentTime;
                }
            }
            else {
                switcher = false;
                if (sY > avY && !changedY) {
                    freqY++;
                    //centroid1 = calculateCentroid();
                    changedY = true;
                    crossed = true;
                    stepTimer1 = currentTime;
                    timer3 = currentTime;
                }
                else if (sY < avY && changedY) {
                    freqY++;
                    //centroid2 = calculateCentroid();
                    changedY = false;
                    crossed = true;
                    stepTimer2 = currentTime;
                    timer3 = currentTime;
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
    movementDetection = ofMap(process, 150, minDeltaStep, 6, 1, true);
    */
}

void ofApp::MovementDetector2() {

    
    // store current cog
    vec2 _curr = currentCentroid;
    
    // compare curr and prev
    float _dis = distance(_curr, _old) * 3.3f / 100; // convert unit to meters
    _avgDis.push_front(_dis);

    // get _avgDif - maxDif
    float avgDis = _dis;
    float disMax = 0;
    if (_avgDis.size() == disWindow) {
        for (int i = 0; i < disWindow; i++) {
            avgDis += _avgDis[i];
            disMax = MAX(disMax, _avgDis[i]);
        }
        avgDis -= disMax;
        avgDis /= disWindow-1;
    }

    // if (distance(curr,old) > T) -> state = moving
    // CoG in movement (FOOT OFF)
    if (_dis > 0.01f) {
        _timer2 = currentTime;
        shiftCentroid = true;
        _state = 1;
    }
    // CoG not in movement (FOOT MAX HEIGHT / DOUBLE SUPPORT)
    else {

        if (currentTime - _timer2 > timeToStop)  {
            _state = 0;
        }
        
    }


    switch (_state) {
    case 0:
        ofSetColor(ofColor::red);
        break;
    case 1:
        ofSetColor(ofColor::green);
        break;
    }

    ofDrawCircle(50, ofGetHeight() * 0.5f,25);
    
    // store velocity -> (curr-old)/constant  [constant = 1/60]
    _velocity.push_front(avgDis / constant);

    
    // state MOVING
    if (_state == 1) {
        
        // calculate average velocity - maxVel based on CoG displacement
        float maxVel = 0;
        if (_velocity.size() == velWindow) {
            for (int i = 0; i < velWindow; i++) {
                _s += _velocity[i];
                maxVel = MAX(maxVel, _velocity[i]);
            }
            _s -= maxVel;
            _s /= velWindow-1;
            // prevents input errors
            if (avgDis > 0.04f) _s /= avgDis;
        }
    }
    // state NOT MOVING
    else {
        LowPass2.process(0);
        _s = 0;
        _velocity.clear();
        _avgDis.clear();
    }

    // is moving
    if (_s > 0) {
        _s -= 0.1f;
        _timer3 = currentTime;
    }
    
    if (currentTime - _timer2 <= timeToStop) {
        _s = MAX(_s, 0.4f);
    }
    if (currentTime - _timer3 > 150) {
        LowPass2.process(0);
        __fs = 0;
    }

    __fs = LowPass2.process(_s);
    _fs = LowPass.process(_s);


    if (_fs < 0.001f) {
        _fs = 0;
        _s = 0;
        LowPass.process(0);
        //_state = 0;
        if (!stopped) {
            timer4 = currentTime;
            stopped = !stopped;
        }
    }
    
    // limit the size of arrays
    if (_velocity.size() >= velWindow) _velocity.pop_back();
    if (_avgDis.size()   >= disWindow) _avgDis.pop_back();

    // debug
    string _velFS = ofToString(_fs, 2);
    text.drawString(_velFS, ofGetWidth() * 0.5f - text.stringWidth(_velFS)*0.5f, ofGetHeight() * 0.5f + text.stringHeight(_velFS) * 0.5f);
    //text.drawString(ofToString(_dis, 4), ofGetWidth() / 2, 100);
    //text.drawString(ofToString(movementDetection, 4), ofGetWidth() / 2, 180);
    //text.drawString(ofToString(_velocity.size()), ofGetWidth() / 2, 260);

    // old=curr
    _old = _curr;
}


void ofApp::SpeedCalculation() {

    // comparing currentCentroid and avgCentroid
    if (!wait) {
        if (_debug) std::cout << "1 - comparing currentCentroid and avgCentroid" << endl;
        if (distance(currentCentroid, avgCentroid) < _constant) {
            avgCentroid = ((avgCentroid * elapsedTime) + currentCentroid * constant) / (elapsedTime + constant);
            elapsedTime += constant;
            if (_debug) std::cout << "1 - elapsedTime: " << elapsedTime << "s" << endl;
        }
        else {
            elapsedTime = constant;
            avgCentroid = currentCentroid;
        }
        // step was deteced
        float StepTimeThreshold = 0.095f;
        if (elapsedTime >= StepTimeThreshold) {
            if (_debug) std::cout << "2 - step was deteced" << endl;
            _stepDist = distance(currentCentroid, previousStep);
            stepDetected = true;
            _Timer2 = _Timer1;
            _Timer1 = currentTime;
            if (_stepDist > 4)stepDistance = _stepDist;
            ofSetColor(ofColor::gold);
            ofDrawRectangle(0, 0, ofGetWidth() * 0.1f, ofGetHeight() * 0.1f);
            centroid1 = currentCentroid;
            stepLocations.push_front(centroid1);
            if (stepLocations.size() > 6) stepLocations.pop_back();
            centroid2 = previousStep;
            /*teta.pushToWindow(ofRadToDeg(atan2(centroid1.y - centroid2.y, centroid1.x - centroid2.x)));
            if (_debug)std::cout << teta.mean() << endl;*/ 
            previousStep = currentCentroid;
            elapsedTime = constant;
            avgCentroid = currentCentroid;
        }
    }


/********************* STEP DETECTION *********************/
// if stepDetected wait for current and previous to be bigger than Threshold
    if (stepDetected) {
        // variables
        vec2 avgR = vec2(0, 0);
        vec2 HighestRate = vec2(0, 0);
        vec2 avgS = vec2(0, 0);
        vec2 HighestSpeed = vec2(0, 0);
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
        if (abs(currentTime - _Timer1) < 2000) {
            ofSetColor(ofColor::pink);
            ofDrawRectangle(0, ofGetHeight() - ofGetHeight() * 0.1f, ofGetWidth() * 0.1f, ofGetHeight() * 0.1f);
            if (_debug) std::cout << "5 - calculating speed" << endl;

            // check if turned in place
            if (xWin) cXcYColor = ofColor::cyan;
            else cXcYColor = ofColor::orange;
            if (xWin != oldXwin && abs(currentTime - timer5) > T4) {
                turned = true;
                timer5 = currentTime;
                oldXwin = xWin;
            }
            

            int _rateWindow = 10;
            int _rateSpeed = 6;
            // calculate distance between centroids
            rateOfChange = abs(currentCentroid - previousCentroid) / constant;
            previousCentroid = currentCentroid;
            // average rate of change and speed
            avgRateOfChange.push_back(rateOfChange);
            if (avgRateOfChange.size() > _rateWindow - 1) avgRateOfChange.pop_front();
            for (int i = 0; i < avgRateOfChange.size(); i++) {
                //if (length(avgRateOfChange[i]) > length(HighestRate)) HighestRate = avgRateOfChange[i];
                avgR += avgRateOfChange[i];
            }
            //if (avgRateOfChange.size() == _rateWindow) avgR = (avgR - HighestRate) / (avgRateOfChange.size() - 1);
            avgR /= avgRateOfChange.size();
            avgSpeed.push_back(avgR);
            if (avgSpeed.size() > _rateSpeed - 1) avgSpeed.pop_front();
            for (int i = 0; i < avgSpeed.size(); i++) {
                //if (length(avgSpeed[i]) > length(HighestSpeed)) HighestSpeed = avgSpeed[i];
                avgS += avgSpeed[i];
            }
            // if (avgSpeed.size() == _rateSpeed)avgS = (avgS - HighestSpeed) / (avgSpeed.size() - 1);
            avgS /= avgSpeed.size();
            // final speed
            if (stepDistance > 0)finalSpeed = avgS / stepDistance;
            theSpeed = finalSpeed.x + finalSpeed.y;
            if (theSpeed < 1 || abs(_Timer2 - _Timer1) > 1200 && stopped) theSpeed = 0;
        }
    }
}

void ofApp::FinalSpeedDecision() {
    // initial movement
    if (theSpeed < 3 && movementDetection > 0.7f) {
        theSpeed += 2.5f;
    }
    if (stopped) {
        for (int i = 0; i < 50; i++)finalS.process(0);
        theSpeed = 0;
        stopped = !stopped;
    }
    if (abs(currentTime - timer5) > T4) turned = false;
    theSpeed = finalS.process(theSpeed);
    //acceleration.pushToWindow(theSpeed);
}
#pragma endregion

#pragma region OrientationMethods
void ofApp::calculateOrientation() {
    // ORIENTATION ////////
    int maxVectorSize = 64;
    vec2 center = currentCentroid;
    float min1, min2, max1, max2, max3, max4;

    distances.clear();
    points.clear();

    // Selecting the high pressure points
    //int t = 30;
    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > T2) {
                points.push_back(vec2(i, j));
                if (points.size() > maxVectorSize) points.pop_front();
            }
        }
    }
    // order values in distances array
    for (int i = 0; i < points.size(); i++) {
        if (i == 0) distances.push_back(vec3(points[i], distance(points[i], currentCentroid)));
        else {
            float tmpDistance = distance(points[i], currentCentroid);
            if (tmpDistance >= distances[i - 1].z) distances.insert(distances.end(), vec3(points[i], tmpDistance));
            else {
                int j = i - 2;
                if (j > 0) {
                    for (j; j >= 0; j--) {
                        if (tmpDistance > distances[j].z) {
                            distances.insert(distances.begin() + j + 1, vec3(points[i], tmpDistance));
                            break;
                        }
                        else if (j == 0) {
                            distances.insert(distances.begin(), vec3(points[i], tmpDistance));
                        }
                    }
                }
                else {
                    distances.insert(distances.begin(), vec3(points[i], tmpDistance));
                }
            }
        }
    }
    vec2 feetDistance1 = normalize(centroid1 - avgCentroid);
    vec2 feetDistance2 = normalize(centroid2 - avgCentroid);
    float angle1 = ofRadToDeg(acos(dot(feetDistance1, previousCentroid1)));
    float angle2 = ofRadToDeg(acos(dot(feetDistance1, previousCentroid2)));
    //if (angle1 > 1 || angle2 > 1) {
    //    //angle1 = a1.process(angle1);
    //    std::cout << " angle1: " << angle1 << endl;
    //    std::cout << " angle2: " << angle2 << endl;
    //}
    previousCentroid1 = feetDistance1;
    previousCentroid2 = feetDistance2;
    //if (distances.size() > 2) {
    //    vec2 currentShort = normalize(vec2(distances[0].x, distances[0].y) - avgCentroid);
    //    vec2 currentLong1 = normalize(vec2(distances[distances.size() - 1].x, distances[distances.size() - 1].y) - avgCentroid);
    //    vec2 currentLong2 = normalize(vec2(distances[distances.size() - 2].x, distances[distances.size() - 2].y) - avgCentroid);

    //    float angle1 = ofRadToDeg(acos(dot(currentShort, prevShort)));
    //    float angle2 = ofRadToDeg(acos(dot(currentLong1, prevLong1)));
    //    float angle3 = ofRadToDeg(acos(dot(currentLong2, prevLong2)));

    //    angle1 = a1.process(angle1);
    //    angle2 = a2.process(angle2);
    //    angle3 = a3.process(angle3);

    //    if (angle1 > 5)   cout << " angle1: " << angle1 << endl;
    //    if (angle2 > 5)   cout << " angle2: " << angle2 << endl;
    //    if (angle3 > 5)   cout << " angle3: " << angle3 << endl;


    //    prevShort = currentShort;
    //    prevLong1 = currentLong1;
    //    prevLong2 = currentLong2;
    //}

}

void ofApp::blobDetection() {
    vector<FootBlob> currentBlobs;
    int blobCounter = 0;

    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            if (readingArray[(i * sensorsBase) + j] > T2) {
                bool found = false; 
                if (currentBlobs.size() != 0) {
                    for (int f = 0; f < currentBlobs.size(); f++) {
                        if (currentBlobs[f].IsNear(i, j, T5)) {
                            currentBlobs[f].add(i, j);
                            found = true;
                            break;
                        }
                    }
                }
                if (!found) {   
                    FootBlob b;
                    b.NewBlob(i, j); 
                    currentBlobs.push_back(b);
                }
            }
        }
    }

    // no blobs currently
    if (blobs.empty() && currentBlobs.size() > 0) {
        for (int i = 0; i < currentBlobs.size();i++) {
            currentBlobs[i].id = blobCounter;
            blobs.push_back(currentBlobs[i]);
            blobCounter++;
        }
    }
    // Match whatever blobs you can match
    else if (blobs.size() <= currentBlobs.size()) {
        for (int i = 0; i < blobs.size();i++) {
            float recordD = 10000;
            FootBlob matched;
            for (int j = 0; j < currentBlobs.size();j++) {
                vec2 centerB = blobs[i].getCenter();
                vec2 centerCB = currentBlobs[j].getCenter();
                float d = distance(centerB, centerCB);
                if (d < recordD && !currentBlobs[j].taken) {
                    recordD = d;
                    matched = currentBlobs[j];
                }
            }
            matched.taken = true;
            blobs[i].become(matched);
        }
        // Whatever is left over, make new blobs
        for (int i = 0;i < currentBlobs.size(); i++) {
            if (!currentBlobs[i].taken) {
                currentBlobs[i].id = blobCounter;
                blobs.push_back(currentBlobs[i]);
                blobCounter++;
            }
        }
    }
    else if (blobs.size() > currentBlobs.size()) {
        for (int i = 0; i < blobs.size();i++) {
            blobs[i].taken = false;
        }
        // Match whatever blobs you can match
        for (int i = 0; i < currentBlobs.size();i++) {
            float recordD = 10000;
            FootBlob matched;
            for (int j = 0; j < blobs.size();j++) {
                blobs[j].taken = false;
                vec2 centerB = blobs[j].getCenter();
                vec2 centerCB = currentBlobs[i].getCenter();
                float d = distance(centerB, centerCB);
                if (d < recordD && !blobs[j].taken) {
                    recordD = d;
                    matched = blobs[j];
                }
            }
            matched.taken = true;
            matched.become(currentBlobs[i]);
        }
        for (int i = blobs.size() - 1; i >= 0; i--) {
            FootBlob b = blobs[i];
            if (!b.taken) {
                blobs.erase(blobs.begin() + i);
            }
        }
    }
    vec2 biggest;
    vec2 secondBiggest;
    if (blobs.size() > 2) {
        for (int i = 0; i < blobs.size(); i++) {
            if (biggest.x < blobs[i].size()) {
                biggest.x = blobs[i].size();
                biggest.y = i;
            }
        }
        for (int i = 0; i < blobs.size(); i++) {
            if (blobs[i].size() < biggest.x) {
                if (secondBiggest.x < blobs[i].size()) {
                    secondBiggest.x = blobs[i].size();
                    secondBiggest.y = i;
                }
            }
        }
        centroid1 = blobs[biggest.y].getCenter();
        centroid2 = blobs[secondBiggest.y].getCenter();
    }
    else if (blobs.size() == 2) {
        centroid1 = blobs[0].getCenter();
        centroid2 = blobs[1].getCenter();
    }
    teta.push_front(ofRadToDeg(atan2(centroid1.y - centroid2.y, centroid1.x - centroid2.x)));
    if (teta.size() > 60) teta.pop_back();


    numOfFoot = currentBlobs.size();
    //text.drawString(ofToString(numOfFoot), 100, 100);
}
#pragma endregion

#pragma region DrawMethods
void ofApp::drawMatrix(int startingX, int startingY, int size) {
    ofPushStyle();
    // draw matrix
    int spacing = size / sensorsBase;

    //Optical flow
    float* flowXPixels;
    float* flowYPixels;
    float sumX = 0, sumY = 0, avgX = 0, avgY = 0;
    int numOfEntries;
    numOfEntries = 0;
    if (_debugOpticalFlow && !simulating) {
        flowXPixels = flowX.getPixelsAsFloats();
        flowYPixels = flowY.getPixelsAsFloats();
    }

    for (int i = 0; i < sensorsBase; i++) {
        for (int j = 0; j < sensorsBase; j++) {
            ofSetColor(255);
            ofPushMatrix();
            ofPushStyle();
            ofTranslate(startingX + (i * spacing), startingY + (j * spacing));
            ofNoFill();
            ofDrawRectangle(0, 0, spacing, spacing);
            ofPopStyle();
            float ReadingMapped = ofMap(readingArray[(i * sensorsBase) + j], 0, 80, 0, 255, true);
            if (_debugOpticalFlow && !simulating) {
                /*if (calculatedFlow) {
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
                }*/
            }
            else {
                ofColor pressureColor = ReadingMapped;
                if (ReadingMapped < 100 && _debugNewMethod) pressureColor.a = 50;
                ofSetColor(pressureColor);
                ofFill();
                ofDrawRectangle(0, 0, spacing, spacing);
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
    ofSetColor(255, 255, 0);
    for (int i = 0; i < points.size(); i++) {
        ofDrawCircle(startingX + (points[i].x * spacing),
                     startingY + (points[i].y * spacing),
                     spacing / 3);

        //ofDrawArrow(vec3(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing, 0), vec3(startingX + points[i].x * spacing, startingY + points[i].y * spacing, 0), 1);
    }
    if (distances.size() > 5) {
        ofSetColor(ofColor::yellow);
        ofDrawLine(vec2(startingX + distances[0].x * spacing, startingY + distances[0].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
        ofSetColor(ofColor::cyan);
        ofDrawLine(vec2(startingX + distances[1].x * spacing, startingY + distances[1].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
        ofSetColor(ofColor::white);
        ofDrawLine(vec2(startingX + distances[distances.size() - 1].x * spacing, startingY + distances[distances.size() - 1].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
        ofSetColor(ofColor::red);
        ofDrawLine(vec2(startingX + distances[distances.size() - 2].x * spacing, startingY + distances[distances.size() - 2].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
        ofSetColor(ofColor::purple);
        ofDrawLine(vec2(startingX + distances[distances.size() - 3].x * spacing, startingY + distances[distances.size() - 3].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
        ofSetColor(ofColor::pink);
        ofDrawLine(vec2(startingX + distances[distances.size() - 4].x * spacing, startingY + distances[distances.size() - 4].y * spacing), vec2(startingX + currentCentroid.x * spacing, startingY + currentCentroid.y * spacing));
    }


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
    ofSetColor(0, 255, 0);
    ofSetLineWidth(3);
    ofPolyline stepsLine;
    float hipotenuse = distance(centroid1, centroid2);
    if (hipotenuse > 7) {
        stepsLine.addVertex(startingX + centroid1.x * spacing, startingY + centroid1.y * spacing);
        stepsLine.addVertex(startingX + centroid2.x * spacing, startingY + centroid2.y * spacing);
        
        //if (_debug) std::cout << teta.mean() << endl;
    }
    stepsLine.draw();


    if (_debugNewMethod) {
        ofNoFill();
        ofSetColor(ofColor::white, 75);
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

    if (showPainting) {
        centroidPainting.addVertex(ofPoint(startingX + (avgCentroid.x * spacing), startingY + (avgCentroid.y * spacing)));
        if (centroidPainting.size() > (int)ofGetFrameRate() * 2) centroidPainting.removeVertex(0);
        ofSetColor(ofColor::fuchsia);
        centroidPainting.draw();
    }

    // FRONT DIRECTION
    if (teta.size() > 0) {
        ofPushMatrix();
        ofPushStyle();
        ofSetLineWidth(10);
        ofTranslate((startingX + size) * 0.5f, (startingY + size) * 0.5f);
        float _teta=0;
        for (float b : teta) _teta += b;
        _teta /= teta.size();
        ofRotateDeg(_teta);
        ofDrawArrow(vec3(0, 150, 0), vec3(0, -150, 0), 20);
        ofPopStyle();
        ofPopMatrix();
    }


    ofPopMatrix();
    ofPopStyle();
}

void ofApp::drawGraph(float sX, float sY, float avgX, float avgY, int sN, float speed) {
    ofPoint xpt, ypt;
    ofPoint avx, avy;
    ofPoint npt;
    ofPoint spt, smpt;
    xpt.set(xPos, ofMap(sX, -400, 400, ofGetHeight(), 0, true), 2);
    avx.set(xPos, ofMap(avgX, -400, 400, ofGetHeight(), 0, true), 2);
    ypt.set(xPos, ofMap(sY, -400, 400, ofGetHeight(), 0, true), 2);
    avy.set(xPos, ofMap(avgY, -400, 400, ofGetHeight(), 0, true), 2);
    npt.set(xPos, ofMap(sN, 0, sensorNumber, ofGetHeight(), 0, true), 2);
    /*spt.set(xPos, ofMap(theSpeed, 0, 20, ofGetHeight(), 0, true), 2);
    smpt.set(xPos, ofMap(movementDetection, 0, 9, ofGetHeight(), 0, true), 2);*/
    spt.set(xPos, ofMap(__fs, 0, 3, ofGetHeight(), 0, true), 2);
    smpt.set(xPos, ofMap(_fs, 0, 3, ofGetHeight(), 0, true), 2);
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

    ofSetColor(255, 100, 100, alphaX);
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
    ofDrawBitmapString("The Speed: " + ofToString(__fs), txtXpos, 65);
    ofDrawBitmapString("SpeedMul: " + ofToString(_fs), txtXpos, 75);
    /*ofDrawBitmapString("The Speed: " + ofToString(theSpeed), txtXpos, 65);
    ofDrawBitmapString("SpeedMul: " + ofToString(movementDetection), txtXpos, 75);*/
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
#pragma endregion

#pragma region ExtraMethods
void ofApp::setupDevices(string COM, int Baud) {
    // SERIAL COM 
    serial.setup(COM, Baud);
    if (_debug) std::cout << serial.available() << endl;
    if (serial.available() >= 0) {
        serial.writeByte('A');
    }
}

void ofApp::tryFirstConnection() {
    if (currentTime - timeOut > 100) {
        int inByte = serial.readByte();

        if (inByte == '1' || currentTime > 2000) {
            serial.writeByte('1');
            firstContact = true;
        }
        timeOut = currentTime;
    }

}

void ofApp::OpenCV() {
    ofPixels pixel;
    pixel.setFromPixels(readingArray, 16, 16, OF_IMAGE_GRAYSCALE);
    gray1 = pixel;
    contourFinder.findContours(gray1, 1, 5, 2, false, true);
    /*if (gray2.bAllocated) {
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
    }*/
}

void ofApp::oscSend() {
    // OSC send
    ofxOscMessage m;
    m.setAddress("/walkingpad");
    m.addFloatArg(MAX(__fs * 10,0)); //theSpeed
    //m.addFloatArg(theSpeed); //theSpeed
    //m.addFloatArg(movementDetection);
    //m.addBoolArg(jump);
    //m.addFloatArg(teta.mean());
    //m.addFloatArg(currentCentroid.x);
    //m.addFloatArg(currentCentroid.y);
    //m.addFloatArg(_fs*10);
    //m.addIntArg(sN);
    osc.sendMessage(m, false);
}

void ofApp::csvRecord() {
    // Log values in CSV file
    if (recording) {
        ofxCsvRow row;
        row.setFloat(0, avgCentroid.x); // CoG X
        row.setFloat(1, avgCentroid.y); // CoG Y
        row.setInt(2, N);   // NaS mean
        //row.setBool(3, jump);
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
        if (!recording) {
            filename = "rec_" +
                ofToString(ofGetDay()) + "_" +
                ofToString(ofGetMonth()) + "_" +
                ofToString(ofGetYear()) + "_" +
                ofToString(ofGetHours()) + "_" +
                ofToString(ofGetMinutes()) + "_" +
                ofToString(ofGetSeconds()) + "_" +
                ".csv";
            csv.createFile(filename);
        }
        else {
            // CSV
            bool exitAfterSave = false;
            exitAfterSave = csv.save(filename);
            if (exitAfterSave) {
                cout << "fileSaved with name: " << filename << endl;
                recording = !recording;
                //ofExit();
            }
        }
        toggle = !toggle; 
        recording = toggle;
    }
    if (key == '0') {
        _calibrateNow = true;
    }
    if (key == '1') {
        showX = true;
        showY = true;
        showNas = true;
        bigMatrix = false;
    }
    if (key == '2') {
        showSpeed = true;
        showSpeedMul = true;
        showX = false;
        showY = false;
        showNas = false;
        bigMatrix = false;
    }
    if (key == '3') {
        bigMatrix = true;
        _debugNewMethod = true;
        _debugOpticalFlow = false;
        showSpeed = false;
        showSpeedMul = false;
        showX = false;
        showY = false;
        showNas = false;
    }
    if (key == '4') {
        bigMatrix = true;
        _debugNewMethod = false;
        _debugOpticalFlow = true;
        showSpeed = false;
        showSpeedMul = false;
        showX = false;
        showY = false;
        showNas = false;
    }
    if (key == '5') {
        simulating = !simulating;
    }
    if (key == '.') {
        drawGui = !drawGui;
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


#pragma endregion



