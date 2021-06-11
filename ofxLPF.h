//
//  ofxLPF.h
//  Simple Low Pass Filter
//
//  Created by Koki Ibukuro on 6/17/14.
//
//

#pragma once

class ofxLPF {
public:
    ofxLPF(const int samplerate = 60, const double cutoff = 1.8f);
    void initialize(const int samplerate, const double cutoff);
    double process(const double value);

private:
    double ax[3];
    double by[3];
    double xv[3];
    double yv[3];
};