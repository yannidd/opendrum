// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef OPENDRUM_MAGCAL_H_
#define OPENDRUM_MAGCAL_H_

// magnetic calibration constants
#define MAGBUFFSIZEX \
  12  // x dimension in magnetometer buffer (12x24 equals 288 elements)
#define MAGBUFFSIZEY \
  (2 * MAGBUFFSIZEX)  // y dimension in magnetometer buffer (12x24 equals 288
                      // elements)
#define MINMEASUREMENTS4CAL \
  40  // minimum number of measurements for 4 element calibration
#define MINMEASUREMENTS7CAL \
  100  // minimum number of measurements for 7 element calibration
#define MINMEASUREMENTS10CAL \
  150  // minimum number of measurements for 10 element calibration
#define MAXMEASUREMENTS \
  240                     // maximum number of measurements used for calibration
#define INTERVAL4CAL 75   // 3s at 25Hz: 4 element interval (samples)
#define INTERVAL7CAL 250  // 10s at 25Hz: 7 element interval (samples)
#define INTERVAL10CAL 750  // 30s at 25Hz: 10 element interval (samples)
#define MINBFITUT \
  10.0F  // minimum acceptable geomagnetic field B (uT) for valid calibration
#define MAXBFITUT \
  90.0F  // maximum acceptable geomagnetic field B (uT) for valid calibration
#define FITERRORAGINGSECS \
  7200.0F  // 2 hours: time (s) for fit error to increase (age) by e=2.718
#define MESHDELTACOUNTS 50  // magnetic buffer mesh spacing in counts (here 5uT)

// Edits from me ---------------------------------------------------------------
#include <Arduino.h>

static const float DEFAULTB = 50.0;  // default geomagnetic field (uT)
static const float ONETHIRD = 0.33333333;
static const float ONESIXTH = 0.166666667;
static const int OVERSAMPLE_RATIO = 8;
static const int SENSORFS = 200;

// accelerometer sensor structure definition
struct AccelSensor {
  // int32_t iSumGpFast[3];  // sum of fast measurements
  // float fGpFast[3];       // fast (typically 200Hz) readings (g)
  // float fGp[3];           // slow (typically 25Hz) averaged readings (g)
  // float fgPerCount;       // initialized to FGPERCOUNT
  // int16_t iGpFast[3];     // fast (typically 200Hz) readings
  int16_t iGp[3];  // slow (typically 25Hz) averaged readings (counts)
};

// magnetometer sensor structure definition
struct MagSensor {
  // int32_t iSumBpFast[3];  // sum of fast measurements
  // float fBpFast[3];   // fast (typically 200Hz) raw readings (uT)
  // float fBp[3];       // slow (typically 25Hz) averaged raw readings (uT)
  // float fBcFast[3];   // fast (typically 200Hz) calibrated readings (uT)
  // float fBc[3];       // slow (typically 25Hz) averaged calibrated readings
  // (uT)
  float fuTPerCount;  // initialized to FUTPERCOUNT
  // float fCountsPeruT;  // initialized to FCOUNTSPERUT
  int16_t iBpFast[3];  // fast (typically 200Hz) raw readings (counts)
  // int16_t iBp[3];      // slow (typically 25Hz) averaged raw readings
  // int16_t iBc[3];  // slow (typically 25Hz) averaged calibrated readings
};
// -----------------------------------------------------------------------------

// magnetometer measurement buffer
struct MagneticBuffer {
  int16_t iBpFast[3][MAGBUFFSIZEX]
                 [MAGBUFFSIZEY];  // uncalibrated magnetometer readings
  int32_t index[MAGBUFFSIZEX][MAGBUFFSIZEY];  // array of time indices
  int16_t tanarray[MAGBUFFSIZEX - 1];  // array of tangents of (100 * angle)
  int16_t iMagBufferCount;             // number of magnetometer readings
};

// magnetic calibration structure
struct MagCalibration {
  float fV[3];            // current hard iron offset x, y, z, (uT)
  float finvW[3][3];      // current inverse soft iron matrix
  float fB;               // current geomagnetic field magnitude (uT)
  float fFourBsq;         // current 4*B*B (uT^2)
  float fFitErrorpc;      // current fit error %
  float ftrV[3];          // trial value of hard iron offset z, y, z (uT)
  float ftrinvW[3][3];    // trial inverse soft iron matrix size
  float ftrB;             // trial value of geomagnetic field magnitude in uT
  float ftrFitErrorpc;    // trial value of fit error %
  float fA[3][3];         // ellipsoid matrix A
  float finvA[3][3];      // inverse of ellipsoid matrix A
  float fmatA[10][10];    // scratch 10x10 matrix used by calibration algorithms
  float fmatB[10][10];    // scratch 10x10 matrix used by calibration algorithms
  float fvecA[10];        // scratch 10x1 vector used by calibration algorithms
  float fvecB[4];         // scratch 4x1 vector used by calibration algorithms
  int8_t iCalInProgress;  // flag denoting that a calibration is in progress
  int8_t iMagCalHasRun;  // flag denoting that at least one calibration has been
                         // launched
  int8_t iValidMagCal;   // integer value 0, 4, 7, 10 denoting both valid
                         // calibration and solver used
};

// function prototypes for function in magnetic.c
void MagCal_Run(struct MagSensor *pthisMag, struct MagCalibration *pthisMagCal,
                struct MagneticBuffer *pthisMagBuffer);
void fInitMagCalibration(struct MagCalibration *pthisMagCal,
                         struct MagneticBuffer *pthisMagBuffer);
void iUpdateMagnetometerBuffer(struct MagneticBuffer *pthisMagBuffer,
                               struct AccelSensor *pthisAccel,
                               struct MagSensor *pthisMag, int32_t loopcounter);
// void fInvertMagCal(struct MagSensor *pthisMag,
//                    struct MagCalibration *pthisMagCal);
void fUpdateCalibration4INV(struct MagCalibration *pthisMagCal,
                            struct MagneticBuffer *pthisMagBuffer,
                            struct MagSensor *pthisMag);
void fUpdateCalibration7EIG(struct MagCalibration *pthisMagCal,
                            struct MagneticBuffer *pthisMagBuffer,
                            struct MagSensor *pthisMag);
void fUpdateCalibration10EIG(struct MagCalibration *pthisMagCal,
                             struct MagneticBuffer *pthisMagBuffer,
                             struct MagSensor *pthisMag);

#endif  // #ifndef OPENDRUM_MAGCAL_H_
