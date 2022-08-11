/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging tag implementation */

#include "uwb.h"

#include <string.h>
#include <stdio.h>

#include "cfg.h"
#include "led.h"

#include "tdoa_tag.h"

#include "libdw1000.h"
#include "physical_constants.h"
#include "dwOps.h"
#include "mac.h"

#define ANCHOR_OK_TIMEOUT 1500


static lpsTdoaAlgoOptions_t defaultOptions = {
   .anchorAddress = {
     0xbccf000000000000,
     0xbccf000000000001,
     0xbccf000000000002,
     0xbccf000000000003,
     0xbccf000000000004,
     0xbccf000000000005,
     0xbccf000000000006,
     0xbccf000000000007,
   },
   .combinedAnchorPositionOk = false,
};

static lpsTdoaAlgoOptions_t* options = &defaultOptions;

// State
typedef struct {
  rangePacket2_t packet;
  dwTime_t arrival;
  double clockCorrection_T_To_A;

  uint32_t anchorStatusTimeout;
} history_t;

static uint8_t previousAnchor;
// Holds data for the latest packet from all anchors
static history_t history[LOCODECK_NR_OF_TDOA_ANCHORS];
uwbConfig_t config;
static packet_t txPacket;

// Log data
// static float logUwbTdoaDistDiff[LOCODECK_NR_OF_TDOA_ANCHORS];
// static float logClockCorrection[LOCODECK_NR_OF_TDOA_ANCHORS];
// static uint16_t logAnchorDistance[LOCODECK_NR_OF_TDOA_ANCHORS];

typedef struct tdoaMeasurement_s {
  point_t anchorPositions[2];
  uint8_t anchorIds[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;


// #define printf(...)
#define debug(...) // printf(__VA_ARGS__)


static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(uint8_t anchorA, uint8_t anchorB, double distanceDiff) {
  // TODO: Send TDOA meas from here
  // tdoaMeasurement_t tdoa = {
  //   .stdDev = MEASUREMENT_NOISE_STD,
  //   .distanceDiff = distanceDiff,

    // .anchorPosition[0] = options->anchorPosition[anchorA],
    // .anchorPosition[1] = options->anchorPosition[anchorB]
  // };
  printf("[%d]<->[%d]:[%f].\n", anchorA, anchorB, distanceDiff);
}

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}


static bool isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
  // printf("prev:[%x] curr:[%x].\n", prevSeqNr, currentSeqNr);
  // printf("prev:[%x] curr:[%x].\n", currentSeqNr, ((prevSeqNr + 1) & 0xff ));
  return (currentSeqNr == ((prevSeqNr + 1) & 0xff));
}

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference anchor (Ar), Anchor n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Reference
// Anchor by Anchor N expressed in the clock of Anchor N"

static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket2_t* packet, const dwTime_t* arrival) {

  if (! isSeqNrConsecutive(history[anchor].packet.sequenceNrs[anchor], packet->sequenceNrs[anchor])) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t latest_rxAn_by_T_in_cl_T = history[anchor].arrival.full;
  const int64_t latest_txAn_in_cl_An = history[anchor].packet.timestamps[anchor];

  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - latest_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;
  return true;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket2_t* packet, const dwTime_t* arrival) {
  const bool isSeqNrInTagOk = isSeqNrConsecutive(history[anchor].packet.sequenceNrs[previousAnchor], packet->sequenceNrs[previousAnchor]);
  const bool isSeqNrInAnchorOk = isSeqNrConsecutive(history[anchor].packet.sequenceNrs[anchor], packet->sequenceNrs[anchor]);

  if (! (isSeqNrInTagOk && isSeqNrInAnchorOk)) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T  = arrival->full;
  const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
  const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
  const double clockCorrection = history[anchor].clockCorrection_T_To_A;

  const bool isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
  const bool isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }

  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t rxAr_by_T_in_cl_T = history[previousAnchor].arrival.full;

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

  *tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / TS_FREQ;

  return true;
}


static void txcallback(dwDevice_t *dev)
{
}


static void rxcallback(dwDevice_t *dev) {  
  int dataLength = dwGetDataLength(dev);
  //
  packet_t rxPacket;
  bzero(&rxPacket, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  //
  const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;

  if (packet->type == PACKET_TYPE_TDOA) {
    const uint8_t anchor = rxPacket.sourceAddress[0];

    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);
    // printf("Event pkt rec type:[%x] anc:[%d].\n", packet->type, anchor);

    if (anchor < LOCODECK_NR_OF_TDOA_ANCHORS) {

      calcClockCorrection(&history[anchor].clockCorrection_T_To_A, anchor, packet, &arrival);

      if (anchor != previousAnchor) {
        float tdoaDistDiff = 0.0;
        if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival)) {
          enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
        }
      }

      history[anchor].arrival.full = arrival.full;
      memcpy(&history[anchor].packet, packet, sizeof(rangePacket2_t));

      history[anchor].anchorStatusTimeout = xTaskGetTickCount() + ANCHOR_OK_TIMEOUT;

      previousAnchor = anchor;
    }
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t tdoaTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      // setRadioInReceiveMode(dev);
      // 10ms between rangings
      return 1;
      break;
    case eventPacketSent:
      // txcallback(dev);
      return 1;
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      //initiateRanging(dev);
      return 1;
      break;
    case eventReceiveFailed:
      // Try again ranging in 1ms
      return 1;
      break;
    case eventRangeRequest:
      // initiateRanging(dev);
      return 1;
      break;
    default:
      configASSERT(false);
  }

  return 1;
}

static void tdoaTagInit(uwbConfig_t * newconfig, dwDevice_t *dev)
{
  // Set the LED for tag mode
  ledOn(ledMode);

  config = *newconfig;

  previousAnchor = 0;
  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  // onEvent is going to be called with eventTimeout which will start ranging
}

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = tdoaTagInit,
  .onEvent = tdoaTagOnEvent,
};
