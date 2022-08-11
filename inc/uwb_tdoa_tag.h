#ifndef __LPS_TDOA_TAG_H__
#define __LPS_TDOA_TAG_H__

#include "libdw1000.h"

#include "mac.h"

#include "physical_constants.h"


#define LOCODECK_NR_OF_TDOA_ANCHORS 8

typedef uint64_t locoAddress_t;

typedef struct {
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_TDOA_ANCHORS];

//   point_t anchorPosition[LOCODECK_NR_OF_TDOA_ANCHORS];
  bool combinedAnchorPositionOk;
} lpsTdoaAlgoOptions_t;

// anchor_j - anchor_i 
typedef struct uwbTdoa_s {
  uint8_t header;
  uint8_t stamp;
  uint8_t anchor_i;
  uint8_t anchor_j;
  uint32_t data;
} uwbTdoa_t;

typedef struct {
  uint8_t type;
  uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA_ANCHORS];
  uint32_t timestamps[LOCODECK_NR_OF_TDOA_ANCHORS];
  uint16_t distances[LOCODECK_NR_OF_TDOA_ANCHORS];
} __attribute__((packed)) rangePacket2_t;

// Protocol version
#define PACKET_TYPE_TDOA 0x22

#define TDOA_RECEIVE_TIMEOUT 10000

void lpsTdoaTagSetOptions(lpsTdoaAlgoOptions_t* newOptions);

#endif // __LPS_TDOA_TAG_H__