// Upstream: https://github.com/drowe67/freedv-gui/blob/master/src/reporting/pskreporter.cpp
//
// License: https://github.com/drowe67/freedv-gui/blob/master/COPYING (GNU Lesser General Public License v2.1)
//
// Authors: https://github.com/drowe67/freedv-gui/graphs/contributors
//
// Commit: https://github.com/drowe67/freedv-gui/commit/737418b51044d1ce5d5888d61ddc0b8c1dd167dd

#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include "common.h"

#include <errno.h>
#include <string.h>

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <stdio.h>
#include <string.h>

#ifdef __ANDROID__
#include <android/log.h>
#endif

#include "pskreporter.h"

#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/util/datetime.h"

extern time_t epoch;

// Live server
#define PSK_REPORTER_HOSTNAME "report.pskreporter.info"
#define PSK_REPORTER_PORT "4739"

// RX record:
/* For receiverCallsign, receiverLocator, decodingSoftware use */

static const unsigned char rxFormatHeader[] = {
  0x00, 0x03, 0x00, 0x24, 0x99, 0x92, 0x00, 0x03, 0x00, 0x00,
  0x80, 0x02, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x08, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,
  0x00, 0x00
};

// TX record:
/* For senderCallsign, frequency, sNR (1 byte), mode, informationSource (1 byte), flowStartSeconds use */

static const unsigned char txFormatHeader[] = {
  0x00, 0x02, 0x00, 0x34, 0x99, 0x93, 0x00, 0x06,
  0x80, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x05, 0x00, 0x04, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x0A, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,
  0x80, 0x0B, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F,
  0x00, 0x96, 0x00, 0x04
};

SenderRecord::SenderRecord(std::string callsign, unsigned int frequency, char snr)
  : callsign(callsign)
  , frequency(frequency)
  , snr(snr)
{
  mode = "FT8";
  infoSource = 1;
  /* XXX */
  datetime_t dt;
  rtc_get_datetime(&dt);
  struct tm t;
  memset(&t, 0, sizeof(struct tm));
  t.tm_year = dt.year - 1900;
  t.tm_mon = dt.month - 1;
  t.tm_mday = dt.day;
  t.tm_hour = dt.hour;
  t.tm_min = dt.min;
  t.tm_sec = dt.sec;
  time_t timeSinceEpoch = mktime(&t);
  flowTimeSeconds = timeSinceEpoch;
}

int SenderRecord::recordSize()
{
  return (1 + callsign.size()) + 4 + 1 + (1 + mode.size()) + 1 + 4;
}

void SenderRecord::encode(char* buf)
{
  // Encode callsign
  char* fieldPtr = &buf[0];
  *fieldPtr = (char)callsign.size();
  memcpy(fieldPtr + 1, callsign.c_str(), callsign.size());

  // Encode frequency
  fieldPtr += 1 + callsign.size();
  unsigned int temp = htonl(frequency);
  memcpy((unsigned char*)fieldPtr, (unsigned char *)&temp, 4);
  // *((unsigned short*)fieldLoc) = htons(getRxDataSize_());

  // Encode SNR
  fieldPtr += sizeof(unsigned int);
  *fieldPtr = snr;

  // Encode mode
  fieldPtr += 1;
  *fieldPtr = (char)mode.size();
  memcpy(fieldPtr + 1, mode.c_str(), mode.size());

  // Encode infoSource
  fieldPtr += 1 + mode.size();
  *fieldPtr = infoSource;

  // Encode flow start time
  fieldPtr += 1;
  temp = htonl(flowTimeSeconds);
  memcpy((unsigned char*)fieldPtr, (unsigned char *)&temp, 4);
  // *((unsigned int*)fieldPtr) = htonl(flowTimeSeconds);
}

PskReporter::PskReporter(std::string callsign, std::string gridSquare, std::string software)
  : currentSequenceNumber_(0)
  , receiverCallsign_(callsign)
  , receiverGridSquare_(gridSquare)
  , decodingSoftware_(software)
{
  srand(time(0));
  randomIdentifier_ = rand();
}

PskReporter::~PskReporter()
{
  recordList_.clear();
}

void PskReporter::addReceiveRecord(std::string callsign, unsigned int frequency, char snr)
{
  recordList_.push_back(SenderRecord(callsign, frequency, snr));
}

static void print_hex(char *str, int len)
{
  int i;

  for (i = 0; i < len; ++i)
    printf("%02x", str[i]);
  printf("\n");
}

char packet[4096]; // 4 KiB max

int PskReporter::getRxDataSize_()
{
  int size = 4 + (1 + receiverCallsign_.size()) + (1 + receiverGridSquare_.size()) + (1 + decodingSoftware_.size());
  if ((size % 4) > 0)
  {
    // Pad to aligned boundary.
    size += (4 - (size % 4));
  }
  return size;
}

int PskReporter::getTxDataSize_()
{
  if (recordList_.size() == 0)
  {
    return 0;
  }

  int size = 4;
  for (auto& item : recordList_)
  {
    size += item.recordSize();
  }
  if ((size % 4) > 0)
  {
    // Pad to aligned boundary.
    size += (4 - (size % 4));
  }
  return size;
}

void PskReporter::encodeReceiverRecord_(char* buf)
{
  // Encode RX record header.
  buf[0] = 0x99;
  buf[1] = 0x92;

  // Encode record size.
  char* fieldLoc = &buf[2];
  *((unsigned short*)fieldLoc) = htons(getRxDataSize_());

  // Encode RX callsign.
  fieldLoc += sizeof(unsigned short);
  *fieldLoc = (char)receiverCallsign_.size();
  memcpy(fieldLoc + 1, receiverCallsign_.c_str(), receiverCallsign_.size());

  // Encode RX locator.
  fieldLoc += 1 + receiverCallsign_.size();
  *fieldLoc = (char)receiverGridSquare_.size();
  memcpy(fieldLoc + 1, receiverGridSquare_.c_str(), receiverGridSquare_.size());

  // Encode RX decoding software.
  fieldLoc += 1 + receiverGridSquare_.size();
  *fieldLoc = (char)decodingSoftware_.size();
  memcpy(fieldLoc + 1, decodingSoftware_.c_str(), decodingSoftware_.size());
}

void PskReporter::encodeSenderRecords_(char* buf)
{
  if (recordList_.size() == 0) return;

  // Encode TX record header.
  buf[0] = 0x99;
  buf[1] = 0x93;

  // Encode record size.
  char* fieldLoc = &buf[2];
  *((unsigned short*)fieldLoc) = htons(getTxDataSize_());

  // Encode individual records.
  fieldLoc += sizeof(unsigned short);
  for (auto& rec : recordList_)
  {
    rec.encode(fieldLoc);
    fieldLoc += rec.recordSize();
  }
}

unsigned char *PskReporter::send(int *length)
{
  // Header (2) + length (2) + time (4) + sequence # (4) + random identifier (4) +
  // RX format block + TX format block + RX data + TX data
  int dgSize = 16 + sizeof(rxFormatHeader) + sizeof(txFormatHeader) + getRxDataSize_() + getTxDataSize_();
  if (getTxDataSize_() == 0) dgSize -= sizeof(txFormatHeader);

  // char* packet = new char[dgSize];
  memset(packet, 0, dgSize);

  // Encode packet header.
  packet[0] = 0x00;
  packet[1] = 0x0A;

  // Encode datagram size.
  char* fieldLoc = &packet[2];
  *((unsigned short*)fieldLoc) = htons(dgSize);

  // Encode send time.
  fieldLoc += sizeof(unsigned short);
  datetime_t dt;
  rtc_get_datetime(&dt);
  struct tm t;
  memset(&t, 0, sizeof(struct tm));
  t.tm_year = dt.year - 1900;
  t.tm_mon = dt.month - 1;
  t.tm_mday = dt.day;
  t.tm_hour = dt.hour;
  t.tm_min = dt.min;
  t.tm_sec = dt.sec;
  time_t timeSinceEpoch = mktime(&t);
  *((unsigned int*)fieldLoc) = htonl(timeSinceEpoch);
  // printf("Epoch is %lu\n", timeSinceEpoch);
  // *((unsigned int*)fieldLoc) = htonl(epoch);
  // *((unsigned int*)fieldLoc) = htonl(time(0));

  // Encode sequence number.
  fieldLoc += sizeof(unsigned int);
  *((unsigned int*)fieldLoc) = htonl(currentSequenceNumber_++);

  // Encode random identifier.
  fieldLoc += sizeof(unsigned int);
  *((unsigned int*)fieldLoc) = htonl(randomIdentifier_);

  // Copy RX and TX format headers.
  fieldLoc += sizeof(unsigned int);
  memcpy(fieldLoc, rxFormatHeader, sizeof(rxFormatHeader));
  fieldLoc += sizeof(rxFormatHeader);

  if (getTxDataSize_() > 0)
  {
    memcpy(fieldLoc, txFormatHeader, sizeof(txFormatHeader));
    fieldLoc += sizeof(txFormatHeader);
  }

  // Encode receiver and sender records.
  encodeReceiverRecord_(fieldLoc);
  fieldLoc += getRxDataSize_();
  encodeSenderRecords_(fieldLoc);
  recordList_.clear();

  // Send to PSKReporter.
  *length = dgSize;
  return (unsigned char *)packet;
}

