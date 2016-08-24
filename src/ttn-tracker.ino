#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define JOHAN
//#define PIETER
//#define LUUK

TinyGPS gps;
SoftwareSerial ss(7,8);

#ifdef PIETER
static const PROGMEM u1_t NWKSKEY[16] = {0x1E, 0xC9, 0x6A, 0x66, 0x67, 0x98, 0xCE, 0x15, 0x6C, 0xB2, 0x55, 0xD9, 0x6E, 0x90, 0x7D, 0xDC};
static const u1_t PROGMEM APPSKEY[16] = {0xC9, 0xB5, 0xDD, 0x45, 0x52, 0x3F, 0xAC, 0xE7, 0x9B, 0xCC, 0x53, 0xEB, 0x76, 0x95, 0x12, 0x70};
static const u4_t DEVADDR = 0x384261A9 ;
#endif

#ifdef JOHAN
#define OTAA
// Needs to be lsb, so should end in: be .... 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x70, 0x0A, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x86, 0x9F, 0x2C, 0xAC, 0x06, 0x24, 0x48, 0x8A };
// This key should be in big endian format (or, since it is not really a
static const u1_t PROGMEM APPKEY[16] = { 0x41, 0xCA, 0x74, 0x0A, 0x37, 0xFB, 0xE7, 0x0E, 0x02, 0x71, 0x69, 0xDF, 0xB5, 0xF9, 0x8C, 0x88 };
#endif

#ifdef LUUK
static const PROGMEM u1_t NWKSKEY[16] = {0x41, 0x37, 0x56, 0x65, 0xED, 0xC9, 0xFD, 0x61, 0x94, 0x71, 0x1D, 0x88, 0xDE, 0x10, 0xB0, 0x22};
static const u1_t PROGMEM APPSKEY[16] = {0x9B, 0xF2, 0x3B, 0xF4, 0x83, 0xA7, 0xE0, 0x77, 0xD1, 0xD2, 0x54, 0x0C, 0x26, 0xEA, 0x85, 0x94};
static const u4_t DEVADDR = 0x6A2A98D2;
#endif

#ifdef OTAA
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#else
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

uint8_t coords[6];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

void get_coords () {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat, flon;
  unsigned long age;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }

  if ( newData ) {
    gps.f_get_position(&flat, &flon, &age);
    flat = (flat == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flat;
    flon = (flon == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flon;
  }

  gps.stats(&chars, &sentences, &failed);

  int32_t lat = flat * 10000;
  int32_t lon = flon * 10000;

  // Pad 2 int32_t to 6 8uint_t, big endian (24 bit each, having 11 meter precision)
  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    get_coords();
    LMIC_setTxData2(1, (uint8_t*) coords, sizeof(coords), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup()
{
  Serial.begin(115200);

  #ifdef JOHAN
  Serial.println(F("Starting with config: Johan"));
  #endif

  #ifdef PIETER
  Serial.println(F("Starting with config: Pieter"));
  #endif

  #ifdef LUUK
  Serial.println(F("Starting with config: Luuk"));
  #endif

  ss.begin(9600);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Allow small clock errors
  LMIC_setClockError(MAX_CLOCK_ERROR * .001);

  // Disable data rate adaptation
  LMIC_setAdrMode(0);

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop();
}
