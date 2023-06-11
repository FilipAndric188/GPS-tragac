#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;
SoftwareSerial ss(0, 1);

static void smartdelay(unsigned long ms);
extern SoftwareSerial ss;

unsigned int count = 1;

uint8_t datasend[20];

char gps_lon[20] = {"\0"};
char gps_lat[20] = {"\0"};
char gps_alt[20] = {"\0"};
float flat, flon, falt;

static uint8_t mydata[] = "Hello, world!";

static const PROGMEM u1_t NWKSKEY[16] = {0xE4, 0x9E, 0x15, 0xB4, 0x7E, 0x3D, 0xB1, 0x32, 0xEA, 0x08, 0x03, 0xDE, 0x9A, 0x02, 0x6D, 0x12};
static const u1_t PROGMEM APPSKEY[16] = {0xEF, 0x56, 0x10, 0xB6, 0x9B, 0x55, 0x61, 0xF5, 0x17, 0x89, 0x70, 0xC2, 0x3B, 0x54, 0xC9, 0xDA};
static const u4_t DEVADDR = 0x260BB373;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

static osjob_t initjob, sendjob, blinkjob;

const unsigned TX_INTERVAL = 30;

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void GPSRead() {
  // Read GPS data using TinyGPSPlus library
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.altitude.isValid()) {
    snprintf(gps_alt, sizeof(gps_alt), "%.2f", gps.altitude.meters());
  } else {
    gps_alt[0] = '\0'; // Clear the array if altitude is not valid
  }

  if (gps.location.isValid()) {
    snprintf(gps_lat, sizeof(gps_lat), "%.6f", gps.location.lat());
    snprintf(gps_lon, sizeof(gps_lon), "%.6f", gps.location.lng());
  } else {
    gps_lat[0] = '\0'; // Clear the array if location is not valid
    gps_lon[0] = '\0'; // Clear the array if location is not valid
  }
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
        GPSRead();
        uint8_t len = sizeof(mydata);
        datasend[0] = len >> 8;
        datasend[1] = len;
        datasend[2] = gps_alt[0];
        datasend[3] = gps_alt[1];
        datasend[4] = gps_alt[2];
        datasend[5] = gps_alt[3];
        datasend[6] = gps_lat[0];
        datasend[7] = gps_lat[1];
        datasend[8] = gps_lat[2];
        datasend[9] = gps_lat[3];
        datasend[10] = gps_lon[0];
        datasend[11] = gps_lon[1];
        datasend[12] = gps_lon[2];
        datasend[13] = gps_lon[3];
        LMIC_setTxData2(1, datasend, sizeof(datasend), 0);
        Serial.println(F("Packet queued"));
    }
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
                Serial.println(F("Received "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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

void setup() {
    while (!Serial);
    Serial.begin(9600);
    delay(100);
    Serial.println(F("Starting"));
    os_init();
    LMIC_reset();
    #ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(3, 868850000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(4, 869050000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(5, 869525000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(6, 864100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(7, 864300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(8, 864500000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
    #else
    # error Region not supported
    #endif

    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7,14);

    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
