#include <avr/sleep.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <LowPower.h>

static const PROGMEM u1_t NWKSKEY[16] = { 0x9E, 0xBE, 0x24, 0x5A, 0x7A, 0xBF, 0xFD, 0xB5, 0x6D, 0x34, 0xE5, 0x42, 0x72, 0x6A, 0xDC, 0x23 };
static const u1_t PROGMEM APPSKEY[16] = { 0xEB, 0xFD, 0x11, 0xEC, 0x38, 0xF5, 0x1C, 0xF5, 0xFB, 0x42, 0xF9, 0x00, 0x7E, 0x7E, 0x74, 0x3A };
static const u4_t DEVADDR = 0x26011ED3 ; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint32_t humidity;  //Stores humidity value
uint32_t temperature; //Stores temperature value
uint32_t humidity_sent=0;
uint32_t temperature_sent=0;
byte payload[4];
static osjob_t sendjob;
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2,4,6},
};

#define DHTPIN 8     // DATA Pin 
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

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

void do_send(osjob_t* j, byte *payload){
    // Check if there is not a current TX/RX job running
   // ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void sleepOneMinute()
{
  //sleep 1 minute et 4 secondes
  for (int i = 0; i < 8; i++) { 
     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  }
}

void setup () 
{
   Serial.begin(115200);
    Serial.println(F("Starting"));
    dht.begin();
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(10);
    #endif
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
     #if defined(CFG_eu868)
         LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      #elif defined(CFG_us915)
      LMIC_selectSubBand(1);
      #endif
       // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
}  // end of setup

void loop () {


  // disable ADC
  ADCSRA = 0;  //down from 335 µA to 0.355 µA in sleep mode
 
  /* set sleep mode running 0.36mA*/
 
   // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above

  sleepOneMinute();

  // Start job
    Serial.println("Sensing....");
          
          delay(20);
              //Read data and store it to variables hum and temp
              humidity = dht.readHumidity()*100;
              temperature= dht.readTemperature()*100;
              
          
          Serial.print("Value found: ");
          delay(10);
           Serial.print("Humidity: ");
              Serial.print(humidity);
              Serial.print(" %, Temp: ");
              Serial.print(temperature);
              Serial.println(" Celsius");
              delay(10);
              
              byte payload[4];
              if ((humidity!=humidity_sent) || (temperature!=temperature_sent)){
              payload[0]=highByte(humidity);
              payload[1]=lowByte(humidity);
              humidity_sent=humidity;

              payload[2]=highByte(temperature);
              payload[3]=lowByte(temperature);
              temperature_sent=temperature;
              
               LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
               Serial.println(F("Packet queued"));}
                os_runloop_once();
  
  }
