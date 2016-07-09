#include <SoftwareSerial.h>

#define PINS 15
#define PIN_LEN 8

#define BLUE_BUTTON    13
#define RED_BUTTON     11
#define GREEN_BUTTON   10
#define YELLOW_BUTTON   9

#define YELLOW_LED 7
#define GREEN_LED  6
#define RED_LED    5

#define ESP_SWITCH 8

/*************************************************************************************************
  Change IP addresses and lengths in the defines
  AT+CIPSEND=1,8\r\n" (8 bytes send, pin len)
  "+IPD,1,7:SUCCESS"  (7 chars of SUCCESS word)
  "AT+CIPSTART=0,\"UDP\",\"0\",1333\r\n"                 (Listen on 1333)
  "AT+CIPSTART=1,\"UDP\",\"192.168.1.90\",1333,1333\r\n" (Server IP, listen and send to 1333)
*************************************************************************************************/

#define ESP_FREQ       9600
#define SEND_SIZE     "AT+CIPSEND=1,8\r\n"
#define RECV_SUCCESS  "+IPD,1,7:SUCCESS"
#define ESP_MUX       "AT+CIPMUX=1\r\n"
#define UDP_LISTEN    "AT+CIPSTART=0,\"UDP\",\"0\",1333\r\n"
#define UDP_CHANNEL   "AT+CIPSTART=1,\"UDP\",\"192.168.1.90\",1333,1333\r\n"
#define ESP01_BOOT_MS 375 // Got this number experimentally

/****************************************************************************
   TODO: Remove hardcoded strings. Craft the strings given a set of values
   Check for failures on ESP related things
   Implement brand new ESP config (Connect to wifi)
 ***************************************************************************/
 
SoftwareSerial ESPserial(2, 3); // RX | TX

bool          should_read[PINS];
unsigned char current_state[PINS];
unsigned char input_pins[] = {YELLOW_BUTTON, GREEN_BUTTON, RED_BUTTON, BLUE_BUTTON};
char pin_buffer[PIN_LEN + 3];

bool isconnected = false;

int buffer_index = 0;

// ratio_on is defined as a 0 - 1 time frame. 0.5 being 50%
void blink_led(int led, float ratio_on, int millisec, int loops = 1) {
  int time_on  = (ratio_on * (float)millisec);
  int time_off = ((1.0 - ratio_on) * (float)millisec);
  int i = 0;

  for (; i < loops; ++i) {
    // Inverse logic
    digitalWrite(led, LOW);
    delay(time_on);
    digitalWrite(led, HIGH);
    delay(time_off);
  }
}

void esp_listen(String * output) {
  while ( ! ESPserial.available()) {
    blink_led(YELLOW_LED, 0.70, 500);
  }

  *output = ESPserial.readString();
  Serial.write(output->c_str());
}

void esp_sendcmd(char * command, String * output) {
  ESPserial.write(command);
  esp_listen(output);
}

void post_data() {
  blink_led(YELLOW_LED, 0.70, 500);

  String retval;
  String str_pinlen = String(PIN_LEN + 2);
  char   cmd[]      = SEND_SIZE;

  esp_sendcmd(cmd, &retval);
  esp_sendcmd(pin_buffer, &retval);

  while (retval.indexOf("+IPD") == -1)
    esp_listen(&retval);

  if (retval.indexOf(RECV_SUCCESS) != -1) blink_led(GREEN_LED, 1, 1000);
  else                                    blink_led(RED_LED,   1, 1000);

  digitalWrite(ESP_SWITCH,  LOW);
}

void turn_transceiver_on() {
  String result;
  digitalWrite(ESP_SWITCH,  HIGH);
  delay(ESP01_BOOT_MS);
  esp_sendcmd(ESP_MUX, &result);
  esp_sendcmd(UDP_LISTEN, &result);
  esp_sendcmd(UDP_CHANNEL, &result);
}

void setup() {
  String result;
  pinMode(RED_BUTTON,    INPUT);
  pinMode(GREEN_BUTTON,  INPUT);
  pinMode(BLUE_BUTTON,   INPUT);
  pinMode(YELLOW_BUTTON, INPUT);

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED,    OUTPUT);
  pinMode(GREEN_LED,  OUTPUT);

  pinMode(ESP_SWITCH,  OUTPUT);

  digitalWrite(ESP_SWITCH,  HIGH);
  
  // Inverse logic - turn off the leds
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED,    HIGH);
  digitalWrite(GREEN_LED,  HIGH);

  memset(should_read, true, sizeof(should_read));
  memset(current_state, 0, sizeof(current_state));

  Serial.begin(ESP_FREQ);
  ESPserial.begin(ESP_FREQ);

  pin_buffer[PIN_LEN + 1] = '\r';
  pin_buffer[PIN_LEN + 2] = '\n';
  pin_buffer[PIN_LEN + 3] = 0;

  turn_transceiver_on();
  blink_led(RED_LED,    1.0, 333);
  blink_led(YELLOW_LED, 1.0, 333);
  blink_led(GREEN_LED,  1.0, 333);
  digitalWrite(ESP_SWITCH,  LOW);
}

void loop() {
  int input;

  for ( input = 0; input < 4; ++input ) {
    if (buffer_index == PIN_LEN) {
      buffer_index = 0;
      post_data();
    }
    if (digitalRead(input_pins[input]) == HIGH) {
      if (should_read[input_pins[input]]) {
        // Turn on the transceiver on the first keystroke
        if ( ! buffer_index) turn_transceiver_on();
        should_read[input_pins[input]] = false;
        pin_buffer[buffer_index++] = (input + '0');
      }
    }
    else if ( ! should_read[input_pins[input]]) {
      should_read[input_pins[input]] = true;
    }
  }

  if (ESPserial.available()) Serial.write(ESPserial.read());
  if (Serial.available())    ESPserial.write(Serial.read());
}
