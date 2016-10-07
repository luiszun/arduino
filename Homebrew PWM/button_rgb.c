/* Manipulate color in a RGB led with custom made PWM [Not using built in PWM0 */]

#define RED_BUTTON   7
#define GREEN_BUTTON 4
#define BLUE_BUTTON  2

#define RED_LED   13
#define GREEN_LED 12
#define BLUE_LED  8

#define PINS 15

#define PULSE_BASE 40
#define INCREMENT 1

#define CURRENT_LED    button_led[current_button]

signed char   button_led[PINS];
unsigned char pulse_width[PINS];
unsigned char current_width[PINS];
unsigned char current_state[PINS];
bool          should_read[PINS];

void setup() {
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);
  
  pinMode(RED_BUTTON,   INPUT);
  pinMode(GREEN_BUTTON, INPUT);
  pinMode(BLUE_BUTTON,  INPUT);

  memset(button_led, -1, sizeof(button_led));
  memset(should_read, true, sizeof(should_read));

  memset(pulse_width, 0, sizeof(pulse_width));  
  memset(current_width, 0, sizeof(current_width));
  memset(current_state, 0, sizeof(current_state));
  
  button_led[RED_BUTTON]   = RED_LED;
  button_led[GREEN_BUTTON] = GREEN_LED;
  button_led[BLUE_BUTTON]  = BLUE_LED;
  
  pinMode(13, OUTPUT);
}

void loop(void) {
  
  int current_button;

  for (current_button=0; current_button<PINS; ++current_button) {
    if (button_led[current_button] > -1) {  
        if (current_width[CURRENT_LED] > 0) {
          digitalWrite(CURRENT_LED, (current_state[CURRENT_LED] ? HIGH : LOW));
          current_width[CURRENT_LED]--;
        }
        else {
          digitalWrite(CURRENT_LED, LOW);
          current_state[CURRENT_LED] = ~current_state[CURRENT_LED];
          current_width[CURRENT_LED] = current_state[CURRENT_LED] ? pulse_width[CURRENT_LED] : PULSE_BASE;
        }
        
        if (   should_read[current_button]
            && digitalRead(current_button) == HIGH) {
            
            should_read[current_button] = false;
            pulse_width[CURRENT_LED] += INCREMENT;
            pulse_width[CURRENT_LED] %= (unsigned char)(PULSE_BASE * 0.66);
        }
        
        if (! should_read[current_button] 
            && digitalRead(current_button) == LOW) {
          
          should_read[current_button] = true;
        }    
    }
  }
}