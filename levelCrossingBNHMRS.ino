#define GATE_SPEED        100 // [ms] lower number is higher servo speed
#define GATE_DELAY       5000 // [ms] delay time before gate closes 
#define GATE_OPEN_ANGLE    60 // servo angle
#define GATE_CLOSED_ANGLE 135 // servo angle
#define BLINK_SPEED       400 // [ms] smaller number is faster blinking
#define GATES_CLOSED_TIME 10000 // Time gates are closed for

#define START_DELAY        1000 // Time between pressing the button and the sequence starting
#define YELLOW_ONLY_TIME   5000
#define RED_YELLOW_TIME     500
#define WIGWAG_TIME        5000
#define TRAIN_PASSING_TIME 15000
#define RESET_TIME         10000

// IO Pins
#define SOUNDER_PIN               2
#define PUSHBUTTON_PIN            7
#define PUSHBUTTON_LED_PIN        8

#define NUM_SERVOS                4
#define NUM_WIGWAG_PANELS         4

static int servoPin[NUM_SERVOS] = {A0, A1, A2, A3};
static int boomLedPin[NUM_SERVOS] = {A0, A1, A2, A3};
static int yellowLedPin[NUM_WIGWAG_PANELS] = {A0, A1, A2, A3};
static int Wigwag1LedPin[NUM_WIGWAG_PANELS] = {A0, A1, A2, A3};
static int Wigwag2LedPin[NUM_WIGWAG_PANELS] = {A0, A1, A2, A3};


enum e_states {
              IDLE,             // all off, except pushbutton LED, awaiting pushbutton.
              PRESS_DELAY,      // button pressed. pushbutton LED off and short delay before action
              YELLOW,           // Yellow constant and sounder
              RED,              // Both Reds constant, Yellow constant and sounder
              RED_WIGWAGS,      // Yellow off, WigWags flashing and sounder
              GATES_CLOSING,    // WigWags flashing, boom lights on and sounder
              GATES_CLOSED,     // WigWags flashing, boom lights on. No sounder.
              GATES_OPENING,    // Wigwags off, boom lights on. No sounder.
              POST_ACTION_DELAY // all off.
};
enum e_transitions {
              NO_TRANSITION,  // awaiting another transition
              BUTTON_PRESS,   // The 'start' button has been pressed
              START_SEQUENCE, // after short delay after pressing button
              START_REDS,     // after yellow only
              START_WIGWAGS,  //
              START_CLOSING,  //
              FINISH_CLOSING, //
              START_OPENING,  //
              FINISH_OPENING, //
              RESET_BUTTON    //
};

e_states current_state = IDLE;
e_transitions current_transition = NO_TRANSITION;

byte led1, led2;
boolean wigwags_enabled = false;

byte angle    = GATE_OPEN_ANGLE;
byte setpoint = GATE_OPEN_ANGLE;

unsigned long time_to_blink;
unsigned long time_to_close_gate;
unsigned long time_for_servo;

unsigned long time_to_delay_start;
unsigned long time_for_yellow;
unsigned long time_for_reds_and_yellow;
unsigned long time_for_red_wigwags;
unsigned long time_for_train_to_pass;
unsigned long time_for_post_operation_delay;

boolean gates_closed;
boolean gates_open;
boolean alarm_sounding = false;

#include <Servo.h>
Servo gate_servos [NUM_SERVOS];

// Sounder data
// notes in the sounder melody:
#define NUM_SOUNDER_NOTES   4
int melody[NUM_SOUNDER_NOTES] = {
  4186, 698, 2794, 1047
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[NUM_SOUNDER_NOTES] = {
  250,50,250,50
};
byte current_note = 1;
unsigned long time_for_note;

void setup() {
  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);
  pinMode(WIGWAG_LED1_PIN, OUTPUT);
  pinMode(WIGWAG_LED2_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(BOOM_LED_PIN, OUTPUT);
  pinMode(PUSHBUTTON_LED_PIN, OUTPUT);
  pinMode(SOUNDER_PIN, OUTPUT);

  for (byte s = 0; s < NUM_SERVOS; s++){
    gate_servos[s].attach(servoPin[s]);
    gate_servos[s].write(angle);
    delay(50);
    gate_servos[s].detach();
  }

  gates_closed = false;
  gates_open = true;
  alarm_sounding = false;

  digitalWrite(PUSHBUTTON_LED_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("BNHMRS Level Crossing Controller");
  Serial.println();
  Serial.println("Waiting for pushbutton to be pressed.");
}

void loop() {

  // Check if there needs to be a change of state

  switch(current_state) {

    case IDLE: // idle
      if(digitalRead(PUSHBUTTON_PIN) == LOW) {
        current_transition = BUTTON_PRESS;
        Serial.println("pushbutton pressed.");
      }
    break;
  
    case PRESS_DELAY: // short pause
      if (millis() > time_to_delay_start) {
        current_transition = START_SEQUENCE;
        Serial.println("starting sequence.");
      }
    break;
  
    case YELLOW: // Yellow Light showing
      if (millis() > time_for_yellow) {
        current_transition = START_REDS;
        Serial.println("starting reds.");
      }
    break;
  
    case RED: // Red and Yellow constant
      if (millis() > time_for_reds_and_yellow) {
        current_transition = START_WIGWAGS;
        Serial.println("starting red wigwags.");
      }
    break;

    case RED_WIGWAGS: // Red wig-wags
      if (millis() > time_for_red_wigwags) {
        current_transition = START_CLOSING;
        Serial.println("starting closing gates.");
      }
    break;

    case GATES_CLOSING: // Red wig-wags and servos moving
      if (gates_closed) {
        current_transition = FINISH_CLOSING;
        Serial.println("finished closing gates.");
      }
    break;

    case GATES_CLOSED: // Red wig-wags
      if (millis() > time_for_train_to_pass) {
        current_transition = START_OPENING;
        Serial.println("starting opening gates.");
      }
    break;

    case GATES_OPENING: // Red wig-wags and servos moving
      if (gates_open) {
        current_transition = FINISH_OPENING;
        Serial.println("finished opening gates.");
      }
    break;

    case POST_ACTION_DELAY: // Red wig-wags
      if (millis() > time_for_post_operation_delay) {
        current_transition = RESET_BUTTON;
        Serial.println("starting waiting for button press.");
      }
    break;
  }

  // Are we transitioning from one state to another?
  switch(current_transition) {
    case BUTTON_PRESS: //
      Serial.println("Button Pressed, start short delay");
      time_to_delay_start = millis() + (unsigned long)START_DELAY;
      digitalWrite(PUSHBUTTON_LED_PIN, LOW);
      current_transition = NO_TRANSITION;
      current_state = PRESS_DELAY;
    break;
  
    case START_SEQUENCE: //
      Serial.println("Time to start the sequence. Yellow only");
      digitalWrite(YELLOW_LED_PIN, HIGH);
      time_for_yellow = millis() + (unsigned long)YELLOW_ONLY_TIME;
      current_note = 1;
      time_for_note = millis() + (unsigned long)noteDurations[current_note-1];
      alarm_sounding = true;
      tone(SOUNDER_PIN, melody[current_note-1]);
      current_transition = NO_TRANSITION;
      current_state = YELLOW;
    break;

    case START_REDS: //
      Serial.println("Time to start the sequence, Yellow and both Reds");
      digitalWrite(WIGWAG_LED1_PIN, HIGH);
      digitalWrite(WIGWAG_LED2_PIN, HIGH);
      digitalWrite(BOOM_LED_PIN, HIGH);
      time_for_reds_and_yellow = millis() + (unsigned long)RED_YELLOW_TIME;
      current_transition = NO_TRANSITION;
      current_state = RED;
    break;

    case START_WIGWAGS: //
      Serial.println("Time to start the Wig-Wags, Yellow off");
      digitalWrite(YELLOW_LED_PIN, LOW);
      wigwags_enabled = true;
      time_to_blink = millis() + (unsigned long)BLINK_SPEED;
      led1 = LOW;
      led2 = !led1;
      digitalWrite(WIGWAG_LED1_PIN, led1);
      digitalWrite(WIGWAG_LED2_PIN, led2);
      time_for_red_wigwags = millis() + (unsigned long)WIGWAG_TIME;
      current_transition = NO_TRANSITION;
      current_state = RED_WIGWAGS;
    break;

    case START_CLOSING: //
      Serial.println("Time to start closing the gates");

      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_servos[s].attach(servoPin[s]);
      }

      gates_open = false;
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
      setpoint = GATE_CLOSED_ANGLE;
      current_transition = NO_TRANSITION;
      current_state = GATES_CLOSING;
    break;

    case FINISH_CLOSING: //
      Serial.println("Gates closed, wait for train");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_servos[s].detach();
      }
      time_for_train_to_pass = millis() + (unsigned long)TRAIN_PASSING_TIME;
      current_transition = NO_TRANSITION;
      current_state = GATES_CLOSED;
    break;

    case START_OPENING: //
      Serial.println("Time to start opening the gates");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_servos[s].attach(servoPin[s]);
      }
      gates_closed = false;
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
      setpoint = GATE_OPEN_ANGLE;
      current_transition = NO_TRANSITION;
      current_state = GATES_OPENING;
    break;

    case FINISH_OPENING: //
      Serial.println("Gates opened");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_servos[s].detach();
      }
      digitalWrite(WIGWAG_LED1_PIN, LOW);
      digitalWrite(WIGWAG_LED2_PIN, LOW);
      digitalWrite(BOOM_LED_PIN, LOW);
      wigwags_enabled = false;
      alarm_sounding = false;
      noTone(SOUNDER_PIN);
      time_for_post_operation_delay = millis() + (unsigned long)RESET_TIME;
      current_transition = NO_TRANSITION;
      current_state = POST_ACTION_DELAY;
    break;

    case RESET_BUTTON: //
      Serial.println("Reset pushbotton");
      digitalWrite(PUSHBUTTON_LED_PIN, HIGH);
      current_transition = NO_TRANSITION;
      current_state = IDLE;
    break;
  }


  // Other processes

  if (not gates_open and not gates_closed and millis() > time_for_servo) {
    time_for_servo = millis() + (unsigned long)GATE_SPEED;
    if (angle < setpoint) angle++;
    if (angle > setpoint) angle--;
    // Serial.println((String)"Move Servos to " + angle);
    for (byte s = 0; s < NUM_SERVOS; s++){
      gate_servos[s].write(angle);
    }
    if (angle == setpoint) {
      if (setpoint == GATE_OPEN_ANGLE) {
        gates_open = true;
      }
      else {
        gates_closed = true;
      }
    }
  }

    
  if(wigwags_enabled and millis() > time_to_blink) {
    time_to_blink = millis() + (unsigned long)BLINK_SPEED;
    led1 = !led1;
    led2 = !led1;
    digitalWrite(WIGWAG_LED1_PIN, led1);
    digitalWrite(WIGWAG_LED2_PIN, led2);
  }

  if (alarm_sounding and millis() > time_for_note) {
    current_note = current_note + 1;
    if (current_note > NUM_SOUNDER_NOTES) {
      current_note = 1;
    } 
    // Serial.println((String)"Play Note " + current_note + " duration " + noteDurations[current_note-1]);
    time_for_note = millis() + (unsigned long)noteDurations[current_note-1];
    noTone(SOUNDER_PIN);
    tone(SOUNDER_PIN, melody[current_note-1]);
  }
}