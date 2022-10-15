#define GATE_SPEED         12 // [ms] lower number is higher servo speed
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
#define PUSHBUTTON_PIN            3
#define PUSHBUTTON_LED_PIN        4

#define NUM_SERVOS                2
#define NUM_WIGWAG_PANELS         4

static int servoLeftPin[NUM_SERVOS] = {A1, A3};
static int servoRightPin[NUM_SERVOS] = {A0, A2};
static boolean servoLeftInvert[NUM_SERVOS] = {false, true};
static boolean servoRightInvert[NUM_SERVOS] = {false, true};

static int yellowLedPin[NUM_WIGWAG_PANELS] = {5, 6, 7, 8};
static int wigwag1LedPin[NUM_WIGWAG_PANELS] = {9, 10, 11, 12};
static int wigwag2LedPin[NUM_WIGWAG_PANELS] = {A4, A5, A4, A5}; //A6 and A7 cannot do digital I/O


enum e_states {
              IDLE,             // all off, except pushbutton LED, awaiting pushbutton.
              PRESS_DELAY,      // button pressed. pushbutton LED off and short delay before action
              YELLOW,           // Yellow constant and sounder
              RED,              // Both Reds constant, Yellow constant and sounder
              RED_WIGWAGS,      // Yellow off, WigWags flashing and sounder
              LEFT_GATES_CLOSING, // WigWags flashing, boom lights on and sounder
              LEFT_GATES_CLOSED,  // WigWags flashing, boom lights on and sounder
              RIGHT_GATES_CLOSING,// WigWags flashing, boom lights on and sounder
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
              START_LEFT_CLOSING,  //
              START_RIGHT_CLOSING,  //
              FINISH_CLOSING, //
              START_OPENING,  //
              FINISH_OPENING, //
              RESET_BUTTON    //
};

e_states current_state = IDLE;
e_transitions current_transition = NO_TRANSITION;

byte led1, led2;
boolean wigwags_enabled = false;

float angleLeft = GATE_OPEN_ANGLE;
float setpointLeft = GATE_OPEN_ANGLE;
float angleRight = GATE_OPEN_ANGLE;
float setpointRight = GATE_OPEN_ANGLE;

unsigned long time_to_blink;
unsigned long time_to_close_gate;
unsigned long time_for_servo;

unsigned long time_to_delay_start;
unsigned long time_for_yellow;
unsigned long time_for_reds_and_yellow;
unsigned long time_for_red_wigwags;
unsigned long time_for_train_to_pass;
unsigned long time_for_post_operation_delay;

boolean gates_closed_left;
boolean gates_open_left;
boolean gates_closed_right;
boolean gates_open_right;
boolean alarm_sounding = false;

#include <Servo.h>
Servo gate_left_servos [NUM_SERVOS];
Servo gate_right_servos [NUM_SERVOS];

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
  pinMode(PUSHBUTTON_LED_PIN, OUTPUT);
  pinMode(SOUNDER_PIN, OUTPUT);

  for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
    pinMode(yellowLedPin[s], OUTPUT);
    pinMode(wigwag1LedPin[s], OUTPUT);
    pinMode(wigwag2LedPin[s], OUTPUT);
  }

  for (byte s = 0; s < NUM_SERVOS; s++){
    gate_left_servos[s].attach(servoLeftPin[s]);
    gate_right_servos[s].attach(servoRightPin[s]);
    if (servoLeftInvert[s]) {
      gate_left_servos[s].write(GATE_CLOSED_ANGLE-(angleLeft-GATE_OPEN_ANGLE));
    }
    else {
      gate_left_servos[s].write(angleLeft);
    }
    if (servoRightInvert[s]) {
      gate_right_servos[s].write(GATE_CLOSED_ANGLE-(angleRight-GATE_OPEN_ANGLE));
    }
    else {
      gate_right_servos[s].write(angleRight);
    }
    delay(50);
    gate_left_servos[s].detach();
    gate_right_servos[s].detach();
  }

  gates_closed_left = false;
  gates_open_left = true;
  gates_closed_right = false;
  gates_open_right = true;
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
        current_transition = START_LEFT_CLOSING;
        Serial.println("starting closing left gates.");
      }
    break;

    case LEFT_GATES_CLOSING: // Red wig-wags and servos moving
      if (gates_closed_left) {
        current_transition = START_RIGHT_CLOSING;
        Serial.println("finished closing gates.");
      }
    break;

    case RIGHT_GATES_CLOSING: // Red wig-wags and servos moving
      if (gates_closed_right) {
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
      if (gates_open_left) {
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
      for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
        digitalWrite(yellowLedPin[s], HIGH);
      }
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
      for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
        digitalWrite(wigwag1LedPin[s], HIGH);
        digitalWrite(wigwag2LedPin[s], HIGH);
      }
      time_for_reds_and_yellow = millis() + (unsigned long)RED_YELLOW_TIME;
      current_transition = NO_TRANSITION;
      current_state = RED;
    break;

    case START_WIGWAGS: //
      Serial.println("Time to start the Wig-Wags, Yellow off");
      for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
        digitalWrite(yellowLedPin[s], LOW);
      }
      wigwags_enabled = true;
      time_to_blink = millis() + (unsigned long)BLINK_SPEED;
      led1 = LOW;
      led2 = !led1;
      for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
        digitalWrite(wigwag1LedPin[s], led1);
        digitalWrite(wigwag2LedPin[s], led2);
      }
      time_for_red_wigwags = millis() + (unsigned long)WIGWAG_TIME;
      current_transition = NO_TRANSITION;
      current_state = RED_WIGWAGS;
    break;

    case START_LEFT_CLOSING: //
      Serial.println("Time to start closing the gates");

      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_left_servos[s].attach(servoLeftPin[s]);
      }

      gates_open_left = false;
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
      setpointLeft = GATE_CLOSED_ANGLE;
      current_transition = NO_TRANSITION;
      current_state = LEFT_GATES_CLOSING;
    break;

    case START_RIGHT_CLOSING: //
      Serial.println("Time to start closing the gates");

      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_right_servos[s].attach(servoRightPin[s]);
      }

      gates_open_right = false;
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
      setpointRight = GATE_CLOSED_ANGLE;
      current_transition = NO_TRANSITION;
      current_state = RIGHT_GATES_CLOSING;
    break;

    case FINISH_CLOSING: //
      Serial.println("Gates closed, wait for train");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_left_servos[s].detach();
      }
      time_for_train_to_pass = millis() + (unsigned long)TRAIN_PASSING_TIME;
      current_transition = NO_TRANSITION;
      current_state = GATES_CLOSED;
    break;

    case START_OPENING: //
      Serial.println("Time to start opening the gates");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_left_servos[s].attach(servoLeftPin[s]);
        gate_right_servos[s].attach(servoRightPin[s]);
      }
      gates_closed_left = false;
      gates_closed_right = false;
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
      setpointLeft = GATE_OPEN_ANGLE;
      setpointRight = GATE_OPEN_ANGLE;
      current_transition = NO_TRANSITION;
      current_state = GATES_OPENING;
    break;

    case FINISH_OPENING: //
      Serial.println("Gates opened");
      for (byte s = 0; s < NUM_SERVOS; s++){
        gate_left_servos[s].detach();
        gate_right_servos[s].detach();
      }
      for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
        digitalWrite(wigwag1LedPin[s], LOW);
        digitalWrite(wigwag2LedPin[s], LOW);
      }
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

  if (not gates_open_left and not gates_closed_left and millis() > time_for_servo) {
    if (gates_open_right or gates_closed_right) {
      time_for_servo = millis() + (unsigned long)GATE_SPEED;
    }
    if (angleLeft < setpointLeft) angleLeft = angleLeft + 0.125;
    if (angleLeft > setpointLeft) angleLeft = angleLeft - 0.125;
    // Serial.println((String)"Move Servos to " + angleLeft);
    for (byte s = 0; s < NUM_SERVOS; s++){
      if (servoLeftInvert[s]){
        gate_left_servos[s].write(GATE_CLOSED_ANGLE-(angleLeft-GATE_OPEN_ANGLE));
      }
      else {
        gate_left_servos[s].write(angleLeft);
      }
    }
    if (angleLeft == setpointLeft) {
      if (setpointLeft == GATE_OPEN_ANGLE) {
        gates_open_left = true;
      }
      else {
        gates_closed_left = true;
      }
    }
  }

  if (not gates_open_right and not gates_closed_right and millis() > time_for_servo) {
    time_for_servo = millis() + (unsigned long)GATE_SPEED;
    if (angleRight < setpointRight) angleRight = angleRight + 0.125;
    if (angleRight > setpointRight) angleRight = angleRight - 0.125;
    // Serial.println((String)"Move Servos to " + angle);
    for (byte s = 0; s < NUM_SERVOS; s++){
      if (servoRightInvert[s]){
        gate_right_servos[s].write(GATE_CLOSED_ANGLE-(angleRight-GATE_OPEN_ANGLE));
      }
      else {
        gate_right_servos[s].write(angleRight);
      }
    }
    if (angleRight == setpointRight) {
      if (setpointRight == GATE_OPEN_ANGLE) {
        gates_open_right = true;
      }
      else {
        gates_closed_right = true;
      }
    }
  }

    
  if(wigwags_enabled and millis() > time_to_blink) {
    time_to_blink = millis() + (unsigned long)BLINK_SPEED;
    led1 = !led1;
    led2 = !led1;
    for (byte s = 0; s < NUM_WIGWAG_PANELS; s++){
      digitalWrite(wigwag1LedPin[s], led1);
      digitalWrite(wigwag2LedPin[s], led2);
    }
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