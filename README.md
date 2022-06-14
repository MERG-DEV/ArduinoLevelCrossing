# ArduinoLevelCrossing
Arduino sketch to control a model modern UK level crossing on a diorama.

Modern 4 barrier UK Level Crossing Controller

This sketch was written to control a level crossing on a diorama created by the
Basingstoke and North Hants Model Railway Society for the Community Safety Officer
for Network Rail (Wessex) to be taken to schools, youth clubs, etc to promote
safety on the railways. The level crossing is fully functioning to current
regulations, but being a diorama, there are no moving trains. The sequence is
started by pressing a button, and there is a delay while an imginary train passes by
before the gates raise again to complete the sequence.

The marker lights on the barrier arms are not included as there was not enough time to add
them and wire them up, but could easily be added to the sketch as an output to come on when
the barriers start to lower, and be extinguished when they have been raised again.

It should be straightforward to modify this sketch so that sensors would start the sequence
and signal when a train has passed.

This sketch requires the following libraries:
  -  Servo

The sketch is created as a Finite State Machine. Each stage of the sequence is a State, and
certain 'events' will trigger the transition to the next State, and cause various actions to be performed.

Some of these actions will be to simple light or extinguish an LED.
Other actions will be to set up a timed delay. This is done by finding the current time in milliseconds, and
adding the the required delay duration in milliseconds to it, and storing the result. The program loop will
then keep checking the current time to see if the target time has been reached yet. If not, the program can
continue doing other things.
Another important action is to set where the gate/barriers servos should move to. Again, a target angle is set
and a repeating process will gradually move the servo(s) from their current position towards the target position,
until the target position is reached.

Initially created by Ian Morgan, May 2022
