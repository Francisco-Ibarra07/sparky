#include <PololuMaestro.h>

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#include <SoftwareSerial.h>

SoftwareSerial maestroSerial(10, 11); // RX, TX
MiniMaestro maestro(maestroSerial);

void setup()
{
  // Set the serial baud rate.
  maestroSerial.begin(115200);
}

void loop()
{
  /* setTarget takes the channel number you want to control, and
     the target position in units of 1/4 microseconds. A typical
     RC hobby servo responds to pulses between 1 ms (4000) and 2
     ms (8000). */

  // Set the target of channel 0 to 1500 us, and wait 2 seconds.
  maestro.setTarget(0, 6000);
  delay(2000);

  // Set the target of channel 0 to 1750 us, and wait 2 seconds.
  maestro.setTarget(0, 7000);
  delay(2000);

  // Set the target of channel 0 to 1250 us, and wait 2 seconds.
  maestro.setTarget(0, 5000);
  delay(2000);
}
