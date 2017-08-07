/*
This sketch demonstrates how to send data from a Device
to a Host in a Gazell network.

The host and upto 3 devices should have the RGB shield
attached.  When Button A on a Device is pressed, the
associated led on the Host will toggle.  Device1 is
associated with the Red led, Device2 with the Green led
and Device3 with the Blue led.

The Green led on the Device will blink to indicate
that an acknowledgement from the Host was received.
*/

#include <RFduinoGZLL.h>

device_t role = HOST;

// pin for the Green Led
int green_led = 3;

struct myCustomPacket_t
{
  short arr[9];
};

struct myCustomPacket_t packet;
struct myCustomPacket_t packet1;

void setup()
{

  Serial.begin(9600);
  Serial.println("Check");
  pinMode(green_led, OUTPUT);

  // start the GZLL stack  
  RFduinoGZLL.begin(role);
}

// Define my custom packet of no more than 20 bytes.

void loop()
{
  
  //Serial.print("Device One\n");
  Serial.print("1:  ");
  Serial.print ("A: ");
  Serial.print (packet.arr[0]);
  Serial.print (",\t");
  Serial.print (packet.arr[1]);
  Serial.print (",\t");
  Serial.print (packet.arr[2]);
  Serial.print (";\t");

  // Gyroscope
  Serial.print ("G: ");
  Serial.print (packet.arr[3]);
  Serial.print (",\t");
  Serial.print (packet.arr[4]);
  Serial.print (",\t");
  Serial.print (packet.arr[5]);
  Serial.print (";\t");

  // Magnetometer
  Serial.print ("M: ");
  Serial.print (packet.arr[6]);
  Serial.print (",\t");
  Serial.print (packet.arr[7]);
  Serial.print (",\t");
  Serial.print (packet.arr[8]);
  Serial.print (";\t");
  Serial.print ("\n");
  Serial.println();


  //Serial.print("Device Two\n");
  Serial.print("2:  ");
  Serial.print ("A: ");
  Serial.print (packet1.arr[0]);
  Serial.print (",\t");
  Serial.print (packet1.arr[1]);
  Serial.print (",\t");
  Serial.print (packet1.arr[2]);
  Serial.print (";\t");

  // Gyroscope
  Serial.print ("G: ");
  Serial.print (packet1.arr[3]);
  Serial.print (",\t");
  Serial.print (packet1.arr[4]);
  Serial.print (",\t");
  Serial.print (packet1.arr[5]);
  Serial.print (";\t");

  // Magnetometer
  Serial.print ("M: ");
  Serial.print (packet1.arr[6]);
  Serial.print (",\t");
  Serial.print (packet1.arr[7]);
  Serial.print (",\t");
  Serial.print (packet1.arr[8]);
  Serial.print (";\t");
  Serial.print ("\n");
  Serial.println();
  //Serial.println("I am Host");
  //delay(50);
 
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  
  
  if (device == DEVICE0){
   memcpy( &packet, &data[0], len);
  
  }

  if(device == DEVICE1) {
    memcpy( &packet1, &data[0], len);
  }
  
  // this test is not needed for a single device
  

  // no data to piggyback on the acknowledgement sent back to the Device
  //RFduinoGZLL.sendToDevice(device, "OK");
}
