      
//Sensor
/*
#define    MPU9250_ADDRESS            0x68
#define    AK8963_ADDRESS             0x0C

#define    XG_OFFSET_H                0x13
#define    XA_OFFSET_H                0x77

#define    MPU9250_ACC_OUT            0x3B
#define    MPU9250_GYRO_OUT           0x43

#define    MPU9250_RA_SMPLRT_DIV      0x19
#define    MPU9250_RA_ACCEL_CONFIG    0x1C
#define    MPU9250_RA_GYRO_CONFIG     0x1B
#define    MPU9250_RA_CONFIG          0x1A
#define    MPU9250_RA_PWR_MGMT_1      0x6B
#define    MPU9250_RA_INT_PIN_CFG     0x37

#define    AK8963_MAG_CNTL            0x0A
#define    AK8963_MAG_ST1             0x02
#define    AK8963_MAG_XOUT_L          0x03

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08   
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
*/
/*
#define PIN_WIRE_SDA         (5u)
#define PIN_WIRE_SCL         (6u)
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Bluetooth
#include <RFduinoGZLL.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*                Button                   */

/*
int buttonInput = 16;
int buttonState = 0; 
*/

/*                Bluetooth                */

device_t role = DEVICE1;

int count = 0;

struct myCustomPacket_t
{
  short arr[9];
};

struct myCustomPacket_t packet;


/*                 Sensor                  */
/*
// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void MPU9250_Init(void)
{
  int i=0,j=0;
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++);
  }
*/
  /* configure MPU9250 */
  /*
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x00);
  //I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x40);
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_SMPLRT_DIV, 0x04);
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_CONFIG, 0x06);
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_ACCEL_CONFIG, ACC_FULL_SCALE_2_G);
  I2CwriteByte(MPU9250_ADDRESS, MPU9250_RA_GYRO_CONFIG, GYRO_FULL_SCALE_2000_DPS);
  
}
*/


void setup(void)
{
  
 
  delay(10);
  //Serial.begin(9600);
  /* Initialise the sensor */
  /*MPU9250_Init();*/

 
  RFduinoGZLL.begin(role);
  
  /*
  //Button
  pinMode(buttonInput, INPUT);  
  */

  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(void)
{
  // delay 0.02s
  delay(20);

  /**************** Read accelerometer and gyroscope ****************/
  // uint8_t Acc_offset[6], Gyro_offset[6];
  //uint8_t Acc[6], Gyro[6];

  // read Accelerometer offset
  /*
  I2Cread(MPU9250_ADDRESS, XA_OFFSET_H, 6, Acc_offset);
  int16_t Ax_offset = (Acc_offset[0] << 8 | Acc_offset[1]);
  int16_t Ay_offset = (Acc_offset[2] << 8 | Acc_offset[3]);
  int16_t Az_offset = (Acc_offset[4] << 8 | Acc_offset[5]);
  */
/*
  // read Accelerometer
  I2Cread(MPU9250_ADDRESS, MPU9250_ACC_OUT, 6, Acc);
  int16_t Ax = (Acc[0] << 8 | Acc[1]);
  int16_t Ay = (Acc[2] << 8 | Acc[3]);
  int16_t Az = (Acc[4] << 8 | Acc[5]);
*/
  // read Gyroscope offset
  /*
  I2Cread(MPU9250_ADDRESS, XG_OFFSET_H, 6, Gyro_offset);
  int16_t Gx_offset = (Gyro_offset[0] << 8 | Gyro_offset[1]);
  int16_t Gy_offset = (Gyro_offset[2] << 8 | Gyro_offset[3]);
  int16_t Gz_offset = (Gyro_offset[4] << 8 | Gyro_offset[5]);
  */
/*
  // read Gyroscope
  I2Cread(MPU9250_ADDRESS, MPU9250_GYRO_OUT, 6, Gyro);
  int16_t Gx = (Gyro[0] << 8 | Gyro[1]);
  int16_t Gy = (Gyro[2] << 8 | Gyro[3]);
  int16_t Gz = (Gyro[4] << 8 | Gyro[5]);
*/
  /**************** Read magnetometer ****************/
  //uint8_t Mag[6];
  //uint8_t ST1;


  /* configure AK8963 */
  /*
  I2CwriteByte(MPU9250_ADDRESS,MPU9250_RA_INT_PIN_CFG,0x02); //turn on Bypass Mode
  //I2CwriteByte(AK8963_ADDRESS,AK8963_MAG_CNTL,0x10); // Power-down mode
  I2CwriteByte(AK8963_ADDRESS,AK8963_MAG_CNTL,0x11); // 16bit output and single measurement mode
  */
  /* monitor DRDY bit of ST1 register */
   /* while(1)
    {
     I2Cread(AK8963_ADDRESS, AK8963_MAG_ST1, 1, &ST1);
     if(ST1 == 1)
       break;
    }
  /*  
    /* read measurement from data registers */
    /*I2Cread(AK8963_ADDRESS, AK8963_MAG_XOUT_L, 6, Mag);
    int16_t Mx = (Mag[1] << 8 | Mag[0]);
    int16_t My = (Mag[3] << 8 | Mag[2]);
    int16_t Mz = (Mag[5] << 8 | Mag[4]);
    I2CwriteByte(MPU9250_ADDRESS,MPU9250_RA_INT_PIN_CFG,0x00); //turn off Bypass Mode
  
  /**************** Display values ****************/
  /*
   
  //Test
  Serial.print("Test");
  
  // Accelerometer
  
  Serial.print ("A: ");
  Serial.print (Ax, DEC);
  Serial.print (",\t");
  Serial.print (Ay, DEC);
  Serial.print (",\t");
  Serial.print (Az, DEC);
  Serial.print (";\t");

  // Gyroscope
  Serial.print ("G: ");
  Serial.print (Gx, DEC);
  Serial.print (",\t");
  Serial.print (Gy, DEC);
  Serial.print (",\t");
  Serial.print (Gz, DEC);
  Serial.print (";\t");

  // Magnetometer
  Serial.print ("M: ");
  Serial.print (Mx, DEC);
  Serial.print (",\t");
  Serial.print (My, DEC);
  Serial.print (",\t");
  Serial.print (Mz, DEC);
  Serial.print (";\t");
  Serial.print ("\n");

  // Accelerometer
  
  Serial.print ("Ao: ");
  Serial.print (Ax_offset, DEC);
  Serial.print (",\t");
  Serial.print (Ay_offset, DEC);
  Serial.print (",\t");
  Serial.print (Az_offset, DEC);
  Serial.print (";\t");

  // Gyroscope
  Serial.print ("Go: ");
  Serial.print (Gx_offset, DEC);
  Serial.print (",\t");
  Serial.print (Gy_offset, DEC);
  Serial.print (",\t");
  Serial.print (Gz_offset, DEC);
  Serial.print (";\t");
  Serial.print ("\n");
  
  */
  
  /////////////////////////////////////////////////////////////////////////////////////////////

  /*              Bluetooth               */

  //Convert Sensor Data to Array
  //Possible could skip this step becuase cant sent an array
  
  
  
  packet.arr[0] = 5;
  packet.arr[1] = 2;
  packet.arr[2] = 3;
  packet.arr[3] = 4;
  packet.arr[4] = 5;
  packet.arr[5] = 6;
  packet.arr[6] = 7;
  //packet.arr[7] = 8;
  //packet.arr[8] = 9;
  packet.arr[7] = count++;
  packet.arr[8] = millis();
  RFduino_ULPDelay(20);
  //delay(1000);
  RFduinoGZLL.sendToHost((char *)&packet, sizeof(packet));
    //Serial.print("is tihs wookring");
  
  
 

}


