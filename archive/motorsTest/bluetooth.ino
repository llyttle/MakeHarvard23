// // #include <SoftwareSerial.h>
// // SoftwareSerial Bluetooth(10,9); // RX, TX

// // int LED = 13; // the on-board LED
// // int Data; // the data received
 
// // void setup() {
// //   Bluetooth.begin(9600);
// //   Bluetooth.setTimeout(2);
// //   Serial.begin(9600);
// //   Bluetooth.println("Send 1 to turn on the LED. Send 0 to turn Off");
// //   pinMode(LED,OUTPUT);
// // }
 
// // void loop() {

// //   if (Bluetooth.available()){ //wait for data received
// //     Data=Bluetooth.read();
// //     if(Data){  
// //       digitalWrite(LED,1);
// //       // Serial.println("LED On!");
// //       // Bluetooth.println("LED On!");
// //     }
// //     else{
// //        digitalWrite(LED,0);
// //       //  Serial.println("LED Off!");
// //       //  Bluetooth.println("LED  On D13 Off ! ");
// //     }
// //     // else{;}
// //   }
// //   // delay(1000);
// // }



// // #define ledPin 7
// // int state = 0;

// // void setup() {
// //   pinMode(ledPin, OUTPUT);
// //   digitalWrite(ledPin, LOW);
// //   Serial.begin(38400); // Default communication rate of the Bluetooth module
// // }

// // void loop() {
// //   if(Serial.available() > 0){ // Checks whether data is comming from the serial port
// //     state = Serial.read(); // Reads the data from the serial port
// //  }

// //  if (state == '0') {
// //   digitalWrite(ledPin, LOW); // Turn LED OFF
// //   Serial.println("LED: OFF"); // Send back, to the phone, the String "LED: ON"
// //   state = 0;
// //  }
// //  else if (state == '1') {
// //   digitalWrite(ledPin, HIGH);
// //   Serial.println("LED: ON");;
// //   state = 0;
// //  } 
// // }


//     #include <SoftwareSerial.h>
//     SoftwareSerial EEBlue(10, 11); // RX | TX
//     void setup()
//     {
     
//       Serial.begin(9600);
//       EEBlue.begin(9600);  //Default Baud for comm, it may be different for your Module. 
//       Serial.println("The bluetooth gates are open.\n Connect to HC-05 from any other bluetooth device with 1234 as pairing key!.");
     
//     }
     
//     void loop()
//     {
     
//       // Feed any data from bluetooth to Terminal.
//       if (EEBlue.available())
//         Serial.write(EEBlue.read());
     
//       // Feed all data from termial to bluetooth
//       if (Serial.available())
//         EEBlue.write(Serial.read());
//     }