#include <objects.h>

#include <WString.h>
#include <SoftwareSerial.h>


















// interface::interface() : myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ) 
// {}

void interface::Init()
{
    // mySerial.begin(115200);
    // mySerial.setTimeout(10);
    
    if(myMPU.begin())               
    {           myMPU.calculate_IMU_error();        }
    else                            
    {           
        // Serial2.println("MPU m4 48ala");    
    }
    
    for(int i = 0; i < 14; i++)     {           joints[i].init();                   }
    for(int i = 0; i <  4; i++)     {           palms[i].init();                    }
}







void interface::updateSensors(){
    for (int i = 0; i < 14; i++)    {        jointStates[i] = joints[i].readAngle()*M_PI/180.0;    }
    for (int i = 0; i <  4; i++)    {        palmStates[i]  = palms[i].getForce();      }

    myMPU.updateOrientation();
    OrientationQuaternion = myMPU.getOrientationQuaternion();
    
}

void interface::StatesSender()
{
    if(millis() - senderPrevTime> SEND_PERIOD)
    {
        senderPrevTime = millis();
        
        //   Serial.print("[");
        for(int i=0; i<14;i++){
            Serial.print(jointStates[i], 3);
            // Serial.print(jointStates[i], 3);
            // if(i != 13){
            Serial.print(",");
            // Serial.print(",");
            // }
        }
        for(int i=0; i<4;i++){
            Serial.print(palmStates[i], 1);
            // Serial.print(palmStates[i], 1);
            // if(i != 3){
            Serial.print(",");
            // Serial.print(",");
            // }
        }
        Serial.print(String(OrientationQuaternion.w) + String(",") + String(OrientationQuaternion.x) + String(",") + String(OrientationQuaternion.y) + String(",") + String(OrientationQuaternion.z));// + String(","));
        // Serial.print(String(OrientationQuaternion.w) + String(",") + String(OrientationQuaternion.x) + String(",") + String(OrientationQuaternion.y) + String(",") + String(OrientationQuaternion.z));// + String(","));
        // Serial.print(                                       String(omega.x)       + String(",") + String(omega.y)       + String(",") + String(omega.z)       + String(","));
        // Serial.print(                                       String(alpha.x)       + String(",") + String(alpha.y)       + String(",") + String(alpha.z)       + String(","));
        // Serial.print(                                       String(accel.x)       + String(",") + String(accel.y)       + String(",") + String(accel.z));
        Serial.println("");
        // Serial.println("");
        //   Serial.println("]");

    }
}

void interface::jointCmdExec()
{
    for (int i = 0; i < 14; i++)
    {        joints[i].writeAngleControlled((double)((jointCmd[i])));        }
    // {        joints[i].writeAngleControlled((double)((jointCmd[i]*(180.0/PI))));        }
}

// void interface::dataReceiver()
// {
//     if (Serial.available()) {
//         // Serial2.print(Serial.readStringUntil('\n'));
//     }

// }

bool interface::dataReceiver()
{
    int numNumbers = 0; // Variable to keep track of the number of numbers received
    // Serial.println("before 'if available'");
    if (Serial.available()) { // Check if data is available to read
        // Serial2.print("in 'if available'\t");
        String receivedData = Serial.readStringUntil('\n'); // Read until newline
        
        receivedData.trim(); // Remove leading/trailing whitespace

        // Check if the data starts with '[' and ends with ']'
        if (receivedData.startsWith("[") && receivedData.endsWith("]")) {
            receivedData.remove(0, 1);                         // Remove the leading '['
            receivedData.remove(receivedData.length() - 1, 1); // Remove the trailing ']'
            if (receivedData.startsWith("[")){
                receivedData.remove(0, 1);                         // Remove the leading '['
            }
            // Split the data based on the two possible separators: ",," or ','
            int separatorIndex;
            while ((separatorIndex = receivedData.indexOf(",,")) != -1 || (separatorIndex = receivedData.indexOf(',')) != -1)
            {
                if(((receivedData.indexOf(",,")) < (receivedData.indexOf(','))) && !(receivedData.indexOf(",,"))){
                    separatorIndex = receivedData.indexOf(",,");
                }
                else{
                    separatorIndex = receivedData.indexOf(",");
                }

                String value;
                if (receivedData.startsWith(",,", separatorIndex))
                {
                    value = receivedData.substring(0, separatorIndex);
                    receivedData.remove(0, separatorIndex + 2); // Remove the parsed number and separator

                }
                else
                {
                    value = receivedData.substring(0, separatorIndex);
                    receivedData.remove(0, separatorIndex + 1); // Remove the parsed number and separator
                }

                // Convert the substring to float
                data[numNumbers++] = value.toFloat();
                if(numNumbers-1<14){
                    data[numNumbers-1] -= 360;
                }
                // if(numNumbers <= 14){
                    // if(abs(data[numNumbers-1])>10){
                    //     data[numNumbers-1] /= 1000.0;
                    // }
                // }

                // Break if maximum number of numbers reached
                if (numNumbers >= MAX_RECEIVED_NUMBERS)
                    break;
            }

            // Convert the last remaining substring to float
            data[numNumbers++] = receivedData.toFloat();
            if(numNumbers-1<14){
                data[numNumbers-1] -= 360;
            }
            // if(numNumbers <= 14){
            //     if(abs(data[numNumbers-1])>10){
            //         data[numNumbers-1] /= 1000.0;
            //     }
            // }

            // Print the received floats
            double w, x, y, z;
            for (int i = 0; i < numNumbers; i++)
            {
                // Serial.print(String(data[i]) + String("\t"));
                if      (i < 14)    {   jointCmd[i] = data[i];  }
                else if (i == 14)   {   w = data[i];    }
                else if (i == 15)   {   x = data[i];    }
                else if (i == 16)   {   y = data[i];    }
                else if (i == 17)   {   z = data[i];
                    myMPU.setOrientationQuaternion(Quaternion(w, x, y, z));
                }

                // Serial.print("Float ");
                // Serial.print(i + 1);
                // Serial.print(": ");
                // Serial2.print(data[i]);
                // Serial2.print("\t");
            }
            // Serial2.println("");

            // Reset numNumbers for the next iteration
            numNumbers = 0;
        }
        return true;
    }
    else{
        return false;
    }

}





// void interface::dataReceiver()
// {
//     Serial.println("before Serial available");
//     if (mySerial.available()) {
//         Serial.println("before reding");
//         int bytesRead = mySerial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
//         buffer[bytesRead] = '\0';
//         Serial.println("before string");
//         String jsonStr(buffer);
//         Serial.println("before dynamic JSON");
//         // DynamicJsonDocument doc(1024);
//         DynamicJsonDocument doc(512);
//         Serial.println("before deserialization");
//         DeserializationError error = deserializeJson(doc, jsonStr);
//         Serial.println("after deserialization");
//         // lcd.print(jsonStr);

//         // Access the parsed array of doubles
//         if(!error)
//         {
//             Serial.println("no eror 1");

//             JsonArray jsonArray = doc.as<JsonArray>();
//             Serial.println("no eror 2");

//             int i=0;
//             double w,x,y,z;

//             for (double value : jsonArray) {
//                 Serial.println("entered for");
//                 // mySerial.print("Received double value: ");
//                 // mySerial.println(value);
//                 // convert string/JSON to array of doubles saved in jointCmd[], then execute
//                 Serial.print(String(value)+ String("\t"));
//                 if(i<14)
//                 {   jointCmd[i]  = value;}
//                 else if (i==14){w= value;}
//                 else if (i==15){x= value;}
//                 else if (i==16){y= value;}
//                 else if (i==17){z= value;
//                     myMPU.setOrientationQuaternion  (Quaternion(w, x, y, z));
//                 }
                
                
//                 i++;
//             }
//             Serial.println("hi from dataReceiver");
//             // lcd.clear();
//             // for(int i =0; i<12; i++){
//             //   lcd.print(jointCmd[i]);
//             //   lcd.print("|");
//             // }
//             // jointCmdExec();

//         }
//         else{
//             Serial.print("msg came: ");
//             Serial.println(String(buffer));
//             Serial.println(" ended");
//         }
//         Serial.println("out of if !error");
//     }
//     Serial.println("every dataReceiver");

// }













