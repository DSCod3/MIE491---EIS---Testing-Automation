String message;

enum OPERATING_MODE {
  SELECT,
  MANUAL,
  OPERATION,
  PAUSE,
  ERROR
};

OPERATING_MODE MODE = SELECT;

#pragma region Mode Strings
String PROGRAM_EXIT = "PROGRAM_EXIT";

String MANUAL_MODE_ENTER = "M_ENTER";

String OPERATION_MODE_ENTER = "O_ENTER";

String MODE_EXIT = "MODE_EXIT";

String ERROR_MODE_ENTER = "E_ENTER";
String ERROR_MODE_EXIT = "ERR_CLEAR";

String CONNECTION_CHECK_REQUEST = "Connection check.";
String CONNECTION_CHECK_RESPONSE = "Connection OK.";

String REPORTING_INTERVAL_EDIT = "R_INT_EDIT";
#pragma endregion

#pragma region Manual Mode Commands and Variables
String SET_LEFT = "SET_L"; // Set left piston pressure
String SET_RIGHT = "SET_R"; // Set right piston pressure

int separatorIndex;
String messageHeader;
float messageValue;
#pragma endregion

#pragma region Operation Mode Commands and Variables
String OPERATION_START = "O_START";
String OPERATION_PAUSE = "O_PAUSE";
String OPERATION_RESUME = "O_RESUME";
String OPERATION_ESTOP = "O_ESTOP";

bool forceDatapointsExchanged = false;
#pragma endregion

#pragma region System Variables
unsigned int dataReportingInterval = 250; // in milliseconds
unsigned long testStartTime;
unsigned long modeStartTimestamp = 0;
unsigned long lastDataReportTimestamp = 0;


float pressureLeft;
float pressureRight;
float forceLeft;
float forceRight;
float distanceUp;
float distanceDown;

bool runTest = false;
bool testPaused = false;


#pragma endregion

#pragma region I/O

#pragma endregion

#pragma region Data/IO Functions

void updateData(){
  pressureLeft = (float)random(0, 150);
  pressureRight = (float)random(0, 150);
  forceLeft = (float)random(0, 5000);
  forceRight = (float)random(0, 5000);
  distanceUp = (float)random(0,500);
  distanceDown = (float)random(0,500);
}

void sendData(){
  Serial.println("$");
  Serial.print(millis()-modeStartTimestamp);
  Serial.print(",");
  Serial.print(pressureLeft);
  Serial.print(",");
  Serial.print(pressureRight);
  Serial.print(",");
  Serial.print(forceLeft);
  Serial.print(",");
  Serial.print(forceRight);
  Serial.print(",");
  Serial.print(distanceUp);
  Serial.print(",");
  Serial.println(distanceDown);
}

void updateAndSendData(){
  updateData();
  sendData();
}



#pragma endregion

#pragma region Supporting Functions
bool debugMode = true;

void dPrint(String msg){
  if(debugMode){
    Serial.print(msg);
  }
}

void dPrintline(String msg){
  if(debugMode){
    Serial.println(msg);
  }
}

void ClearIncomingBytes(){
  delay(1500);
  while(Serial.available()){
    Serial.read();
  }
}
#pragma endregion

void setup() {
  Serial.begin(115200);
  while(!Serial);
  ClearIncomingBytes();
  testStartTime = millis();
}

void loop() {

  switch(MODE){
  
    #pragma region SELECT
    case SELECT:
      //dPrint("Entered selection mode.");  
      if(Serial.available()){
        message = Serial.readStringUntil('\n');
        
        if(message == CONNECTION_CHECK_REQUEST){
          Serial.println(CONNECTION_CHECK_RESPONSE);
        }

        else if(message == MANUAL_MODE_ENTER){
          MODE = MANUAL;
        }

        else if(message == OPERATION_MODE_ENTER){
          MODE = OPERATION;
        }

        else if(message == ERROR_MODE_ENTER){
          MODE = ERROR;
        }

        else if(message == MODE_EXIT){
          // Do nothing
        }

        else if(message == REPORTING_INTERVAL_EDIT){
          while(!Serial.available());
          dataReportingInterval = Serial.readStringUntil('\n').toInt();
          dPrint("Reporting interval set to ");
          dPrint(String(dataReportingInterval));
          dPrintline(" ms.");
        }

        else{
          dPrintline(message);
        }
      }
      else{
        delay(100);
      }
      break;
    #pragma endregion
    
    #pragma region MANUAL
    case MANUAL:
      // TODO: Likewise to operations, during manual mode data needs to be sent to the computer constantly, so let me know what the keyphrase will be, as well as the format;
      dPrintline("Entered manual mode.");
      modeStartTimestamp = millis();      
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == MODE_EXIT){
            MODE = SELECT;
            break;
          }

          // Split the string into header and value
          separatorIndex = message.indexOf(':');
          if(separatorIndex != -1){
            messageHeader = message.substring(0, separatorIndex);
            messageValue = message.substring(separatorIndex+1).toFloat();
          }
          else{
            continue;
          }

          if(messageHeader == SET_LEFT){
            //Set the left piston pressure here. The value is in messageValue (psi). This will be an absolute value (not an increment).
          }

          else if(messageHeader == SET_RIGHT){
            //Set the right piston pressure here. The value is in messageValue (psi). This will be an absolute value (not an increment).
          }

          else{
            continue;
          }
        }

        if((millis() - lastDataReportTimestamp) > dataReportingInterval){
          updateAndSendData();
          lastDataReportTimestamp = millis();
        }
        
      }
      break;
    #pragma endregion

    #pragma region OPERATION
    case OPERATION:
      modeStartTimestamp = millis(); 
      dPrintline("Entered operation mode.");  
      
      // This while loop is used to wait for the computer to exchange the force datapoints and start command
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == MODE_EXIT){
            MODE = SELECT;
            break;
          }

          else if(message == OPERATION_START){
            runTest = true;
            // Start the test, move to a different loop
            // In the new loop, you need to handle the following messages that could be sent by the computer:
            // OPERATION_ESTOP
            // OPERATION_PAUSE_TEST
            // If the test completes succesfully, send a keyword to the computer and tell me what the keyword will be
            // Also during the test, data needs to be sent to the computer, so let me know what format this will be
            dPrintline("Test starting...");
            break;
          }
        }

        else{
            continue;
        }
      }
        // For test running
      while(runTest){
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == OPERATION_ESTOP){
            // ESTOP Code
            break;
          }

          else if(message == OPERATION_PAUSE){
            testPaused = true;
            // Code to pause operation
          }

          else if(message == OPERATION_RESUME){
            testPaused = false;
            // Code to resume operation
          }

          else if(message == MODE_EXIT && debugMode){
            MODE = SELECT;
            break;
          }

        }

        // Code for constant dat areporting
        if((millis() - lastDataReportTimestamp) > dataReportingInterval && !testPaused){
          updateAndSendData();
          lastDataReportTimestamp = millis();
        }

        if(!testPaused){
          // YOUR OPERATION MODE CONTROL CODE HERE WHICH LOOPS
        }
        
      }
      runTest = false;
      break;
    #pragma endregion
    
    #pragma region ERROR
    case ERROR:
      dPrintline("Entered error mode.");  
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == ERROR_MODE_EXIT){
            MODE = SELECT;
            break;
          }
        }

        else{
            continue;
        }
      }
      break;
    #pragma endregion

    default:
      dPrintline("NO CASE");
      break;

  }
  
}
