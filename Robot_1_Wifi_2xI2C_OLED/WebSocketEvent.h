//==============================================================
//                  WEB SOCKET EVENT AND NOT FOUND ROUTINE
//==============================================================


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) { // When a WebSocket message is received
  if (type == WS_EVT_CONNECT) {
    //client connected
    os_printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if (type == WS_EVT_DISCONNECT) {
    //client disconnected
    os_printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if (type == WS_EVT_ERROR) {
    //error was received from the other end
    os_printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if (type == WS_EVT_PONG) {
    //pong message was received (in response to a ping request maybe)
    os_printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
  } else if (type == WS_EVT_DATA) {
    char input;
    input = data[0];
    switch (input) {
      
    case '0': 
      Serial.println("UP0");
      pwm_left = 0;
      pwm_right = 0;
      break;
    case  '1':
      Serial.println("UP1");//jma test
      pwm_left = PWM_RANGE;
      break;
    case  '2':
      Serial.println("UP2");//jma test
      pwm_right = PWM_RANGE;
      break;
    case  '3':
      Serial.println("UP3");//jma test
      pwm_left = 0;
      pwm_right = 0;
      break;
    case  '4':
      Serial.println("UP4");//jma test
      Setpoint_left += 1024;
      break;
    case  '5':
      Serial.println("UP5");//jma test
      pwm_left = PWM_RANGE*3/4;
      break;
    case  '6':
      Serial.println("UP6");//jma test
      pwm_right = PWM_RANGE*3/4;
      break;
    case  '7':
      Serial.println("UP7");//jma test
      pwm_left = 0;
      pwm_right = 0;
      break;
    case  '8':
      Serial.println("UP8");//jma test
      Setpoint_left -= 1024;
      break;
    case  '9':
      Kp *= 1.1;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Kp: ");
      Serial.println(Kp);
      break;
    case  'D':
      Kp *= 0.9;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Kp: ");
      Serial.println(Kp);
      break;
    case  'A':
      Ki *= 1.1;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Ki: ");
      Serial.println(Ki);
      break;
    case  'E':
      Ki *= 0.9;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Ki: ");
      Serial.println(Ki);
      break;
    case  'B':
      Kd *= 1.1;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Kd: ");
      Serial.println(Kd);
      break;
    case  'F':
      Kd *= 0.9;
      myPID_left.SetTunings(Kp,Ki,Kd);
      myPID_right.SetTunings(Kp,Ki,Kd);
      Serial.print("Kd: ");
      Serial.println(Kd);
      break;
    default:
      Serial.print("Unknown Character: ");
      Serial.println(input);
      break;
    }
  }
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
  Serial.println("404 Not Found");
}
