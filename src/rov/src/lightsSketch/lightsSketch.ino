
int pin = 5;
String n ="";
int on = 0;
void setup(){
  pinMode(pin,OUTPUT);
  analogWrite(pin,0);
  Serial.begin(9600);
}
void loop(){
  //int on = 0;
  n = "";
  while(Serial.available() > 0){
    char recieved = Serial.read();
    if(recieved == '1'){
     on = 1;
    }
    else if(recieved == '0'){
     on = 0; 
    }
    else if(recieved == '2'){
     on = 2; 
    }
  }

  if(on == 1){
    analogWrite(pin,255);
  }
  else if(on == 0){
    analogWrite(pin,0);
  }
  else if(on == 2){
    blinkOnce(pin);
  }
}
void blinkOnce(int pin){
  analogWrite(pin,255);
  delay(500);
  analogWrite(pin,0);
  delay(500);
}
