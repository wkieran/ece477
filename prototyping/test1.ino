int timer = 100;
int outputA = 6;
int outputB = 7;

int counter = 4;
int aState;
int aLastState;

void setup() {
  // put your setup code here, to run once:
  
  // // for LEDs
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // for rotary encoder
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);

  digitalWrite(4, HIGH);

  // Serial.begin(115200);
  aLastState = digitalRead(outputA);
}

void loop() {
  // put your main code here, to run repeatedly:
  // for(int i = 3; i < 6; i++) {
  //   digitalWrite(i, HIGH);
  //   delay(timer);
  //   digitalWrite(i, LOW);
  //   delay(timer);
  // }
  Serial.print("aState = ");
  Serial.println(aState);
  Serial.print("outputB = ");
  Serial.println(digitalRead(outputB));


  aState = digitalRead(outputA);
  if(aState != aLastState){
    if(digitalRead(outputB) != aState) {
      counter++;      
    }
    else {
      counter--;
    }
    counter = changeLED(counter);
    // Serial.print("Position: ");
    // Serial.println(counter);
  }
  aLastState = aState;
  delay(1);
  readPins();
}

int changeLED(int counter) {
  if(counter > 5) {
    counter = 3;
  }
  else if (counter < 3) {
    counter = 5;
  }
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(counter, HIGH);
  // Serial.print("increment counter: ");
  // Serial.println(counter);
  return counter;
}

void readPins(){
  // Serial.println(counter);
  Serial.print(digitalRead(3));
  Serial.print(digitalRead(4));
  Serial.print(digitalRead(5));
  Serial.print('\n');


}