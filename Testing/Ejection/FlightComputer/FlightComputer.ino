bool drogeState = false;
const int drogePin = 13;

bool mainState = false;
const int mainPin = 13;



void setup() {
  // put your setup code here, to run once:
  pinMode(drogePin, OUTPUT);
  digitalWrite(drogePin, drogeState);
//  pinMode(mainPin, OUTPUT);
//  digitalWrite(mainPin, mainState);
  
  // pinMode(drogeInputPin, INPUT);
  // pinMode(mainInputPin, INPUT);


  delay(10000);

  //drogeInput = digitalRead(drogeInputPin);
  //mainInput = digitalRead(mainInputPin);
  digitalWrite(drogePin, 1);
  //digitalWrite(mainInput, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
