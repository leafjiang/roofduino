
// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.  Using a constant rather than a normal variable lets
// use this value to determine the size of the readings array.
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
long total = 0;                  // the running total
float average = 0;                // the average
int counter = 0;

int inputPin = A0;

void setup()
{
  Serial.begin(115200);                   
  for (int i = 0; i< numReadings; i++) readings[i] = 0;         
}

void loop() 
{
  total -= readings[index];
  readings[index] = analogRead(inputPin);
  total += readings[index];
  index++;
  counter = max(counter, index);
  if (index >= numReadings) index = 0;
  average = float(total) / counter;         

  //if (average> 400 || average< 300)
  //    Serial.println(average); 
  Serial.println(average);
}
