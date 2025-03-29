int value = 1;
int step = 10; // How much to increment/decrement each loop

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(value);
  value += step;

  if (value >= 1000 || value <= 1) {
    step = -step; // Reverse direction at the limits
  }

  delay(50); // Adjust to control how fast it cycles
}
