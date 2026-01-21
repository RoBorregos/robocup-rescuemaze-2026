#include <Arduino.h>
int myFunction(int, int);

void setup() {
  int result = myFunction(2, 3);
}

void loop() {
}

int myFunction(int x, int y) {
  return x + y;
}