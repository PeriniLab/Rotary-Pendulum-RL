#include <Arduino.h>

double getTheta(long encoderPosition, long encoderSteps) {
  double theta = 0.0;
  long half_revolutions = encoderPosition/encoderSteps; 
  if (encoderPosition > 0) {
     if (half_revolutions > 1 && half_revolutions % 2 == 0){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions + 1)*encoderSteps, 31415,0) / 1e4;
     }
    else if (half_revolutions > 1 && half_revolutions % 2 == 1){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions + 1)*encoderSteps, 0, -31415) / 1e4;
     }
    else {
      theta = map(encoderPosition, 0, encoderSteps, 31415, 0) / 1e4;
    }
  }
  else { 
     if (half_revolutions < -1 && half_revolutions % 2 == 0){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions - 1)*encoderSteps, -31415,0) / 1e4;
     }
    else if (half_revolutions < -1 && half_revolutions % 2 == -1){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions - 1)*encoderSteps, 0, 31415) / 1e4;
     }
    else {
      theta = map(encoderPosition, 0, -encoderSteps, -31415, 0) / 1e4;
    }
  }
  return theta;
}

double clip(double n, double lower, double upper) {
  return max(lower, min(n, upper));
}