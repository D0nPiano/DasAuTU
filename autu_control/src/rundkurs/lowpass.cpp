#include "autu_control/rundkurs/lowpass.h"

Lowpass::Lowpass(size_t maxValues) : maxValues(maxValues) {}

void Lowpass::addValue(float value) {
  if (values.size() >= maxValues)
    values.pop_front();

  values.push_back(value);
}

float Lowpass::getAverage() const {
  float avg = 0;
  for (float value : values)
    avg += value;
  return avg / values.size();
}
