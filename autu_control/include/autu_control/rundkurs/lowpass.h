#ifndef LOWPASS_H
#define LOWPASS_H

#include <cstddef>
#include <list>

class Lowpass {
public:
  Lowpass(size_t maxValues);
  void addValue(float value);
  float getAverage() const;

private:
  std::list<float> values;
  size_t maxValues;
};

#endif // LOWPASS_H
