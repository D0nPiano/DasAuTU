#ifndef LOWPASS_H
#define LOWPASS_H

#include <cstddef>
#include <list>

/**
* @brief implements a abstract lowpass
* calculates average of given count of last values
*/
class Lowpass {
public:
  Lowpass(size_t maxValues);
  /**
  * @brief add one Value and delete last Value if necessary
  */
  void addValue(float value);
  /**
  *@brief get Average
  */
  float getAverage() const;

private:
	//list of values
  std::list<float> values;
  //size of values
  size_t maxValues;
};

#endif // LOWPASS_H
