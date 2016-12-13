#pragma once
#include "ros/ros.h"

class AutoController {
  public:
    virtual ~AutoController(){};
    virtual void run() = 0; // Pure virtual function makes
};