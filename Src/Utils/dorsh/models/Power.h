#pragma once

class Power
{
public:
  int value;
  bool powerPlugged = false;

  Power();
  Power(int value, bool powerPlugged);

  bool isValid();
  
};
