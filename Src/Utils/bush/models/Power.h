#pragma once

class Power
{
public:
  int value;

  bool powerPluged;

  Power();

  Power(int value);

  Power(bool powerPluged);

  bool isValid();
  
};
