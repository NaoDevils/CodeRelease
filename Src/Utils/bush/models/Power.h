#pragma once

class Power
{
public:
  int value;
  bool powerPlugged = false;
  bool naoqi = true;

  Power();
  Power(int value, bool powerPlugged, bool naoqi = true);

  bool isValid();
  
};
