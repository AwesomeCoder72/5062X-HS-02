#include "ladybrown.hpp"
#include "main.h"

void spin_lady_brown_driver(int ladyBrownUpButtonValue, int ladyBrownDownButtonValue) {

  if (((ladyBrownUpButtonValue == 1) && (ladyBrownDownButtonValue == 1)) || 
      ((ladyBrownUpButtonValue == 0) && (ladyBrownDownButtonValue == 0)))  {
        LadyBrownMech.move_voltage(0);
      } else if (ladyBrownUpButtonValue == 1) {
        LadyBrownMech.move_voltage(12000);
      } else if (ladyBrownDownButtonValue == 1) {
        LadyBrownMech.move_voltage(-12000);
      }
}

