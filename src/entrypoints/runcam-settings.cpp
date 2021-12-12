#include <drivers/runcam/Runcam.h>

#include <iostream>

#include "miosix.h"

int main() {
  printf(R"(
            ___                                                      ___             _       _        _              __ _          
    o O O  | _ \   _  _    _ _      __     __ _    _ __      o O O  / __|    ___    | |_    | |_     (_)    _ _     / _` |   ___   
   o       |   /  | +| |  | ' \    / _|   / _` |  | '  \    o       \__ \   / -_)   |  _|   |  _|    | |   | ' \    \__, |  (_-<   
  TS__[O]  |_|_\   \_,_|  |_||_|   \__|_  \__,_|  |_|_|_|  TS__[O]  |___/   \___|   _\__|   _\__|   _|_|_  |_||_|   |___/   /__/_  
 {======|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""| {======|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""| 
./o--000'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'./o--000'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-' 
)");

  Thread::sleep(100);

  printf(
      "Press d to move down, c to confirm the action and o to open the menu\n");
  Thread::sleep(100);
  GpioPin tx(GPIOB_BASE, 6);
  GpioPin rx(GPIOB_BASE, 7);

  tx.mode(Mode::ALTERNATE);
  rx.mode(Mode::ALTERNATE);

  tx.alternateFunction(7);
  rx.alternateFunction(7);
  Runcam test(1);
  if (!test.init()) {
    return -1;
  }
  char c;

  while (true) {
    scanf("%c", &c);
    c = 'c';

    test.moveDown();
    test.selectSetting();
    test.openMenu();
    if (c == 'd') {
      test.moveDown();
    } else if (c == 'c') {
      test.selectSetting();
    } else if (c == 'o') {
      test.openMenu();
    }
  }
  test.close();

  return 0;
}