#ifndef TEST_H
#define TEST_H


#include "definations.cpp"

void test_connection(){
    if (BA) {
      Brain.Screen.printAt(10, 120, "A");  
    }     
    else if (BB) {
        Brain.Screen.printAt(10, 120, "B");
    }
    else if (BX) {
        Brain.Screen.printAt(10, 120, "X");
    }
    else if (BY) {
        Brain.Screen.printAt(10, 120, "Y");
    }
    else if (L1) {
        Brain.Screen.printAt(10, 120, "L1");
    }
    else if (L2) {
        Brain.Screen.printAt(10, 120, "L2");
    }
    else if (R1) {
        Brain.Screen.printAt(10, 120, "R1");
    }
    else if (R2) {
        Brain.Screen.printAt(10, 120, "R2");
    }
    else if (UP) {
        Brain.Screen.printAt(10, 120, "UP");
    }
    else if (DOWN) {
        Brain.Screen.printAt(10, 120, "DOWN");
    }
    else if (RIGHT) {
        Brain.Screen.printAt(10, 120, "RIGHT");
    }
    else if (LEFT) {
        Brain.Screen.printAt(10, 120, "LEFT");
    }
}