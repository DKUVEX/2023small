#ifndef API_H
#define API_H

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
#include <eigen-3.4.0/Eigen/Dense>
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;
controller Controller1 = controller(primary);