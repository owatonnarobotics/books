#pragma once

const int R_controllerPortPlayerOne = 0;
const int R_controllerPortPlayerTwo = 1;
const int R_controllerPortPlayerThree = 2;
const int R_controllerPortGuitar = 0;

//This deadzone is used to determine when the controller is completely motionless
const double R_deadzoneController = .15;
//And this one is to determine when rotation is being induced, as simply operation
//of the controller often results in errant rotation. Due to how easy it is to
//drift, it is significantly higher.
const double R_deadzoneControllerZ = 0.15;

const double R_smoothingFactor = 0.5;