/*
 * rrPID.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: Corey
 */

#include "rrPID.h"
#include <string.h>

rrPID::rrPID() {
	// TODO Auto-generated constructor stub

}

rrPID::~rrPID() {
	// TODO Auto-generated destructor stub
}

void rrPID::init(int newIDnum, float newKp, float newKi){
	IDnum = newIDnum;
	kp = newKp;
	ki = newKi;
}


void rrPID::setKpKi(float newKp, float newKi){
	kp = newKp;
	ki = newKi;
}

void rrPID::setLimits(float newIntLim, float newOutputLim){
	intLim = newIntLim;
	outputLim = newOutputLim;
}

float rrPID::controlOutput(float curTarget, float curActual){
	target = curTarget;
	actual = curActual;
	errorSignal = target - actual;
	integrationTotal = errorSignal + integrationTotal;
	if (integrationTotal < -intLim) integrationTotal = -intLim;
	if (integrationTotal > intLim) integrationTotal = intLim;
	outputCommand = (kp * errorSignal) + (ki * integrationTotal);
	if (outputCommand < -outputLim) outputCommand = -outputLim;
	if (outputCommand > outputLim) outputCommand = outputLim;
	return outputCommand;
}
float rrPID::motorOutput(){
	return outputCommand;
}
float rrPID::debug(){
	char dashBoardString[100];
	sprintf(dashBoardString,"PID:%d : rrError Signal",IDnum);
	SmartDashboard::PutNumber(dashBoardString, errorSignal);
	sprintf(dashBoardString,"PID:%d : rrIntegration Total",IDnum);
	SmartDashboard::PutNumber(dashBoardString, integrationTotal);
	sprintf(dashBoardString,"PID:%d : rrOutput Command",IDnum);
	SmartDashboard::PutNumber(dashBoardString, outputCommand);
	return 1.0;
}
