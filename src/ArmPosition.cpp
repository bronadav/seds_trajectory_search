/*
 * ArmPosition.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: yonadav
 */

#include "ArmPosition.h"


ArmPosition::~ArmPosition() {
}

ArmPosition::ArmPosition(double aJointPosition[],
		sKinematics& aSKinematicChain) {
	this->mJointPosition = aJointPosition;
	aSKinematicChain.setJoints(this->mJointPosition);
	aSKinematicChain.getEndPos(this->mTaskPosition);
}

ArmPosition::ArmPosition(double aJointPosition[], double aTaskPosition[]) {
	this->mJointPosition = aJointPosition;
	this->mTaskPosition = aTaskPosition;
}

double* ArmPosition::getJointPosition() const {
	double temp[] = mJointPosition;
	return temp;
}

double* ArmPosition::getTaskPosition() const {
	double temp[] = mTaskPosition;
	return temp;
}

ArmPosition operator -(const ArmPosition& p1, const ArmPosition& p2) {
	double* q1 = p1.getJointPosition(); double* q2 = p2.getJointPosition();
	double* t1 = p1.getTaskPosition(); double* t2 = p2.getTaskPosition();
	int qSize = sizeof(q1)/sizeof(*q1);
	int tSize = sizeof(t1)/sizeof(*t1);
	double newTaskPosition[];
	double newJointPosition[];
	for(int i =0; i<qSize; i++){
		newTaskPosition[i] = t1[i] - t2[i];
		newJointPosition[i] = q1[i] - q2[i];
	}
	return ArmPosition(newJointPosition, newTaskPosition);
}
