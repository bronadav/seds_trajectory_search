/*
 * JointPosition.h
 *
 *  Created on: Jun 23, 2016
 *      Author: yonadav
 */
#include "sKinematics.h"

#ifndef GPLVM_EXPERIMENT_SEDS_TRAJECTORY_SEARCH_SRC_ARMPOSITION_H_
#define GPLVM_EXPERIMENT_SEDS_TRAJECTORY_SEARCH_SRC_ARMPOSITION_H_

class ArmPosition {
private:
	double mJointPosition[];
	double mTaskPosition[];
	ArmPosition(double aJointPosition[], double aTaskPosition[]);
public:
	virtual ~ArmPosition();
	ArmPosition(double aJointPosition[], sKinematics& aSKinematicChain);
	double* getJointPosition() const;
	double* getTaskPosition()const;
};

ArmPosition operator-(const ArmPosition& p1, const ArmPosition& p2);

#endif /* GPLVM_EXPERIMENT_SEDS_TRAJECTORY_SEARCH_SRC_ARMPOSITION_H_ */
