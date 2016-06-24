#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <time.h>
#include <string>
#include <fstream>
#include <math.h>
#include <nlopt.hpp>
#include <omp.h>
#include "RobotLib/RobotInterface.h"

#include "MathLib/IKGroupSolver.h"

#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "RobotLib/PIDController.h"
#include "sKinematics.h"
#include <ctime>
#include "Gaussians.h"
#include <stdlib.h>

Matrix Trajectory;
Matrix Ttotal;
Robot *mRobot;
RevoluteJointSensorGroup    mSensorsGroup;
RevoluteJointActuatorGroup  mActuatorsGroup;
KinematicChain              mKinematicChain;
InverseDynamics             mInvDynamics;
IKGroupSolver               mIKSolver;
int  			mEndEffectorId;
int			DOFCount;
Vector                      mTorques;
Vector                      mGravityTorques;

Vector                      mJointPos;
Vector                      mJointDesPos;
Vector                      mJointTargetPos;

Vector                      mJointVel;
Vector                      mJointTmp;

Vector                      mJointVelLimits[2];

int                         mState;
int                         mMode;
Vector3 		tpos;
Vector3			mTargetError;
time_t tstart, tend;

int main()
{
	sKinematics mSKinematicChain(7, 1.0/500);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed

	//mSKinematicChain->setDH(0,  0.0,  0.31, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(110.0)*0.77);
	mSKinematicChain.setDH(0,  0.0,  0.36, M_PI_2, 0.0, 1,  DEG2RAD( -170.)*0.8, DEG2RAD( 170.)*0.8, DEG2RAD(98.0)*0.90);
	mSKinematicChain.setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*0.8, DEG2RAD( 120.)*0.8, DEG2RAD(98.0)*0.90);
	//mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0)*0.8);
	//mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(128.0)*0.30);
	mSKinematicChain.setDH(2,  0.0,  0.42,-M_PI_2, 0.0, 1,  DEG2RAD(-170.)*0.8, DEG2RAD(170.)*0.8, DEG2RAD(100.0)*0.90);
	mSKinematicChain.setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.)*0.8, DEG2RAD(120.)*0.8, DEG2RAD(130.0)*0.90);
	mSKinematicChain.setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.)*0.8, DEG2RAD(170.)*0.8, DEG2RAD(140.0)*0.90);
	//mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0));
	mSKinematicChain.setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*0.8, DEG2RAD( 120.)*0.8, DEG2RAD(180.0)*0.90); // reduced joint ang$
	//mSKinematicChain->setDH(6,  0.0,  0.20,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(132.0)*0.99);
	//mSKinematicChain->setDH(6,  0.0,  0.217,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95); // for barrett
	//mSKinematicChain->setDH(6, -0.06,  0.180,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95); // for sim lab
	mSKinematicChain.setDH(6, 0.0,  0.0984,    0.0, 0.0, 1,  DEG2RAD(-175.)*0.8, DEG2RAD(175.)*0.8, DEG2RAD(180.0)*0.90); // for sim lab
	//mSKinematicChain->setDH(6, 0.000,  0.00,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(184.0)*0.95); // for sim lab

	double pi=3.14;
	Vector3 Position;
	mJointTargetPos.Resize(7);
	mSKinematicChain.readyForKinematics();
	cout<<mSKinematicChain.getDOF()<<endl;
	ofstream PositionofEndEffector ("position_of_endeffector_of_robot.txt");
	PositionofEndEffector.precision(4);
	PositionofEndEffector.setf(ios::fixed);
	PositionofEndEffector.setf(ios::showpoint);
	ofstream VelocityofEndEffector ("velocity_of_endeffector_of_robot.txt");
	VelocityofEndEffector.precision(4);
	VelocityofEndEffector.setf(ios::fixed);
	VelocityofEndEffector.setf(ios::showpoint);
	//	ofstream OrientationofEndEffector ("OrientationofEndEffector.txt");
	Matrix Jacobian;
	Jacobian.Resize(3,7);
	Matrix Jacobian_A;
	Jacobian_A.Resize(3,7);
	Vector Dq_max;Dq_max.Resize(7);
	Vector Dq_min;Dq_min.Resize(7);
	Vector Dq_V;Dq_V.Resize(7);
	Matrix Dq;Dq.Resize(64,7);
	Dq_max(0)=DEG2RAD(110.0);	Dq_max(1)=DEG2RAD(110.0);	Dq_max(2)=DEG2RAD(128.0);
	Dq_max(3)=DEG2RAD(128.0);	Dq_max(4)=DEG2RAD(204.0);	Dq_max(5)=DEG2RAD(184.0);
	Dq_max(6)=0.0;
	Dq_min=Dq_max*(-1);
	int count=0;
	for (double Dq_1=-DEG2RAD(110.0);Dq_1<=DEG2RAD(110.0);Dq_1=Dq_1+2*DEG2RAD(110.0))
	{
		for (double Dq_2=-DEG2RAD(110.0);Dq_2<=DEG2RAD(110.0);Dq_2=Dq_2+2*DEG2RAD(110.0))
		{
			for (double Dq_3=-DEG2RAD(128.0);Dq_3<=DEG2RAD(128.0);Dq_3=Dq_3+2*DEG2RAD(128.0))
			{
				for (double Dq_4=-DEG2RAD(128.0);Dq_4<=DEG2RAD(128.0);Dq_4=Dq_4+2*DEG2RAD(128.0))
				{
					for (double Dq_5=-DEG2RAD(204.0);Dq_5<=DEG2RAD(204.0);Dq_5=Dq_5+2*DEG2RAD(204.0))
					{
						for (double Dq_6=-DEG2RAD(184.0);Dq_6<=DEG2RAD(184.0);Dq_6=Dq_6+2*DEG2RAD(184.0))
						{
							Dq(count,0)=Dq_1;
							Dq(count,1)=Dq_2;
							Dq(count,2)=Dq_3;
							Dq(count,3)=Dq_4;
							Dq(count,4)=Dq_5;
							Dq(count,5)=Dq_6;
							Dq(count,6)=Dq_6;
							count=count+1;
						}
					}
				}
			}
		}
	}
	//Dq_max.Print("Dq_max");
	Dq.Print("Dq");
	int count_max=count;
	cout<<"count_max "<<count_max<<endl;
	count=0;
	Vector V(3);
	Vector V_1(3);
	double norm=0;
	double norm_1=0;
	Jacobian_A.Zero();
	int coutn_a;
	for (double q1=-170*pi/180;q1<=170*pi/180;q1=q1+34*pi/180)
	{
		for (double q2=-120*pi/180;q2<=120*pi/180;q2=q2+24*pi/180)
		{
			for (double q3=-170*pi/180;q3<=170*pi/180;q3=q3+34*pi/180)
			{
				for (double q4=-120*pi/180;q4<=120*pi/180;q4=q4+24*pi/180)
				{
					for (double q5=-170*pi/180;q5<=170*pi/180;q5=q5+34*pi/180)
					{
						for (double q6=-120*pi/180;q6<=120*pi/180;q6=q6+24*pi/180)
						{
							for (double q7=-175*pi/180;q7<=175*pi/180;q7=q7+35*pi/180)
							{
								mJointTargetPos(0) = q1;
								mJointTargetPos(1) = q2;
								mJointTargetPos(2) = q3;
								mJointTargetPos(3) = q4;
								mJointTargetPos(4) = q5;
								mJointTargetPos(5) = q6;
								mJointTargetPos(6) = q7;
								mSKinematicChain.setJoints(mJointTargetPos.Array());
								mSKinematicChain.getEndPos(Position.Array());
								mSKinematicChain.getJacobianPos(Jacobian);
								Jacobian_A.Zero();
								if ((Position(2)>0.3))
								{
									if ((PositionofEndEffector.is_open())&&(VelocityofEndEffector.is_open()))
									{
										PositionofEndEffector<<Position(0)<<"  "<<Position(1)<<"   "<<Position(2)<<" "<<count<<std::endl;
									}
									for (int i=0;i<count_max;i++)
									{
										Dq.GetRow(i,Dq_V);
										V=Jacobian*Dq_V;
										norm=sqrt(V(0)*V(0)+V(1)*V(1)+V(2)*V(2));
										if (norm>2.1)
										{
											VelocityofEndEffector<<V(0)<<"  "<<V(1)<<"   "<<V(2)<<" "<<count<<" "<<norm<<std::endl;
										}
									}
								}
							}
							count=count+1;
						}
					}
				}
			}
		}
	}
	return 0;
}


/*

for (double i=0;i<3;i++)
{
	for (double j=0;j<7;j++)
	{
		Jacobian_A(i,j)=abs(Jacobian(i,j));
	}
}

Vector V(3);
V=Jacobian_A*Dq_max;
double norm=0;
norm=sqrt(V(0)*V(0)+V(1)*V(1)+V(2)*V(2));
if ((Position(2)>0.3))
{
	if (PositionofEndEffector.is_open())
	{
		PositionofEndEffector<<"  "<<Position(0)<<"  "<<Position(1)<<"   "<<Position(2)<<std::endl;
	}
}*/
