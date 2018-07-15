package org.usfirst.frc.team6394.robot;

public class Constants {
	
	/*Measurement*/
	
	public final static double rotate90=1.03;

	/*Constants*/
	
	public final static int kTimeoutMs = 10;
	public final static int kPIDLoopIdx = 0;
	public final static int kTurnTimeoutSec = 3;
	
	/****
	/* Below are constants that need to be adjusted
	/* V stands for velocity
	/* RV stands for rotation velocity
	*/
	
	
		//Encoder
	
	public final static double kRawMinVel=200;		//raw encoder value
	public final static double kRawMaxVel=3000.0;	//raw encoder value
	public final static double kMinVel=kRawMinVel/kRawMaxVel;	//normalized min velocity when robot starts to move
	
		//AHRS
	
	public final static double kMaxRate=300;	
	public final static double kSlowDownAngle=20;	//angle where robot starts to slow down
}