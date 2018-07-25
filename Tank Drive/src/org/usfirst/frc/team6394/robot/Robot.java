/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
	
	Joystick actionStick=new Joystick(0);
	Joystick moveStick=new Joystick(1);
	Elevator mElevator=new Elevator();
	Intaker mIntaker=new Intaker();
	Base mBase=new Base();
	
	private static int autoState=0;
	private Timer autoTimer=new Timer();
	private Timer initTimer=new Timer();
	private Timer raiseTimer=new Timer();
	
	private boolean isRight=false;
	private boolean firstRaise=true;

	@Override
	public void robotInit() {
		Climb.Off();
		CameraServer.getInstance().startAutomaticCapture();
//		initTimer.delay(1.0);
		mElevator.setInitialVertical();
		
		
	}
	
	@Override
	public void autonomousInit() {
		autoTimer.stop();
		autoTimer.reset();
		autoTimer.start();
		autoState=0;		
	}
	
	@Override
	public void autonomousPeriodic() {
		isRight=DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='R';
//		if(isRight) {
//			if(autoTimer.get()<1) {
//				mBase.velDrive(0,0.2,false);
//			}else if(autoTimer.get()<2) {
//				mBase.velDrive(0.4,0,false);
//			}else if(autoTimer.get()<3) {
//				mBase.velDrive(0,0.2,false);
//			}else if(autoTimer.get()<4) {
//				mBase.velDrive(0,-0.4,false);
//			}else if(autoTimer.get()<6) {
//				mBase.velDrive(0.35,0,false);
//			}else if(autoTimer.get()<6.5) {
//				mBase.velDrive(0, 0, false);
//				if(firstRaise) {
//					firstRaise=false;
//					mElevator.changePos(4);
//					mElevator.toggleBotAngle();
//				}
//			}else if(autoTimer.get()<7){
//				mBase.velDrive(0,0,true);
//				mIntaker.emitCube(0.8);
//			}else {
//				mBase.velDrive(0, 0, false);
//			}
//		}else {
//			mBase.velDrive(0,0.2,true);
//		}
//		if(isRight) {
//			switch(autoState) {
//			case 0:
//				if(mBase.Rotate(45, 0.05, 0)) {
//					autoState++;
//					mBase.UpdateDistance();
//				}break;
//			case 1:
//				if(mBase.DisDrive(2, 0.2, false)) {
//					autoState++;
//				}break;
//			case 2:
//				if(mBase.Rotate(0, 0.05, 0)) {
//					mBase.UpdateDistance();
//					autoTimer.reset();
//					autoState++;
//				}break;
//			case 3:
//				if(mBase.DisDrive(4, 0.2, false)||autoTimer.get()>5) {
//					autoState++;
//				}break;
//			case 4:
//				mIntaker.emitCube(0.8);
//				mBase.velDrive(0, 0, false);
//			}
//		}
		
		mBase.velDrive(0.3, 0, false);
		
		mBase.run();
		mIntaker.run();
		mElevator.run();
	}
	
	@Override
	public void teleopInit() {
	}

	
	@Override
	public void teleopPeriodic() {
		
		raiseTimer.start();
		
		//actionstick
//		if(actionStick.getRawButton(buttons.elevatorRaiseBut)) {
//			mElevator.manualMove(0.2);
//		}
//		if(actionStick.getRawButton(buttons.elevatorDropBut)) {
//			mElevator.manualMove(-0.2);
//		}
//		if(actionStick.getRawButtonReleased(buttons.elevatorRaiseBut)||actionStick.getRawButtonReleased(buttons.elevatorDropBut)) {
//			mElevator.disableManualMove();
//		}
		if(actionStick.getRawButtonPressed(buttons.intakerBackAngleBut)) {
			mElevator.setBackAngle();
		}
		if(actionStick.getRawButtonPressed(buttons.intakerToggleAngleBut_action)) {
			mElevator.toggleBotAngle();
		}
		mIntaker.emitCube(actionStick.getRawAxis(buttons.intakerEmitAxis));
		SmartDashboard.putNumber("timer", raiseTimer.get());
			switch(actionStick.getPOV()) {
			case 0:
				mElevator.changePos(2);
				break;
			case 90:
				mElevator.changePos(3);
				break;
			case 180:
				mElevator.setVerticalAngle();
				mElevator.changePos(0);
				break;
			case 225:
				mElevator.changePos(4);
				break;
			case 270:
				mElevator.changePos(1);
				break;
			}
		//movestick
//		if(moveStick.getRawButton(buttons.elevatorForceRaiseBut)) {
//			mElevator.manualMove(0.6);
//		}
//		if(moveStick.getRawButton(buttons.elevatorForceDropBut)) {
//			mElevator.manualMove(-0.6);
//		}
//		if(moveStick.getRawButtonReleased(buttons.elevatorForceRaiseBut)
//				||moveStick.getRawButtonReleased(buttons.elevatorForceDropBut)) {
//			mElevator.disableManualMove();
//		}
		switch(moveStick.getPOV()) {
			case 270:
				mElevator.setAngleCalibrated();
				break;
			case 90:
				mElevator.setElevatorCalibrated();
				break;	
			}
		if(moveStick.getRawButton(buttons.angleManual)) {
			mElevator.manualAngle(-moveStick.getRawAxis(3));
		}
		if(moveStick.getRawButtonReleased(buttons.angleManual)) {
			mElevator.disableManualAngle();
		}
		
		if(moveStick.getRawButton(buttons.climbBut)) {
			if(moveStick.getRawButton(buttons.dropBut)) {
				Climb.climb(0.12);
			}else {
				Climb.climb(1);
			}
		}else if(moveStick.getRawButton(buttons.dropBut)){
			Climb.climb(-1);
		}else {
			Climb.climb(0);
		}
		
		mIntaker.setLargeAngle(moveStick.getRawButton(buttons.intakerLargeAngleBut));
		if(moveStick.getRawButtonPressed(buttons.intakerChangeModeBut)) {
			mIntaker.toggleMode(4);
		}
		if(moveStick.getRawButtonPressed(buttons.intakerToggleAngleBut)) {
			mElevator.toggleBotAngle();
		}
		
		
		
		if(moveStick.getRawButton(buttons.LiftUpBut)&&(!moveStick.getRawButton(buttons.LiftDownBut))) {
			Climb.moveHook(0.5);
		}else if(moveStick.getRawButton(buttons.LiftDownBut)&&(!moveStick.getRawButton(buttons.LiftUpBut))) {
			Climb.moveHook(-0.5);
		}else if(moveStick.getRawButton(buttons.LiftDownBut)&&moveStick.getRawButton(buttons.LiftUpBut)){
			Climb.moveHook(0.3);
		}else {
			Climb.moveHook(0.0);
		}
		
		if(mIntaker.isRaiseNeeded()) {
			mElevator.toggleBotAngle();
			mIntaker.isRaiseFinished();
		}
		 if(mElevator.isTop()) {
			 Constants.kRawMaxVel=1000;
		 }else {
			 Constants.kRawMaxVel=3000;
		 }
		 
		 if(moveStick.getRawButtonReleased(buttons.slowModeBut)) {
				Constants.kRawMaxVel=3000;
			}
			if(moveStick.getRawButtonReleased(buttons.medModeBut)) {
				Constants.kRawMaxVel=3000;
			}
			if(moveStick.getRawButton(buttons.medModeBut)) {
				Constants.kRawMaxVel=1500;
			}
			if(moveStick.getRawButton(buttons.slowModeBut)) {
				Constants.kRawMaxVel=1000;
			}
		mElevator.hasCube=mIntaker.hasCube();
		mBase.velDrive(-moveStick.getY(), -moveStick.getX(),false);
		mBase.run();
		mIntaker.run();
		mElevator.run();
		
	}
	
	@Override
	public void disabledInit() {
	}
	
	@Override
	public void disabledPeriodic() {
	}
	
}