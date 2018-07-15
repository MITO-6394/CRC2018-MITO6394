/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

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

	@Override
	public void robotInit() {
		
	}
	
	@Override
	public void autonomousInit() {
		autoTimer.stop();
		autoTimer.reset();
		autoState=0;
	}
	
	@Override
	public void autonomousPeriodic() {
		switch(autoState) {
		case 0:
			if(mBase.Rotate(45, 0.3, 0.3)) {
				autoState++;
			}
			break;
		case 1:
			if(mBase.DisDrive(2, 0.7, false)) {
				autoState++;
			}
			break;
		case 2:
			if(mBase.Rotate(-45, 0.3, 0.3)) {
				autoState++;
			}
			break;
		case 3:
			autoTimer.start();
			if(autoTimer.get()>1) {
				autoTimer.stop();
				autoTimer.reset();
				autoState++;
			}
			break;
		case 4:
			if(mBase.Rotate(90, 0.3, 0.3)) {
				autoState++;
			}
			break;
		case 5:
			if(mBase.DisDrive(3, 0.7, false)) {
				autoState++;
			}
			break;
		case 6:
			if(mBase.Rotate(-90, 0.3, 0.3)) {
				autoState++;
			}
			break;
		case 7:
			if(mBase.DisDrive(3, 0.7, false)) {
				autoState++;
			}
			break;
		case 8:
			mBase.velDrive(0, 0, true);
			autoState++;
		}
		mBase.run();
		mIntaker.run();
		mElevator.run();
	}
	
	@Override
	public void teleopInit() {
		
	}

	
	@Override
	public void teleopPeriodic() {	
		
		//actionstick
		if(actionStick.getRawButtonPressed(buttons.elevatorDropBut)) {
			mElevator.changePos(0);
		}
		if(actionStick.getRawButtonPressed(buttons.elevatorRaisePos1But)) {
			mElevator.changePos(1);
		}
		if(actionStick.getRawButtonPressed(buttons.elevatorRaisePos2But)) {
			mElevator.changePos(2);
		}
		if(actionStick.getRawButtonPressed(buttons.elevatorRaisePos3But)) {
			mElevator.changePos(3);
		}
		if(actionStick.getRawButtonPressed(buttons.intakerBackAngleBut)) {
			mElevator.setBackAngle();
		}
		mIntaker.emitCube(actionStick.getRawAxis(buttons.intakerEmitAxis));
		
		//movestick
		if(moveStick.getRawButtonPressed(buttons.intakerChangeModeBut)) {
			if(mElevator.isOrigin()) {
				mIntaker.toggleMode(4);
			}else {
				mElevator.toggleBotAngle();
				mIntaker.toggleMode(1);
			}
		}
		if(moveStick.getRawButtonPressed(buttons.intakerToggleAngleBut)) {
			mElevator.toggleBotAngle();
		}
		mIntaker.setLargeAngle(moveStick.getRawButton(5));
		
		mBase.velDrive(actionStick.getY(), actionStick.getX(),true);
		mBase.run();
		mIntaker.run();
		mElevator.run();
		
	}

	
}
