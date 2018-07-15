package org.usfirst.frc.team6394.robot;


import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
	
	/*Constants*/
	
	private final static int kPIDLoopIdx = 0;
	private final static int kTimeoutMs = 10;
	private final static int kCurrentLimit = 5;
	private final static int kAngleMotorCurrentLimit=10;
	private final static int kCurrentLimitDuration=10;

	//Elevator Position
	
	private static double TopDistance=15*10000.0;

	private final static double kTopDistance3=16.5*10000.0;
	private final static double kTopDistance2=15.5*10000.0;
	private final static double kTopDistance1=14.5*10000.0;
	private final static double kMidDistance=8.1*10000.0;
	private final static double kBotErrDistance=0.6*10000.0;
	private final static double kBotDistance=0;
	private final static double TolerentErrDistance=0.2*10000.0;
	private final static double TolerentErrVel=0.01*10000.0;
	
	//IntakerAngle
	private final static double maxAngle=4.06*10000;
	private final static double Angle_135=3.34*10000;
	private final static double Angle_90=1.9*10000;
	private final static double Angle_45=1.15*10000;
	private final static double minAngle=0.0*10000;
	
	private static double setAngle=0;
	
	private static int angle=0;
	
	
	/*Components*/
	
	private TalonSRX ElevatorMaster=new TalonSRX(pin.ElevatorMasterID);
	private VictorSPX ElevatorSlave=new VictorSPX(pin.ElevatorSlaveID);
	
	private TalonSRX AngleMotor=new TalonSRX(pin.AngleMotorID);
	
	private DigitalInput light=new DigitalInput(2);
	
	/*State*/
	
	private static double heightComp=0;
	
	private static boolean isTop=false;
	
	private static boolean isCalibrated=false;
	
	public Elevator() {
		//Invert the two motors
		ElevatorMaster.setInverted(true);
		ElevatorSlave.setInverted(true);
		
		//Initiation
		ElevatorSlave.follow(ElevatorMaster);
		TalonSRXInit(ElevatorMaster);
		configPID(ElevatorMaster,0,0.01,0,0);
		
		//HeightLimit
		ElevatorMaster.configForwardSoftLimitThreshold((int)kTopDistance3, Constants.kTimeoutMs);
		ElevatorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
		
		//Motor Safety
		ElevatorMaster.enableCurrentLimit(true);
		ElevatorMaster.configContinuousCurrentLimit(kCurrentLimit, kTimeoutMs);
		ElevatorMaster.configPeakCurrentLimit((int)(kCurrentLimit*2), kTimeoutMs);
		ElevatorMaster.configPeakCurrentDuration(kCurrentLimitDuration, kTimeoutMs);
		
		//Initiate AngleMotor
		TalonSRXInit(AngleMotor);
		configPID(AngleMotor,0,0.2,0,0.5);
				
		//AngleLimit
		AngleMotor.configForwardSoftLimitThreshold((int)maxAngle, kTimeoutMs);
		AngleMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
		
		//Motor Safety**
		AngleMotor.enableCurrentLimit(true);
		AngleMotor.configContinuousCurrentLimit(kAngleMotorCurrentLimit, kTimeoutMs);
		AngleMotor.configPeakCurrentLimit((int)(kAngleMotorCurrentLimit*2), kTimeoutMs*2);
		AngleMotor.configPeakCurrentDuration(kCurrentLimitDuration, kTimeoutMs);
	}
	
	public void changePos(int pos) {
		//Toggle the elevator
		switch(pos) {
		case 0:
			isTop=false;
			break;
		case 1:
			isTop=true;
			TopDistance=kTopDistance1+heightComp;
			break;
		case 2:
			isTop=true;
			TopDistance=kTopDistance2+heightComp;
			break;
		case 3:
			isTop=true;
			TopDistance=kTopDistance3;
			break;
		}
		setVerticalAngle();
		
	}
	
	public boolean isOrigin() {
		return setAngle==Angle_90;
	}
	
	public void toggleBotAngle() {
		//return false when vertical
		if(setAngle==Angle_90) {
			setOriginAngle();
		}else {
			configPID(AngleMotor,0,0.7,0,0.5);
			setAngle=Angle_90;
		}
	}
	
	public void setVerticalAngle() {
		configPID(AngleMotor,0,0.5,0,0.3);
		setAngle=Angle_90;
	}
	
	public void setBackAngle() {
		configPID(AngleMotor,0,0.7,0,0.6);
		setAngle=Angle_135;
	}
	
	public void setOriginAngle() {
		if(AngleMotor.getSelectedSensorPosition(0)>((Angle_90+Angle_135)/2)) {
			configPID(AngleMotor,0,0.5,0,0.8);	
		}else {
			configPID(AngleMotor,0,0.1,0,0.5);
		}
		setAngle=minAngle;
	}
	
	
	
	public void run() {
		//Maintain angle
		AngleMotor.set(ControlMode.Position, setAngle);

		//Move elevator
			if(isTop) {
				isCalibrated=false;
				configPID(ElevatorMaster,0,0.03,0,0);		
				ElevatorMaster.set(ControlMode.Position,TopDistance);
			}else {
				if(isCalibrated) {
					configPID(ElevatorMaster,0,0.02,0,0);
					ElevatorMaster.set(ControlMode.Position,kBotDistance);
				}else if(!light.get()) {
					configPID(ElevatorMaster,0,0.02,0,0);
					ElevatorMaster.set(ControlMode.Position,kBotDistance);
					ElevatorMaster.setSelectedSensorPosition((int)kBotDistance, kPIDLoopIdx, kTimeoutMs);
					isCalibrated=true;
				}else if(ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx)<kBotErrDistance) {
					ElevatorMaster.set(ControlMode.PercentOutput, -0.2);
					isCalibrated=false;
				}else if(ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx)<kMidDistance) {
					configPID(ElevatorMaster,0,0.012,0,0);
					ElevatorMaster.set(ControlMode.Position,kBotDistance);	
				}else {
					configPID(ElevatorMaster,0,0.006,0,0.03);
					if(AngleMotor.getSelectedSensorPosition(kPIDLoopIdx)<=(Angle_90*1.05)) {
						//Move to mid position
						ElevatorMaster.set(ControlMode.Position,kMidDistance*0.95);		
					}
					//set the angle to safe position
					setVerticalAngle();
				}
				
				
				/*if(ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx)<kMidDistance) {
					configPID(ElevatorMaster,0,0.008,0,0);
				}else {
					configPID(ElevatorMaster,0,0.005,0,0);
				}
				
				ElevatorMaster.set(ControlMode.Position, kBotDistance);
				*/
			}
		//Display Info
		SmartDashboard.putBoolean("Info_Elevator_isTop", isTop);
		SmartDashboard.putNumber("Info_Elevator_Distance", ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_Elevator_Current",ElevatorMaster.getOutputCurrent());
		SmartDashboard.putNumber("Info_Elevator_Vel", ElevatorMaster.getSelectedSensorVelocity(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_IntakerAngle", AngleMotor.getSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_IntakerAngleCurrent", AngleMotor.getOutputCurrent());

	}
	
	private void TalonSRXInit(TalonSRX _talon) {

		//set up TalonSRX and closed loop

		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				kPIDLoopIdx,kTimeoutMs);
		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1, kTimeoutMs);
		_talon.configPeakOutputReverse(-1, kTimeoutMs);

		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

	}

	private void configPID(TalonSRX _talon,double kF, double kP,double kI, double kD) {

		_talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		_talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		_talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		_talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

	}
}
