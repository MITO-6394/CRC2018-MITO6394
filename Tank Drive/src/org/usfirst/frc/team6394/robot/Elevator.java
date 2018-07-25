package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;

public class Elevator {

	/* Constants */

	private final static int kPIDLoopIdx = 0;
	private final static int kTimeoutMs = 10;
	private final static int kCurrentLimit = 9;
	private final static int kAngleMotorCurrentLimit = 12;
	private final static int kCurrentLimitDuration = 10;

	private final static double kMovePower = 0.15, kBalancePower = 0.22;

	// Elevator Position

	private static double TopDistance = 15 * 10000.0;

	private final static double kTopDistance3 = 16.5 * 10000.0;
	private final static double kTopDistance2 = 13.5 * 10000.0;
	private final static double kTopDistance1 = 11.5 * 10000.0;
	private final static double kMidDistance = 8.1 * 10000.0;
	private final static double kSwitchDistance = 2.1 * 10000.0;
	private final static double kBotErrDistance = 0.6 * 10000.0;
	private final static double kBotDistance = 0;
	private final static double TolerentErrDistance = 0.12 * 10000.0;
	private final static double TolerentErrVel = 0.01 * 10000.0;

	// IntakerAngle
	private final static double maxAngle = 4.06 * 10000;
	private final static double Angle_135 = 3.34 * 10000;
	private final static double Angle_90 = 3.3 * 10000;
	private final static double Angle_45 = 1.15 * 10000;
	private final static double Angle_start = 0;
	private final static double Angle_0 = 0.15 * 10000;
	private final static double Total_Angle_0 = 0.5 * 10000;
	private static double Angle_Err = -0.3 * 10000.0;

	private static double setAngle = Angle_start;

	/* Components */

	private TalonSRX ElevatorMaster = new TalonSRX(pin.ElevatorMasterID);
	private VictorSPX ElevatorSlave = new VictorSPX(pin.ElevatorSlaveID);

	private TalonSRX AngleMotor = new TalonSRX(pin.AngleMotorID);

	private AnalogInput elevatorLight = new AnalogInput(0);
	private AnalogInput angleLight = new AnalogInput(1);

	/* State */

	private static double heightComp = 0;

	private static boolean angleIsCalibrated = false;

	private static boolean isTop = false;

	private static boolean isCalibrated = false;

	public static boolean hasCube = false;

	public static boolean manualMode = false;

	public static boolean angleManualMode = false;

	public Elevator() {
		ElevatorMaster.setNeutralMode(NeutralMode.Brake);
		ElevatorSlave.setNeutralMode(NeutralMode.Brake);
		AngleMotor.setNeutralMode(NeutralMode.Brake);

		// Invert the two motors
		ElevatorMaster.setInverted(true);
		ElevatorSlave.setInverted(true);

		// Initiation
		ElevatorSlave.follow(ElevatorMaster);
		TalonSRXInit(ElevatorMaster);
		configPID(ElevatorMaster, 0, 0.005, 0, 0);
		ElevatorMaster.configClosedLoopPeakOutput(0, 0.70, Constants.kTimeoutMs);

		// HeightLimit
		ElevatorMaster.configForwardSoftLimitThreshold((int) kTopDistance3, Constants.kTimeoutMs);
		ElevatorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);

		// Motor Safety
		ElevatorMaster.enableCurrentLimit(true);
		ElevatorMaster.configContinuousCurrentLimit(kCurrentLimit, kTimeoutMs);
		ElevatorMaster.configPeakCurrentLimit((int) (kCurrentLimit * 2), kTimeoutMs);
		ElevatorMaster.configPeakCurrentDuration(kCurrentLimitDuration, kTimeoutMs);

		// Initiate AngleMotor
		TalonSRXInit(AngleMotor);
		configPID(AngleMotor, 0, 0.2, 0, 0.5);

		// AngleLimit
		AngleMotor.configClosedLoopPeakOutput(0, 0.6, Constants.kTimeoutMs);
		AngleMotor.configForwardSoftLimitThreshold((int) maxAngle, kTimeoutMs);
		AngleMotor.configReverseSoftLimitThreshold((int) (Angle_0 + Angle_Err), kTimeoutMs);
		AngleMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
		AngleMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
		AngleMotor.configClosedLoopPeakOutput(0, 0.5, Constants.kTimeoutMs);

		// Motor Safety**
		AngleMotor.enableCurrentLimit(true);
		AngleMotor.configContinuousCurrentLimit(kAngleMotorCurrentLimit, kTimeoutMs);
		AngleMotor.configPeakCurrentLimit((int) (kAngleMotorCurrentLimit * 2), kTimeoutMs);
		AngleMotor.configPeakCurrentDuration(kCurrentLimitDuration, kTimeoutMs);
	}

	public boolean isAngleCalibrated() {
		return angleIsCalibrated;
	}

	public void changePos(int pos) {
		// Toggle the elevator
		switch (pos) {
		case 0:
			isTop = false;
			break;
		case 1:
			isTop = true;
			TopDistance = kTopDistance1 + heightComp;
			break;
		case 2:
			isTop = true;
			TopDistance = kTopDistance2 + heightComp;
			break;
		case 3:
			isTop = true;
			TopDistance = kTopDistance3;
			break;
		case 4:
			isTop = true;
			TopDistance = kSwitchDistance;
		}
	}

	public void relativeChangePos(double pos) {
		heightComp += pos;
		TopDistance = (TopDistance + heightComp);
		TopDistance = TopDistance < kTopDistance3 ? TopDistance : kTopDistance3;
	}

	public boolean isOrigin() {
		return setAngle == Angle_0;
	}

	public boolean isTop() {
		return isTop;
	}

	public void setInitialVertical() {
		configPID(AngleMotor, 0, 0.4, 0, 0.6);
		setAngle = 2.5 * 10000;
	}

	public void toggleBotAngle() {
		// return false when vertical
		angleIsCalibrated = false;
		if (setAngle > (0.5 * Angle_90)) {
			setOriginAngle();
		} else {
			setVerticalAngle();
		}
	}

	public void setVerticalAngle() {
		angleIsCalibrated = false;
		if (hasCube) {
			configPID(AngleMotor, 0, 0.6, 0, 0.6);
		} else {
			configPID(AngleMotor, 0, 0.4, 0, 0.6);
		}
		setAngle = Angle_90;
	}

	public void setBackAngle() {
		angleIsCalibrated = false;
		if (ElevatorMaster.getSelectedSensorPosition(0) >= kMidDistance) {
			configPID(AngleMotor, 0, 0.5, 0, 0.6);
			setAngle = Angle_135;
		}
	}

	public void setOriginAngle() {
		if (hasCube) {
			configPID(AngleMotor, 0, 0.5, 0, 0.6);
		} else {
			if (AngleMotor.getSelectedSensorPosition(0) > ((Angle_90 + Angle_135) / 2)) {
				configPID(AngleMotor, 0, 0.5, 0, 0.8);
			} else {
				configPID(AngleMotor, 0, 0.4, 0, 0.4);
			}
		}
		setAngle = Angle_0;
	}

	public void disableManualMove() {
		// manualMode=false;
		// TopDistance=ElevatorMaster.getSelectedSensorPosition(0)+200;
	}

	public void setAngleCalibrated() {
		AngleMotor.setSelectedSensorPosition(kPIDLoopIdx, (int) Angle_start, kTimeoutMs);
		angleIsCalibrated = true;
	}

	public void setElevatorCalibrated() {
		ElevatorMaster.setSelectedSensorPosition((int) kBotDistance, kPIDLoopIdx, kTimeoutMs);
		isCalibrated = true;
	}

	public void manualMove(double value) {
		// manualMode=true;
		// isCalibrated=false;
		// if(ElevatorMaster.getSelectedSensorPosition(0)<kMidDistance) {
		// ElevatorMaster.set(ControlMode.PercentOutput,value);
		// }else if(ElevatorMaster.getSelectedSensorPosition(0)<kTopDistance3) {
		// ElevatorMaster.set(ControlMode.PercentOutput,value+kBalancePower);
		// }else{
		// ElevatorMaster.set(ControlMode.PercentOutput,0);
		// }

	}

	public void manualAngle(double value) {
		AngleMotor.configReverseSoftLimitEnable(false, kTimeoutMs);
		angleManualMode = true;
		isCalibrated = false;
		configPID(AngleMotor, 0, 0.6, 0, 0);
		AngleMotor.set(ControlMode.Velocity, value * Constants.kRawAngleMaxVel);
	}

	public void disableManualAngle() {
		AngleMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
		angleManualMode = false;
		setAngle = AngleMotor.getSelectedSensorPosition(0);
		configPID(AngleMotor, 0, 0.6, 0, 0.6);
	}

	public void run() {
		// adjust current
		if (hasCube) {
			AngleMotor.configReverseSoftLimitThreshold((int) (Angle_0), kTimeoutMs);
			AngleMotor.configClosedLoopPeakOutput(0, 0.5, Constants.kTimeoutMs);

		} else {
			AngleMotor.configReverseSoftLimitThreshold((int) (Angle_0 + Angle_Err), kTimeoutMs);
			AngleMotor.configClosedLoopPeakOutput(0, 0.5, Constants.kTimeoutMs);
		}

		// Maintain angle
		if (!angleManualMode) {
			if ((setAngle == Angle_0) & (!angleIsCalibrated) & (!hasCube)) {
				if (angleLight.getAverageVoltage() < 3.5) {
					AngleMotor.setSelectedSensorPosition(kPIDLoopIdx, (int) Angle_start, kTimeoutMs);
					angleIsCalibrated = true;
					Angle_Err = -0.3 * 10000.0;
				} else {
					AngleMotor.set(ControlMode.PercentOutput, -0.32);
				}
			} else {
				if (hasCube & (setAngle == Angle_0)) {
					if (!util.isWithin(AngleMotor.getSelectedSensorPosition(0), Total_Angle_0, 0.2 * 10000)) {
						configPID(AngleMotor, 0, 0.07, 0, 0);
						AngleMotor.set(ControlMode.Velocity,
								(AngleMotor.getSelectedSensorPosition(0) - Total_Angle_0) * 0.08);
					} else {
						configPID(AngleMotor, 0, 0.6, 0, 0.6);
						AngleMotor.set(ControlMode.Position, Total_Angle_0);
					}
				} else {
					AngleMotor.set(ControlMode.Position, setAngle);
				}
			}
		}

		// Move elevator
		if (!manualMode) {
			if (isTop) {
				isCalibrated = false;
				ElevatorMaster.configContinuousCurrentLimit((int) (kCurrentLimit * 2.3), kTimeoutMs);
				ElevatorMaster.configPeakCurrentLimit((int) (kCurrentLimit * 3), kTimeoutMs);
				ElevatorMaster.configClosedLoopPeakOutput(0, 0.9, Constants.kTimeoutMs);
				configPID(ElevatorMaster, 0, 0.7, 0, 0.03);
				ElevatorMaster.set(ControlMode.Position, TopDistance);
			} else {
				if (isCalibrated) {
					configPID(ElevatorMaster, 0, 0.04, 0, 0);
					ElevatorMaster.configContinuousCurrentLimit((int) (kCurrentLimit * 0.6), kTimeoutMs);
					ElevatorMaster.configPeakCurrentLimit((int) (kCurrentLimit * 1.5), kTimeoutMs);
					ElevatorMaster.configClosedLoopPeakOutput(0, 0.3, Constants.kTimeoutMs);
					ElevatorMaster.set(ControlMode.PercentOutput, -0.1);
				} else {
					ElevatorMaster.configContinuousCurrentLimit(kCurrentLimit, kTimeoutMs);
					ElevatorMaster.configPeakCurrentLimit((int) (kCurrentLimit * 2), kTimeoutMs);
					ElevatorMaster.configClosedLoopPeakOutput(0, 0.65, Constants.kTimeoutMs);
					if (elevatorLight.getAverageVoltage() < 3.5) {
						configPID(ElevatorMaster, 0, 0.025, 0, 0);
						ElevatorMaster.set(ControlMode.Position, kBotDistance);
						ElevatorMaster.setSelectedSensorPosition((int) kBotDistance, kPIDLoopIdx, kTimeoutMs);
						isCalibrated = true;
					} else if (ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx) < kMidDistance * 0.2) {
						ElevatorMaster.set(ControlMode.PercentOutput, -0.2);
						isCalibrated = false;
					} else if (ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx) < kMidDistance) {
						configPID(ElevatorMaster, 0, 0.006, 0, 0);
						ElevatorMaster.set(ControlMode.Position, kBotDistance - kBotErrDistance);
					} else {
						configPID(ElevatorMaster, 0, 0.006, 0, 0.03);
						if (AngleMotor.getSelectedSensorPosition(kPIDLoopIdx) <= (Angle_90 * 1.05)) {
							// Move to mid position
							ElevatorMaster.set(ControlMode.Position, kMidDistance * 0.95);
						} else {
							setVerticalAngle();
						}
						// set the angle to safe position
					}
				}
			}
		}

		/*
		 * if(ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx)<kMidDistance) {
		 * configPID(ElevatorMaster,0,0.008,0,0); }else {
		 * configPID(ElevatorMaster,0,0.005,0,0); }
		 * 
		 * ElevatorMaster.set(ControlMode.Position, kBotDistance);
		 */
		// Display Info
		SmartDashboard.putBoolean("Info_Elevator_isTop", isTop);
		SmartDashboard.putBoolean("Info_Elevator_Calibrated", isCalibrated);
		SmartDashboard.putBoolean("Info_Elevator_elevatorLight", elevatorLight.getVoltage() < 3.5);
		SmartDashboard.putBoolean("Info_Elevator_AngleCalibrated", angleIsCalibrated);
		SmartDashboard.putNumber("Info_Elevator_Distance", ElevatorMaster.getSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_Elevator_PWM", ElevatorMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Info_Elevator_Current", ElevatorMaster.getOutputCurrent());
		SmartDashboard.putNumber("Info_Elevator_Vel", ElevatorMaster.getSelectedSensorVelocity(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_IntakerAngleLight", angleLight.getVoltage());
		SmartDashboard.putNumber("Info_IntakerAngle", AngleMotor.getSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_IntakerAngleCurrent", AngleMotor.getOutputCurrent());

	}

	private void TalonSRXInit(TalonSRX _talon) {

		// set up TalonSRX and closed loop

		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1, kTimeoutMs);
		_talon.configPeakOutputReverse(-1, kTimeoutMs);

		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

	}

	private void configPID(TalonSRX _talon, double kF, double kP, double kI, double kD) {

		_talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		_talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		_talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		_talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

	}
}
