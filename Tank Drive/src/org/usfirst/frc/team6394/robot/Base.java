package org.usfirst.frc.team6394.robot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class Base implements PIDOutput,PIDSource{

	TalonSRX LMotorMaster=new TalonSRX(pin.BaseLMotorMasterID);
	TalonSRX RMotorMaster=new TalonSRX(pin.BaseRMotorMasterID);
	VictorSPX LMotorSlave=new VictorSPX(pin.BaseLMotorSlaveID);
	VictorSPX RMotorSlave=new VictorSPX(pin.BaseRMotorSlaveID);

	private final AHRS ahrs = new AHRS(Port.kMXP);
	private double pos_X=0;
	private double pos_Y=0;
	
	private static double last_dis=0;
	private static double this_dis=0;
	
	private static double LDis=0;
	private static double RDis=0;
	
	private PIDController turnController;
	
	
	/*Constants*/
	
	private double tolErrAngle=2.0;
	private double disErr=600;

	public Base() {

		/** Initiate Closed-loop system*/
		
		TalonSRXInit(LMotorMaster);
		TalonSRXInit(RMotorMaster);
		
		LMotorSlave.setInverted(true);
		LMotorMaster.setInverted(true);

		LMotorSlave.follow(LMotorMaster);
		RMotorSlave.follow(RMotorMaster);
		
		resetSensor();
		
		turnController=new PIDController(0.03, 0, 0, 0, this, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(tolErrAngle);
		turnController.setContinuous(true);

	}


	public AHRS getAHRS() {return  ahrs;}
	public double getX(){return pos_X;}	
	public double getY(){return pos_Y;}
	
	public void run() {
		UpdateMap();
		DisplayInfo();
	}
	

	public void UpdateMap(){

		this_dis=(LMotorMaster.getSelectedSensorPosition(Constants.kPIDLoopIdx)
			+RMotorMaster.getSelectedSensorPosition(Constants.kPIDLoopIdx))/2;

		double delta_dis=this_dis-last_dis;

		pos_X+=Math.cos(ahrs.getAngle())*delta_dis;
		pos_Y+=Math.sin(ahrs.getAngle())*delta_dis;
		last_dis=this_dis;
	}
	
	public void velDrive(double forward, double turn, boolean smoothMode) {
		/****** speed mode : clockwise is positive */
		/* Convert 500 RPM to units / 100ms.
		 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		configPID(LMotorMaster,0.3,0.7,0,0.3);
		configPID(RMotorMaster,0.3,0.7,0,0.3);

		double LTempV=(forward*Math.abs(forward)+turn*Math.abs(turn))
					* Constants.kRawMaxVel;
		
		double RTempV=(forward*Math.abs(forward)-turn*Math.abs(turn))
					* Constants.kRawMaxVel;

		if(smoothMode){
			//motion profile applied
			if(Math.abs(LTempV)<Math.abs(LMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx))) {
				//Deacceleration
				LMotorMaster.set(ControlMode.Velocity,
		            	util.setWithin(
		                	LTempV,   //Temp velocity
		                	LMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx),    //Target velocity
		                	Constants.kDeaccThreshold));   //limit
					
		        RMotorMaster.set(ControlMode.Velocity,
		            	util.setWithin(
		                	RTempV,   //Temp velocity
		                	RMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx),    //Targer velocity
		                	Constants.kDeaccThreshold));   //limit
			}else {
				//Acceleration
				LMotorMaster.set(ControlMode.Velocity,
		            	util.setWithin(
		                	LTempV,   //Temp velocity
		                	LMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx),    //Target velocity
		                	Constants.kAccThreshold));   //limit
					
		        RMotorMaster.set(ControlMode.Velocity,
		            	util.setWithin(
		                	RTempV,   //Temp velocity
		                	RMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx),    //Targer velocity
		                	Constants.kAccThreshold));   //limit
			}
			
		}else{
			//no motion profile applied
			LMotorMaster.set(ControlMode.Velocity,LTempV); 
        	RMotorMaster.set(ControlMode.Velocity,RTempV); 
		}		

	}

	public void PWMDrive(double forward, double turn) {

		/****** PWM mode : clockwise is positive */
		
		/*
		LMotorMaster.set(ControlMode.PercentOutput,
				forward*Math.abs(forward)+turn*Math.abs(turn));

		RMotorMaster.set(ControlMode.PercentOutput,
				forward*Math.abs(forward)-turn*Math.abs(turn));	
		*/
		configPID(LMotorMaster,0.3,0.7,0.0001,0.3);
		configPID(RMotorMaster,0.3,0.7,0.0001,0.3);
		
		LMotorMaster.set(ControlMode.PercentOutput,forward-turn, 0);
		RMotorMaster.set(ControlMode.PercentOutput,forward+turn, 0);
	}

	public boolean Rotate(double angle,double vel, double forVel) {

		/***
		 * angle is in degree
		 */
		
//		turnController.setSetpoint(angle);
//		SmartDashboard.putNumber("Info_PIDOutput", turnController.get());
//		velDrive(forVel,vel*turnController.get(),false);
//		return turnController.onTarget();
		
	
		double angleLeft=angle-(ahrs.getAngle()%360.0);
		SmartDashboard.putNumber("Info_MaxRV", ahrs.getRate());
		if(util.isWithin(angleLeft,0,2.5)) {
			velDrive(forVel,0,false);
			return true;
		}else if(
				util.isWithin(
					angleLeft,
					0,
					Constants.kSlowDownAngle*(ahrs.getRate()/Constants.kMaxRate)
					//Different starting angles
					)
				)
		{
			velDrive(forVel,util.equalsign(-angleLeft, Constants.kMinVel*2),false);
			return false;
		}else{
			velDrive(forVel,-angleLeft*vel,true);
			return false;
		}

	}

	public boolean DisDrive(double dis, double speed, boolean preciseMode) {
		/**** distance closed-loop mode */

		/* One rev=4096units
		 * Times 4096
		 */

		double temppos;
		boolean result=false;
		temppos=(dis)* 4096;
		
		double LTempPos=temppos+LDis;
		double RTempPos=temppos+RDis;
		
		
		//Control
		if(!preciseMode) {
			
			//Approximate velocity
			velDrive(speed,0,false);
			//Enable soft limit
			if(temppos>0) {
				result=LMotorMaster.getSelectedSensorPosition(0)>=LTempPos;
			}else {
				result=LMotorMaster.getSelectedSensorPosition(0)<=LTempPos;
			}
			
		}else {
			
			//Distance Closed Loop
			
			configPID(LMotorMaster,0,0.03,0,0.01);
			configPID(RMotorMaster,0,0.03,0,0.01);
			
			
			LMotorMaster.set(ControlMode.Position,LTempPos);
			RMotorMaster.set(ControlMode.Position,RTempPos);
			
			result=Math.abs(LMotorMaster.getClosedLoopError(0))<=disErr;
		}
			
		return result;
		

		/*
		configPID(LMotorSlave,0,0.05,0,0.02);
		configPID(RMotorSlave,0,0.05,0,0.02);

		LMotorSlave.set(ControlMode.Position,
				(dis+angle)* 4096);

		RMotorSlave.set(ControlMode.Position,
				(dis-angle)* 4096);

		DisplayInfo();
		 */
	}
	
	public void UpdateDistance() {
		LDis=LMotorMaster.getSelectedSensorPosition(0);
		RDis=RMotorMaster.getSelectedSensorPosition(0);
	}

	private void DisplayInfo() {

		/**** Displaying velocity on smart dashboard */

		SmartDashboard.putNumber("Info_Base_LVel",LMotorMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("Info_Base_LPWM",LMotorMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Info_Base_RPWM",RMotorMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Info_Base_LDis",(double)LMotorMaster.getSelectedSensorPosition(Constants.kPIDLoopIdx)/4096.0);
		SmartDashboard.putNumber("Info_Base_RDis",(double)RMotorMaster.getSelectedSensorPosition(Constants.kPIDLoopIdx)/4096.0);
		SmartDashboard.putNumber("Info_Base_Angle",(double)ahrs.getAngle());
		SmartDashboard.putNumber("Info_Base_AngleRate",(double)ahrs.getRate());

		SmartDashboard.putNumber("Info_Base_RPWM_Slave",RMotorSlave.getMotorOutputPercent());
		SmartDashboard.putNumber("Info_Base_LPWM_Slave",LMotorSlave.getMotorOutputPercent());
		
		SmartDashboard.putNumber("Info_X",(double)pos_X);
		SmartDashboard.putNumber("Info_Y",(double)pos_Y);
	}
	
	private void resetSensor() {

		LMotorMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		RMotorMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		ahrs.reset();
		ahrs.resetDisplacement();
	}

	private void TalonSRXInit(TalonSRX _talon) {
		//set up TalonSRX and closed loop
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				Constants.kPIDLoopIdx,Constants.kTimeoutMs);

		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

	}

	private void configPID(TalonSRX _talon,double kF, double kP,double kI, double kD) {
		_talon.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);

	}
	
	@Override
	public void pidWrite(double output) {
	}
	
	@Override
	public double pidGet() {
		return ahrs.getAngle();
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kRate;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {	}
	
}