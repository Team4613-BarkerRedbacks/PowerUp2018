package redbacks.robot;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.ext.motion.pid.AcPIDControl;
import redbacks.arachne.ext.motion.pid.PIDMotor;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.*;

public class CommandList extends CommandListStart
{
	static {subsystemToUse = null;}
	public static CommandSetup
		resetArm = newCom(new AcSetNumSen(Robot.sensors.armEncoder, 0)),
		encoderPos = newCom(new AcSolenoid.Single(Robot.driver.centreEncoderSol, false));
	
	static {subsystemToUse = Robot.driver;}
	public static CommandSetup
		drive = newCom(new AcDrive()),
		limeLight = newCom(new AcLimeLight());
	
	static {subsystemToUse = Robot.sensors;}
	public static CommandSetup
		readSensors = newCom(new AcReadSensors()),
		resetSensors = newCom(new AcResetSensors());
	
	static {subsystemToUse = Robot.monitor;}
	public static CommandSetup
		monitorRobot = newCom(new AcMonitor());
		
	static {subsystemToUse = Robot.arm;}
	public static CommandSetup
		armLoop 	 = newCom(new AcArm()),
		moveArm 	 = newCom(new AcMotor.Set(Robot.arm.armMotor, RobotMap.armSpeed, new ChFalse())),
		reverseArm 	 = newCom(new AcMotor.Set(Robot.arm.armMotor, -RobotMap.armSpeed, new ChFalse())),
//		setArmFlatR  = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(950, Robot.sensors.armEncoder, true, false, false))),
//		setArmFlatL  = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-950, Robot.sensors.armEncoder, false, false, false))),
//		setArm550R 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(550, Robot.sensors.armEncoder, true, false, false))),
//		setArm550L 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-550, Robot.sensors.armEncoder, false, false, false))),
//		setArm300R 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(300, Robot.sensors.armEncoder, true, false, false))),
//		setArm300L 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-300, Robot.sensors.armEncoder, false, false, false))),
//		centreArm 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(0, Robot.sensors.armEncoder, true, false, false))),
		holdArmCentre = newCom(new AcPIDControl(new ChFalse(), false, 0.005, 0.0000003, 0.0001, 0.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -RobotMap.armSpeed, RobotMap.armSpeed, new PIDMotor(Robot.arm.armMotor))),
		holdArmFlatR = newCom(new AcPIDControl(new ChFalse(), false, 0.005, 0.0000003, 0.0001, -200.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -RobotMap.armSpeed, RobotMap.armSpeed, new PIDMotor(Robot.arm.armMotor))),
		holdArmFlatL = newCom(new AcPIDControl(new ChFalse(), false, 0.005, 0.0000003, 0.0001, 200.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -RobotMap.armSpeed, RobotMap.armSpeed, new PIDMotor(Robot.arm.armMotor)));

//	static {subsystemToUse = Robot.climber;}
//	public static CommandSetup
//		climb = newCom(
//			new AcMotor.Set(Robot.climber.climberMotor, 0.25, new ChFalse())
//		);
	
	static {subsystemToUse = Robot.intake;}
	public static CommandSetup
		intake = newCom(
			new AcMotor.Set(Robot.intake.intakeMotor, 0.5, new ChFalse())
		),
		outtake = newCom(
			new AcMotor.Set(Robot.intake.intakeMotor, -0.5, new ChFalse())
		),
		solExtendIntake = newCom(
			new AcSolenoid.Single(Robot.intake.intakeRightSol, true),
			new AcSolenoid.Single(Robot.intake.intakeLeftSol, true)
		),
		solRetractIntake = newCom(
			new AcSolenoid.Single(Robot.intake.intakeLeftSol, false), 
			new AcSolenoid.Single(Robot.intake.intakeRightSol, false)
		),
		solExtendIntakeR = newCom(
			new AcSolenoid.Single(Robot.intake.intakeRightSol, true)
		),
		solRetractIntakeR = newCom(
			new AcSolenoid.Single(Robot.intake.intakeRightSol, false)
		),
		solExtendIntakeL = newCom(
			new AcSolenoid.Single(Robot.intake.intakeLeftSol, true)
		),
		solRetractIntakeL = newCom(
			new AcSolenoid.Single(Robot.intake.intakeLeftSol, false)
		),
		highFireRelease = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, false),
			new AcWait(0.5),
			new AcSolenoid.Single(Robot.shooter.shooterSol, false),
			new AcMotor.Set(Robot.intake.intakeMotor, -0.8, new ChTime(1))
		);
	
	static{subsystemToUse = Robot.shooter;}
	public static CommandSetup
		lowFire = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, false), 
			new AcSolenoid.Single(Robot.shooter.shooterSol, true), 
			new AcWait(0.5), 
			new AcSolenoid.Single(Robot.shooter.shooterSol, false)
		),
		highFirePrime = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, true),
			new AcWait(0.5),
			new AcSolenoid.Single(Robot.shooter.shooterSol, true)
		);
}