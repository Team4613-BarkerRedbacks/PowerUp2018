package redbacks.robot;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.ext.motion.pid.AcPIDControl;
import redbacks.arachne.ext.motion.pid.PIDMotor;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.*;

public class CommandList extends CommandListStart {
	
	static {subsystemToUse = Robot.driver;}
	public static CommandSetup
		drive = newCom(new AcDrive());
	
	static {subsystemToUse = Robot.sensors;}
	public static CommandSetup
		readSensors = newCom(new AcReadSensors());
	
	static {subsystemToUse = Robot.arm;}
	public static CommandSetup
		moveArm 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChFalse())),
		reverseArm 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChFalse())),
//		setArmFlatR  = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(950, Robot.sensors.armEncoder, true, false, false))),
//		setArmFlatL  = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-950, Robot.sensors.armEncoder, false, false, false))),
//		setArm550R 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(550, Robot.sensors.armEncoder, true, false, false))),
//		setArm550L 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-550, Robot.sensors.armEncoder, false, false, false))),
//		setArm300R 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(300, Robot.sensors.armEncoder, true, false, false))),
//		setArm300L 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChNumSen(-300, Robot.sensors.armEncoder, false, false, false))),
//		centreArm 	 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(0, Robot.sensors.armEncoder, true, false, false))),
		holdArmCentre = newCom(new AcPIDControl(new ChFalse(), false, 0.0025, 0.00000015, 0.00005, 0.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -.3, .3, new PIDMotor(Robot.arm.aMotor))),
		holdArmFlatR = newCom(new AcPIDControl(new ChFalse(), false, 0.0025, 0.00000015, 0.00005, -200.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -.3, .3, new PIDMotor(Robot.arm.aMotor))),
		holdArmFlatL = newCom(new AcPIDControl(new ChFalse(), false, 0.0025, 0.00000015, 0.00005, 200.0, new Tolerances.Absolute(3), Robot.sensors.armEncoder, false, 0, 0, PIDSourceType.kDisplacement, -.3, .3, new PIDMotor(Robot.arm.aMotor)));

	
	static {subsystemToUse = Robot.climber;}
	public static CommandSetup
		climb = newCom(
			new AcMotor.Set(Robot.climber.climberMotor, 0.25, new ChFalse())
		);
	
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
		),
		highFireRelease = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, false),
			new AcWait(0.5),
			new AcSolenoid.Single(Robot.shooter.shooterSol, false)
		);
	
	static{subsystemToUse = Robot.pneumatics;}
	public static CommandSetup
		encoderPos = newCom(
			new AcSolenoid.Single(Robot.pneumatics.centreEncoderSol, false)
		);
}