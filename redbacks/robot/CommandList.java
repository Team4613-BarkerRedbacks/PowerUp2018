package redbacks.robot;

import redbacks.arachne.core.references.CommandListStart;
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
		moveArm 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChFalse())),
		reverseArm 	= newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChFalse())),
		setArmFlatR = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(950, Robot.sensors.armEncoder, true, false, false))),
		setArmFlatL = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-950, Robot.sensors.armEncoder, true, false, false))),
		setArm550R 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(550, Robot.sensors.armEncoder, true, false, false))),
		setArm550L 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-550, Robot.sensors.armEncoder, true, false, false))),
		setArm300R 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(300, Robot.sensors.armEncoder, true, false, false))),
		setArm300L 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-300, Robot.sensors.armEncoder, true, false, false))),
		centreArm 	= newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(0, Robot.sensors.armEncoder, true, false, false)));
	
	static {subsystemToUse = Robot.climber;}
	public static CommandSetup
		climb = newCom(new AcMotor.Set(Robot.climber.climberMotor, 0.25, new ChFalse()));
	
	static {subsystemToUse = Robot.intake;}
	public static CommandSetup
		intake = newCom(
			new AcMotor.Set(Robot.intake.intakeRightMotor, 0.25, new ChFalse()), 
			new AcMotor.Set(Robot.intake.intakeLeftMotor, 0.25, new ChFalse())
			),
		outtake = newCom(
			new AcMotor.Set(Robot.intake.intakeRightMotor, -0.25, new ChFalse()), 
			new AcMotor.Set(Robot.intake.intakeRightMotor, -0.25, new ChFalse()));
	
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
}