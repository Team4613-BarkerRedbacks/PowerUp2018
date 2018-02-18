package redbacks.robot;

import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.*;

import static redbacks.robot.RobotMap.*;
import static redbacks.robot.Robot.*;

public class CommandList extends CommandListStart
{
	static {subsystemToUse = null;}
	public static CommandSetup
		resetArm = newCom(new AcSetNumSen(sensors.armEncoder, 0)),
		encoderPos = newCom(new AcSolenoid.Single(driver.centreEncoderSol, false)),
		armToLowFront = newCom(new AcSetArm(armSwitchPos)),
		armToLowBack = newCom(new AcSetArm(-armSwitchPos)),
		armToBaseBack = newCom(new AcSetArm(-armBasePos)),
		armToBaseFront = newCom(new AcSetArm(armBasePos)),
		armToScaleBack = newCom(new AcSetArm(-armScalePos)),
		armToScaleFront = newCom(new AcSetArm(armScalePos)),
		armToTop = newCom(new AcSetArm(0)),
		solExtendIntake = newCom(
			new AcSolenoid.Single(intake.intakeRightSol, true),
			new AcSolenoid.Single(intake.intakeLeftSol, true)
		),
		solRetractIntake = newCom(
			new AcSolenoid.Single(intake.intakeLeftSol, false), 
			new AcSolenoid.Single(intake.intakeRightSol, false)
		),
		solExtendIntakeR = newCom(
			new AcSolenoid.Single(intake.intakeRightSol, true)
		),
		solRetractIntakeR = newCom(
			new AcSolenoid.Single(intake.intakeRightSol, false)
		),
		solExtendIntakeL = newCom(
			new AcSolenoid.Single(intake.intakeLeftSol, true)
		),
		solRetractIntakeL = newCom(
			new AcSolenoid.Single(intake.intakeLeftSol, false)
		),
		lowFire = newCom(
			new AcSolenoid.Single(shooter.shooterLockSol, false),
			new AcSolenoid.Single(shooter.shooterSol1, true),
			new AcSolenoid.Single(shooter.shooterSol2, true),
			new AcWait(0.5), 
			new AcSolenoid.Single(shooter.shooterSol1, false),
			new AcSolenoid.Single(shooter.shooterSol2, false)
		),
		highFirePrime = newCom(
			new AcSolenoid.Single(shooter.shooterLockSol, true),
			new AcWait(0.5),
			new AcSolenoid.Single(shooter.shooterSol1, true),
			new AcSolenoid.Single(shooter.shooterSol2, true)
		),
		climberRelease = newCom(
			new AcSetArm(-armBasePos),
			new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
			new AcSolenoid.Single(climber.climberSol, true)
		),
		spinLeft = newCom(
			new AcIntakeRightSide(new ChTime(3))
		);
	
	static {subsystemToUse = driver;}
	public static CommandSetup
		drive = newCom(new AcDrive()),
		limelightTrack = newCom(new AcLimelightTrack());
	
	static {subsystemToUse = sensors;}
	public static CommandSetup
		readSensors = newCom(new AcReadSensors()),
		resetSensors = newCom(new AcResetSensors());
	
	static {subsystemToUse = arm;}
	public static CommandSetup
			armLoop = newCom(new AcArm()),
			moveArm = newCom(new AcMotor.Set(arm.armMotor, armMaxSpeed, new ChFalse())),
			reverseArm = newCom(new AcMotor.Set(arm.armMotor, -armMaxSpeed, new ChFalse()));

	static {subsystemToUse = climber;}
	public static CommandSetup
		climb = newCom(
			new AcMotor.Set(climber.climberMotor, 0.25, new ChFalse())
		);
	
	static {subsystemToUse = intake;}
	public static CommandSetup
		intakeCube = newCom(
			new AcSolenoid.Single(shooter.shooterSol1, false),
			new AcSolenoid.Single(shooter.shooterSol2, false),
			new AcMotor.Set(intake.intakeMotor, intakeFastSpeed, new ChFalse())
		),
		intakeCubeSlow = newCom(
			new AcSolenoid.Single(shooter.shooterSol1, false),
			new AcSolenoid.Single(shooter.shooterSol2, false),
			new AcMotor.Set(intake.intakeMotor, intakeSlowSpeed, new ChFalse())
		),
		outtakeCube = newCom(
			new AcMotor.Set(intake.intakeMotor, -0.5, new ChFalse())
		),
		outtakeCubeFast = newCom(
			new AcMotor.Set(intake.intakeMotor, -1, new ChFalse())
		),
		highFireRelease = newCom(
			new AcSolenoid.Single(shooter.shooterLockSol, false),
			new AcSeq.Parallel(
					new AcMotor.Set(intake.intakeMotor, -1, new ChTime(0.5))
			),
			new AcWait(0.25),
			new AcSolenoid.Single(shooter.shooterSol1, false),
			new AcSolenoid.Single(shooter.shooterSol2, false)
		);
	
	static{subsystemToUse = shooter;}
	public static CommandSetup
		quickFire = newCom(
			new AcSeq.Parallel(highFirePrime),
			new AcWait(2.5),
			new AcSeq.Parallel(highFireRelease)
		);
}