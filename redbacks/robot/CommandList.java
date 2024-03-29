package redbacks.robot;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.*;

import static redbacks.robot.RobotMap.*;
import static redbacks.robot.Robot.*;

/**
 * Provides a list of standard commands for global use.
 *
 * @author Ben Schwarz, Darin Huang, Lucas Parker, Matthew Brian, Mitchell Barker, Sean Zammit
 */
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
		armToSlightBack = newCom(new AcSetArm(-armSlightPosTele)),
		armToSlightFront = newCom(new AcSetArm(armSlightPosTele)),
		armToTop = newCom(new AcSetArm(0)),
		
		solExtendIntake = newCom(
				new AcSolenoid.Single(intake.intakeRightSol, true),
				new AcSolenoid.Single(intake.intakeLeftSol, true)
		),
		solRetractIntake = newCom(
				new AcSolenoid.Single(intake.intakeLeftSol, false), 
				new AcSolenoid.Single(intake.intakeRightSol, false)
		),
		
		solExtendIntakeR = newCom(new AcSolenoid.Single(intake.intakeRightSol, true)),
		solRetractIntakeR = newCom(
				new AcSolenoid.Single(intake.intakeRightSol, false),
				new AcInterrupt.KillSubsystem(intake)
		),
		
		solExtendIntakeL = newCom(new AcSolenoid.Single(intake.intakeLeftSol, true)),
		solRetractIntakeL = newCom(
				new AcSolenoid.Single(intake.intakeLeftSol, false),
				new AcInterrupt.KillSubsystem(intake)
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
		superFirePrime = newCom(
				new AcSolenoid.Single(shooter.shooterLockSol, true),
				new AcWait(0.5),
				new AcSolenoid.Single(shooter.shooterSol1, true),
				new AcSolenoid.Single(shooter.shooterSol2, true),
				new AcSolenoid.Single(shooter.shooterSolHigh, true)
		),
		
		climberRelease = newCom(
				new AcSetArm(-armBasePos),
				new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
				new AcSolenoid.Single(climber.climberSol, true)
		),
		
		stopIntake = newCom(new AcInterrupt.KillSubsystem(intake)),
		stopAll = newCom(new AcInterrupt.KillAllCommands());
	
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
		climbUp = newCom(new AcMotor.Set(climber.climberMotor, 0.25, new ChFalse())),
		climbDown = newCom(new AcMotor.Set(climber.climberMotor, -0.25, new ChFalse())),
		climbManual = newCom(new AcClimber());
	
	static {subsystemToUse = intake;}
	public static CommandSetup
		intakeCube = newCom(
				new AcSolenoid.Single(shooter.shooterSol1, false),
				new AcSolenoid.Single(shooter.shooterSol2, false),
				new AcMotor.Set(intake.intakeMotor, intakeSpeed, new ChFalse())
		),
		intakeCubeFast = newCom(
				new AcSolenoid.Single(shooter.shooterSol1, false),
				new AcSolenoid.Single(shooter.shooterSol2, false),
				new AcMotor.Set(intake.intakeMotor, intakeFastSpeed, new ChFalse())
		),
		intakeCubeSlow = newCom(
				new AcSolenoid.Single(shooter.shooterSol1, false),
				new AcSolenoid.Single(shooter.shooterSol2, false),
				new AcMotor.Set(intake.intakeMotor, intakeSlowSpeed, new ChFalse())
		),
		
		outtakeCube = newCom(new AcMotor.Set(intake.intakeMotor, -0.5, new ChFalse())),
		outtakeCubeSlow = newCom(new AcMotor.Set(intake.intakeMotor, -0.35, new ChFalse())),
		outtakeCubeFast = newCom(new AcMotor.Set(intake.intakeMotor, -1, new ChFalse())),
					
		intakeCubeSpin = newCom(new AcSplitIntakeControl(new ChFalse(), RobotMap.intakeSpeed, intakeSlowSpeed)),
		outtakeCubeSpinRight = newCom(new AcSplitIntakeControl(new ChFalse(), -RobotMap.intakeSlowSpeed, -RobotMap.intakeFastSpeed)),
		
		intakeCubeAnalog = newCom(new AcIntakeAnalog() {
			public double getSpeed() {
				double trigger = OI.axis_o_LT.get();
				return 0.3 + (trigger - 0.2) / 2;
			}
		}),
		outtakeCubeAnalog = newCom(new AcIntakeAnalog() {
			public double getSpeed() {
				double trigger = OI.axis_o_RT.get();
				return -0.2 - (trigger - 0.2) / 2;
			}
		}),
		
		sideKickR = newCom(
				new AcSolenoid.Single(intake.intakeRightSol, true),
				new AcSplitIntakeControl(new ChFalse(), -0.6, 0)
		),
		sideKickL = newCom(
				new AcSolenoid.Single(intake.intakeLeftSol, true),
				new AcSplitIntakeControl(new ChFalse(), 0, -0.6)
		),
		
		highFireRelease = newFireCommand(subsystemToUse, new AcMotor.Set(intake.intakeMotor, -1, new ChTime(0.5))),
		superFireRelease = newCom(
				new AcSolenoid.Single(shooter.shooterLockSol, false),
				new AcSeq.Parallel(
						new AcMotor.Set(intake.intakeMotor, -1, new ChTime(0.5))
				),
				new AcWait(0.5),
				new AcSolenoid.Single(shooter.shooterSol1, false),
				new AcSolenoid.Single(shooter.shooterSol2, false),
				new AcSolenoid.Single(shooter.shooterSolHigh, false)
		);
	
	static{subsystemToUse = shooter;}
	public static CommandSetup
		quickFire = newCom(
				new AcSeq.Parallel(highFirePrime),
				new AcWait(2.5),
				new AcSeq.Parallel(highFireRelease)
		);
	
	static {subsystemToUse = driver;}
	public static CommandSetup
		drive = newCom(new AcDrive()),
		tuneLinearPID = newCom(new AcTuneLinearPID(sensors.distanceEncoder)),
		cubeFollow = newCom(
				new AcSeq.Parallel(intakeCube),
				new AcMovetoCube(0.55)
		);
	
	public static CommandSetup newFireCommand(SubsystemBase requiredSystem, Action intakeAction) {
		return newCom(requiredSystem,
				new AcSolenoid.Single(shooter.shooterLockSol, false),
				new AcSeq.Parallel(intakeAction),
				new AcSolenoid.Single(shooter.shooterSol1, true),
				new AcSolenoid.Single(shooter.shooterSol2, true),
				new AcWait(0.25),
				new AcSolenoid.Single(shooter.shooterSol1, false),
				new AcSolenoid.Single(shooter.shooterSol2, false)
		);
	}
}