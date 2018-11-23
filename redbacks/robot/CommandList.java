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

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static redbacks.robot.Robot.*;

public class CommandList extends CommandListStart
{
	static {subsystemToUse = null;}
	public static CommandSetup
		armToLowFront = newCom(new AcSetArm(armSwitchPos)),
		armToLowBack = newCom(new AcSetArm(-armSwitchPos)),
		armToBaseBack = newCom(new AcSetArm(-armBasePos)),
		armToBaseFront = newCom(new AcSetArm(armBasePos)),
		armToScaleBack = newCom(new AcSetArm(-armScalePos)),
		armToScaleFront = newCom(new AcSetArm(armScalePos)),
		armToSlightBack = newCom(new AcSetArm(-armSlightPosTele)),
		armToSlightFront = newCom(new AcSetArm(armSlightPosTele)),
		armToTop = newCom(new AcSetArm(0)),
		solRetractIntakeR = newCom(
			new AcSolenoid.Single(intake.intakeRightSol, false),
			new AcInterrupt.KillSubsystem(intake)
		),
		solRetractIntakeL = newCom(
			new AcSolenoid.Single(intake.intakeLeftSol, false),
			new AcInterrupt.KillSubsystem(intake)
		),
		highFirePrime = newCom(
			new AcSolenoid.Single(shooter.shooterLockSol, true),
			new AcWait(0.5),
			new AcSolenoid.Single(shooter.shooterSol, true)
		),
		climberRelease = newCom(
			new AcSetArm(-armBasePos),
			new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
			new AcSolenoid.Double(climber.climberSol, Value.kForward)
		),
		stopIntake = newCom(new AcInterrupt.KillSubsystem(intake)),
		stopAll = newCom(new AcInterrupt.KillAllCommands());
	
	static {subsystemToUse = sensors;}
	public static CommandSetup
		readSensors = newCom(new AcReadSensors());
	
	static {subsystemToUse = arm;}
	public static CommandSetup
			armLoop = newCom(new AcArm());

	static {subsystemToUse = climber;}
	public static CommandSetup
		climbManual = newCom(new AcClimber());
	
	static {subsystemToUse = intake;}
	public static CommandSetup
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
		highFireRelease = newFireCommand(subsystemToUse,
			new AcMotor.Set(intake.intakeMotor, -1, new ChTime(0.5))
		);
	
	static {subsystemToUse = driver;}
	public static CommandSetup
		drive = newCom(new AcDrive());
	
	public static CommandSetup newFireCommand(SubsystemBase requiredSystem, Action intakeAction) {
		return newCom(requiredSystem,
				new AcSolenoid.Single(shooter.shooterLockSol, false),
				new AcSeq.Parallel(intakeAction),
				new AcSolenoid.Single(shooter.shooterSol, true),
				new AcWait(0.25),
				new AcSolenoid.Single(shooter.shooterSol, false)
		);
	}
}