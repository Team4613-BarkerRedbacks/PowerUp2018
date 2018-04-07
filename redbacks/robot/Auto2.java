package redbacks.robot;

import static redbacks.arachne.core.ArachneRobot.sequencer;
import static redbacks.arachne.ext.motion.MotionSettings.encoderTicksPerMetre;
import static redbacks.robot.CommandList.*;
import static redbacks.robot.OldPathList.*;
import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DriverStation;
import redbacks.arachne.lib.actions.AcDoNothing;
import redbacks.arachne.lib.actions.AcInterrupt;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.Auto.AutoComponent;
import redbacks.robot.actions.AcDriveDirection;
import redbacks.robot.actions.AcResetSensors;
import redbacks.robot.actions.AcSetArm;
import redbacks.robot.actions.AcSplitIntakeControl;
import redbacks.robot.actions.AcStraight;
import redbacks.robot.actions.AcStraightFinishOnTarget;
import redbacks.robot.actions.AcStraightLenient;
import redbacks.robot.actions.AcTankDrive;
import redbacks.robot.actions.AcTankTurn;
import redbacks.robot.actions.AcTurn;

import static redbacks.robot.Auto.*;

public class Auto2
{
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
		
		
		case C_L__LLL_SS:
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcTankTurn(-60),
					new AcSetArm(armScalePos + 100),
					new AcStraight(3.1, -60, sensors.distanceEncoder, true),
					new AcTankTurn(-5),
					new AcDriveDirection(new ChNumSen(0.5*encoderTicksPerMetre, sensors.distanceEncoder), 0.7, 0),
					new AcSeq.Parallel(outtakeCubeSlow),
					new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
					new AcInterrupt.KillSubsystem(intake),
					//2rd cube 
					new AcDriveDirection(new ChNumSen(-0.4 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 0),
					new AcSetArm(armBasePos),
					new AcTankTurn(80),
					new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeFastSpeed)),
					new AcDriveDirection(new ChNumSen(0.9 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true), 0.55, 80),
					new AcWait(0.25),
					new AcSeq.Parallel(intakeCubeFast),
					new AcDriveDirection(new ChNumSen(0.2 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, false), -0.6, 80),
					new AcInterrupt.KillSubsystem(intake),
					new AcSetArm(armScalePos + 100),
					new AcTankTurn(10),
					new AcDriveDirection(new ChTime(1), 0.6, 10),
					new AcSeq.Parallel(outtakeCubeSlow),
					new AcWait(0.25),
					new AcInterrupt.KillSubsystem(intake),
					//3nd cube (pick up cube)
					new AcDriveDirection(new ChNumSen(-0.4 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 0),
					new AcSetArm(armBasePos),
					new AcTankTurn(65),
					new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeFastSpeed)),
					new AcDriveDirection(new ChNumSen(0.85 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true), 0.6, 65),
					new AcWait(0.5),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcSetArm(0),
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcDriveDirection(new ChNumSen(-0.5 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 65)
					);
		case L__L_HHH_12:
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(6.3 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
//							//TODO Check the height on this shot
//							new AcSeq.Parallel(newFireCommand(Robot.intake, new AcSplitIntakeControl(new ChTime(0.5), 0.8, 1)))
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(6.9, 8.25, sensors.distanceEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTankTurn(-11),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
							new AcSeq.Parallel(intakeCube)
					),
					new AcStraightLenient(-1.8, -11, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.4), -0.5, -0.5),
					new AcSetArm(armSlightPos),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(highFirePrime),
					new AcTankTurn(0),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1.25 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(1.5, 0, sensors.averageEncoder, true),
					//3rd cube
					new AcWait(0.25),
					new AcSetArm(-armBasePos),
					new AcTankTurn(-25),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-2, -25, sensors.averageEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcSetArm(-armScalePos),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraight(2.5, -35, sensors.distanceEncoder, true),
					new AcInterrupt.KillSubsystem(intake),
					new AcTankTurn(-90),
					new AcWait(0.25),
					new AcSeq.Parallel(highFireRelease)
					);
			default: return null;
		}
	}
}
