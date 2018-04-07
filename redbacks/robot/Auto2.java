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
		
		case R_RR_LHH_65:
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1.5 * encoderTicksPerMetre, sensors.distanceEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcDoNothing(new ChNumSen(5 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSlowSpeed))
					),
					new AcStraight(4, -11, sensors.distanceEncoder, true),
					//2nd cube
					new AcStraight(5.8, 5, sensors.distanceEncoder, false),
					new AcTurn(40),
					new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
					new AcStraightLenient(-1.2, 40, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
					new AcWait(0.25),
					new AcSeq.Parallel(
							new AcSetArm(armSlightPosTele),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcTurn(-5),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(1.25, -5, sensors.distanceEncoder, true),
					//3rd cube
					new AcWait(0.25),
					new AcSetArm(-armBasePos),
					new AcDriveDirection(new ChNumSen(-0.6 * encoderTicksPerMetre, sensors.averageEncoder), -1, -5),
					new AcTankTurn(35),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-0.8, 35, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.averageEncoder, false, false, true)),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcSetArm(armScalePos),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcDriveDirection(new ChNumSen(0.8 * encoderTicksPerMetre, sensors.distanceEncoder), 1, 35),
					new AcTankTurn(-5),
					new AcInterrupt.KillSubsystem(intake),
					new AcDriveDirection(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.distanceEncoder), 1, -5),
					new AcWait(0.25),
					new AcSeq.Parallel(highFireRelease)	
				);
			default: return null;
		}
	}
}
