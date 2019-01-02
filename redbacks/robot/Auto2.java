package redbacks.robot;

import static redbacks.arachne.core.ArachneRobot.sequencer;
import static redbacks.arachne.ext.motion.MotionSettings.encoderTicksPerMetre;
import static redbacks.robot.CommandList.*;
import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import redbacks.arachne.lib.actions.AcDoNothing;
import redbacks.arachne.lib.actions.AcInterrupt;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.robot.Auto.AutoComponent;
import redbacks.robot.actions.AcDriveDirection;
import redbacks.robot.actions.AcResetSensors;
import redbacks.robot.actions.AcSetArm;
import redbacks.robot.actions.AcSplitIntakeControl;
import redbacks.robot.actions.AcStraight;
import redbacks.robot.actions.AcStraightLenient;
import redbacks.robot.actions.AcTankDrive;
import redbacks.robot.actions.AcTankTurn;
import redbacks.robot.actions.AcTurn;

import static redbacks.robot.Auto.*;

public class Auto2
{
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
			case L_R__LLL_43:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraight(5.45, 0, sensors.distanceEncoder, true),
						new AcTurn(90),
						new AcStraight(2.45, 90, sensors.distanceEncoder, true),
						new AcTurn(135),
						new AcSetArm(armSwitchPos),
						new AcSeq.Parallel(sequencer,
								new AcStraight(1.3, 135, sensors.distanceEncoder, true)
						),
						new AcDoNothing(new ChNumSen(1.5, new SenTimer())),
						new AcInterrupt.KillSubsystem(sequencer),
						new AcSeq.Parallel(outtakeCubeFast),
						//2nd cube
						new AcWait(0.25),
						new AcInterrupt.KillSubsystem(intake),
						new AcDriveDirection(new ChTime(0.75), -0.6, 135),
						new AcSetArm(armBasePos),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeFastSpeed)),
						new AcDriveDirection(new ChTime(1.25), 0.6, 145),
						new AcWait(0.5),
						new AcSetArm(armSwitchPos - 100),
						new AcDoNothing(new ChNumSen(armSwitchPos, sensors.armEncoder, false, false, false)),
						new AcInterrupt.KillSubsystem(intake),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//3rd cube
						new AcResetSensors(),
						new AcWait(0.25),
						new AcInterrupt.KillSubsystem(intake),
						new AcDriveDirection(new ChNumSen(-0.4 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.55, -10),
						new AcSetArm(armBasePos),
						new AcTankTurn(35),
						new AcSeq.Parallel(intakeCube),
						new AcDriveDirection(new ChTime(1), 0.6, 35),
						new AcSetArm(armSwitchPos),
						new AcWait(0.5),
						new AcInterrupt.KillSubsystem(intake),
						new AcTankTurn(0),
						new AcDriveDirection(new ChTime(0.5), 0.5, 0),
						new AcSeq.Parallel(highFireRelease)
				);
			case L__R_HH_5:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraight(5.3, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -1),
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), 1)),
						new AcTankTurn(90),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(3 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraight(5.4, 90, sensors.averageEncoder, true,
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -1),
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), 1)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSetArm(armSlightPos),
						new AcTankTurn(-5),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.35 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcDriveDirection(new ChNumSen(0.6 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.6, -5),
						//2nd cube
						new AcWait(0.25),
						new AcSetArm(-armBasePos),
						new AcTankTurn(8),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSpeed)),
						new AcStraightLenient(-2, 8, sensors.averageEncoder, true),
						new AcSeq.Parallel(intakeCube),
						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
						new AcTankTurn(45),
						new AcSeq.Parallel(highFirePrime),
						new AcSetArm(armSlightPos),
						new AcDriveDirection(new ChNumSen(0.3 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.5, 45),
						new AcTankTurn(-5),
						new AcInterrupt.KillSubsystem(intake),
						new AcWait(0.25),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.45 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcDriveDirection(new ChNumSen(0.7 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.6, -10)
//						new AcSetArm(armSlightPos),
//						new AcSeq.Parallel(highFirePrime),
//						new AcWait(0.25),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
//								new AcInterrupt.KillSubsystem(intake),
//								new AcDoNothing(new ChNumSen(0.9 * encoderTicksPerMetre, sensors.averageEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcDriveDirection(new ChNumSen(1.4 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.6, 0)
				);
			default: return null;
		}
	}
}
