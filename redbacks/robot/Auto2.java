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
import redbacks.arachne.lib.actions.AcSetNumSen;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.sensors.SenTimer;
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
		case L_L__LLLL_123:
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(1.5 * encoderTicksPerMetre, sensors.distanceEncoder)),
							new AcSeq.Parallel(solExtendIntakeR),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeR),
							new AcDoNothing(new ChNumSen(5 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeFastSpeed))
					),
					new AcStraight(4, 11, sensors.distanceEncoder, true),
					//2nd cube
					new AcStraight(5.8, -5, sensors.distanceEncoder, false),
					new AcTurn(-40),
					new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
					new AcStraightLenient(-1.2, -40, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
					new AcWait(0.25),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
					new AcDoNothing(new ChNumSen(-armSwitchPos - 100, sensors.armEncoder, true, false, false)),
					new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), -intakeSlowSpeed, -intakeFastSpeed)),
					//3rd cube
					new AcWait(0.25),
					new AcTankTurn(-75),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeSpeed - 0.1)),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcDriveDirection(new ChNumSen(-0.8 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.55, -80),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(armSlightPosTele),
					new AcDoNothing(new ChNumSen(0, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(solExtendIntakeL),
					new AcSeq.Parallel(new AcTankDrive(new ChTime(0.5), 0.5, 0.5)),
					new AcWait(0.7),
					new AcSeq.Parallel(solRetractIntakeL),
					//4th cube
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeSpeed - 0.1)),
					new AcSeq.Parallel(sequencer, new AcDriveDirection(new ChNumSen(-1.6 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.55, -80)),
					new AcDoNothing(new ChTime(2)),
					new AcInterrupt.KillSubsystem(sequencer),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(armScalePos),
					new AcSeq.Parallel(new AcTankDrive(new ChTime(0.5), 0.5, 0.5)),
					new AcDoNothing(new ChNumSen(0, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(solExtendIntakeL),
					new AcWait(0.7),
					new AcSeq.Parallel(solRetractIntakeL)
			);
		case L__R_HH_5: //start left, right scale, 2 in scale, starting cube + cube 2
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraight(5.55, 0, sensors.distanceEncoder, true),
					new AcTurn(90),
					new AcStraight(5.2, 90, sensors.distanceEncoder, true),
					new AcSeq.Parallel(sequencer, 
							new AcDoNothing(new ChNumSen(2, new SenTimer())),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(6.5, new SenTimer())),
							new AcSeq.Parallel(highFireRelease)
									),
					new AcTurn(0),
					new AcSetArm(armScalePos),
					new AcStraight(1.6, 0, sensors.distanceEncoder, true)
					//2nd cube
					
					);
		case L_R__LLL_43: //start left, right switch, 3 in switch, starting cube + cube 3 + cube 4  
			return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraight(5.45, 0, sensors.distanceEncoder, true),
					new AcTurn(90),
					new AcStraight(2.45, 90, sensors.distanceEncoder, true),
					new AcTurn(135),
					new AcSetArm(armSwitchPos),
					new AcSeq.Parallel(sequencer,
							new AcStraight(1.2, 135, sensors.distanceEncoder, true)
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
					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(armSwitchPos - 100),
					new AcDoNothing(new ChNumSen(armSwitchPos, sensors.armEncoder, false, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcWait(0.25),
					new AcSeq.Parallel(
							CommandList.newFireCommand(null, new AcDoNothing(new ChTrue()))
					),
					//3rd cube
					new AcResetSensors(),
					new AcWait(0.25),
					new AcInterrupt.KillSubsystem(intake),
					new AcDriveDirection(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, -10),
					new AcSetArm(armBasePos),
					new AcTankTurn(35),
					new AcSeq.Parallel(intakeCube),
					new AcDriveDirection(new ChTime(1), 0.6, 35),
					new AcSetArm(armSwitchPos),
					new AcSeq.Parallel(highFirePrime),
					new AcWait(0.5),
					new AcTankTurn(0),
					new AcDriveDirection(new ChTime(0.5), 0.5, 0),
					new AcSeq.Parallel(highFireRelease)
					);
			default: return null;
		}
	}
}
