package redbacks.robot;

import static redbacks.arachne.core.ArachneRobot.sequencer;
import static redbacks.arachne.ext.motion.MotionSettings.encoderTicksPerMetre;
import static redbacks.robot.CommandList.*;
import static redbacks.robot.OldPathList.*;
import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.AcDoNothing;
import redbacks.arachne.lib.actions.AcInterrupt;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.AcSetNumSen;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.actions.SwitchAction;
import redbacks.arachne.lib.actions.SwitchAction.OptionAction;
import redbacks.arachne.lib.actions.actuators.AcSolenoid;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.Auto.AutoComponent;
import redbacks.robot.actions.AcResetSensors;
import redbacks.robot.actions.AcSetArm;
import redbacks.robot.actions.AcStraight;
import redbacks.robot.actions.AcStraightFinishOnTarget;
import redbacks.robot.actions.AcStraightLenient;
import redbacks.robot.actions.AcStraightPrecise;
import redbacks.robot.actions.AcTankDrive;
import redbacks.robot.actions.AcTurn;
import redbacks.robot.actions.AcTurnLenient;
import redbacks.robot.actions.AcTurnPrecise;

import static redbacks.robot.Auto.*;

@SuppressWarnings("deprecation")
public class OldAuto
{
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
			//4 cube CC HLHH
			case OLD_CC_HLHH_1:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.65 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(9),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.5, 9, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.3), -0.55, -0.55),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCube),
					new AcStraightLenient(-0.65, 80, sensors.distanceEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
//					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightLenient(0.35, 80, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightLenient(-0.5, 0, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(65),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.4, 60, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcSeq.Parallel(intakeCubeSlow),
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-65, sensors.yaw, false, false, false)),
							new SwitchAction(
									new AcDoNothing(new ChTrue()),
									new OptionAction(new ChNumSen(cube4ToHR3.totalDistance - 0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false), new AcSeq.Parallel(highFireRelease))
							)
					),
					new AcPath(new ChFalse(), true, cube4ToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//3 cube CF LHH
			case OLD_CF_LHH_1:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightLenient(3, -11, sensors.distanceEncoder, true),
					//2nd cube
					new AcStraightLenient(4.85, 5, sensors.distanceEncoder, false),
					new AcTurn(40),
					new AcSeq.Parallel(sequencer,
							new AcTankDrive(new ChTime(0.25), 0.5, 0.5)
					),
					new AcDoNothing(new ChMulti(
							LogicOperators.OR,
							new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false),
							new ChNumSen(3, new SenTimer())
					)),
					new SwitchAction(
							new AcSeq.Parallel(
									new AcInterrupt.KillSubsystem(sequencer),
									new AcStraightLenient(-0.7, 40, sensors.distanceEncoder, true),
									new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
									new AcWait(0.25),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcTurn(130),
									new AcStraightLenient(-0.7, 130, sensors.averageEncoder, true),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
											new AcSeq.Parallel(highFirePrime)
									),
									new AcStraightFinishOnTarget(-4.75, 90, sensors.averageEncoder, false,
											new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
											new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
									new AcInterrupt.KillSubsystem(intake),
									new AcTurn(12),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
											new AcSeq.Parallel(highFireRelease)
									),
									new AcStraightLenient(1.05, 12, sensors.distanceEncoder, true),
									//3rd cube
									new AcSetArm(-armBasePos),
									new AcTurn(-2),
									new AcSeq.Parallel(intakeCubeFast),
									new AcStraightLenient(-1, -2, sensors.distanceEncoder, true),
									new AcTankDrive(new ChTime(1), -0.6, -0.6),
									new AcSeq.Parallel(highFirePrime),
									new AcSetArm(0),
									new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
									new AcInterrupt.KillSubsystem(intake),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-0.75 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
											new AcSeq.Parallel(highFireRelease)
									),
									new AcStraightLenient(0, -2, sensors.distanceEncoder, false)),
							new OptionAction(new ChNumSen(-armBasePos + 50, sensors.armEncoder, true, false, false),
									new AcSeq.Parallel(
											new AcInterrupt.KillSubsystem(sequencer),
											new AcSetArm(-armSwitchPos),
											new AcWait(0.25),
											new AcTankDrive(new ChTime(0.75), 0.5, 0.5),
											new AcSetArm(-armBasePos),
											new AcTankDrive(new ChTime(1.25), -0.5, -0.5),
											new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
											new AcWait(0.25),
											new AcSetArm(0),
											new AcSeq.Parallel(intakeCubeSlow),
											new AcTurn(130),
											new AcStraightLenient(-0.7, 130, sensors.averageEncoder, true),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
													new AcSeq.Parallel(highFirePrime)
											),
											new AcStraightFinishOnTarget(-4.75, 90, sensors.averageEncoder, false,
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
											new AcInterrupt.KillSubsystem(intake),
											new AcTurn(12),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
													new AcSeq.Parallel(highFireRelease)
											),
											new AcStraightLenient(1.05, 12, sensors.distanceEncoder, true)
									)
							)
					)
				);
			//3 cube FC HLH
			case OLD_FC_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.65 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcStraightPrecise(new ChNumSen(1.2, new SenTimer()), wallToHR2.totalDistance / encoderTicksPerMetre - 0.7, 0, sensors.distanceEncoder, false,
							new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -0.7),
							new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, 0.7)),
					new AcTurn(90),
					new AcSetArm(-armSwitchPos),
					new AcStraightLenient(-2.45, 90, sensors.averageEncoder, true,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
					new AcTankDrive(new ChTime(0.1), 0.1, 0.1),
					new AcSetArm(-armBasePos),
					new AcTurn(30),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-0.5, 30, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.25),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcTurn(-75),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(sequencer, new AcStraightLenient(0.3, -70, sensors.distanceEncoder, true)),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcInterrupt.KillSubsystem(sequencer),
					new AcSeq.Parallel(intakeCube),
					new AcStraightLenient(-0.7, -70, sensors.distanceEncoder, false),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(-115),
					new SwitchAction(
							new AcDoNothing(new ChTrue()),
							new OptionAction(new ChNumSen(-110, sensors.yaw, true, false, false), new AcTankDrive(new ChTime(0.5), -0.7, 0.3))
					),
					new SwitchAction(
							new AcInterrupt.KillSubsystem(intake),
							new OptionAction(new ChNumSen(-105, sensors.yaw, false, false, false),
									new AcSeq.Parallel(
											new AcSeq.Parallel(intakeCube),
											new AcSolenoid.Single(driver.centreEncoderSol, true),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, true)),
													new AcSeq.Parallel(highFirePrime),
													new AcInterrupt.KillSubsystem(intake)
											),
											new AcStraightFinishOnTarget(-4.8, -120, sensors.averageEncoder, true,
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.2 * encoderTicksPerMetre), -0.6),
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.2 * encoderTicksPerMetre), 0.6)),
											new AcSetArm(armScalePos),
											new AcTurn(-90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//3 cube FF HLH
			case OLD_FF_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraightLenient(5.1, 0, sensors.distanceEncoder, true,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
					new AcTurnLenient(90),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightFinishOnTarget(-5.15, 90, sensors.averageEncoder, true,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.6)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.45 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightLenient(1, 10, sensors.distanceEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-7),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.2, -7, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(1), -0.6, -0.6),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
					new AcTurn(-80),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(intakeCubeFast),
					new AcSeq.Parallel(sequencer, new AcStraightLenient(0.3, -80, sensors.distanceEncoder, true)),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcInterrupt.KillSubsystem(sequencer),
					new AcStraightLenient(-0.7, -80, sensors.distanceEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube2ToHL.totalDistance - 0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube2ToHL, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//4 cube CC HLHH
			case OLD_CC_HLHH_2:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR2.totalDistance - 0.42 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR2.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR2, sensors.distanceEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
							new AcPath.ChangeMinMax(wallToHR2, sensors.distanceEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(8),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.3, 8, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.6, -0.6),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightLenient(-0.65, 80, sensors.distanceEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
//					new AcSeq.Parallel(intakeCubeSpin),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightLenient(0.35, 80, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightLenient(-0.5, 0, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(70),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.4, 65, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(47),
					new SwitchAction(
							new AcInterrupt.KillSubsystem(intake),
							new OptionAction(new ChNumSen(57, sensors.yaw, false, false, false),
									new AcSeq.Parallel(
											new AcSeq.Parallel(intakeCube),
											new AcSolenoid.Single(driver.centreEncoderSol, true),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
													new AcSeq.Parallel(highFirePrime),
													new AcInterrupt.KillSubsystem(intake)
											),
											new AcStraightFinishOnTarget(3.6, 52, sensors.averageEncoder, true,
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), -0.6),
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), 0.6)),
											new AcSetArm(-armScalePos),
											new AcTurn(90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//4 cube _C HHHH
			case OLD__C_HHHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.65 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(10),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.5, 10, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.3), -0.55, -0.55),
					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(0),
					new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightLenient(0, 5, sensors.distanceEncoder, false),
					//3rd cube
					new AcSetArm(-armBasePos),
					new AcTurn(20),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.2, 20, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcTurn(80),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightLenient(0.35, 80, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightLenient(-0.5, 0, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(63),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.4, 58, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(45),
					new SwitchAction(
							new AcInterrupt.KillSubsystem(intake),
							new OptionAction(new ChNumSen(60, sensors.yaw, false, false, false),
									new AcSeq.Parallel(
											new AcSeq.Parallel(intakeCube),
											new AcSolenoid.Single(driver.centreEncoderSol, true),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
													new AcSeq.Parallel(highFirePrime),
													new AcInterrupt.KillSubsystem(intake)
											),
											new AcStraightFinishOnTarget(3.6, 50, sensors.averageEncoder, true,
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), -0.6),
													new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), 0.6)),
											new AcSetArm(-armScalePos),
											new AcTurn(90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//3 cube _F HHH
			case OLD__F_HHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraightLenient(5, 0, sensors.distanceEncoder, true,
							new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
					new AcTurnLenient(90),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightFinishOnTarget(-5.15, 90, sensors.averageEncoder, true,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.6)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightLenient(1, 10, sensors.distanceEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-5),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1, -5, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.7), -0.6, -0.6),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(0),
					new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightLenient(0, -5, sensors.distanceEncoder, false),
					//3rd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-20),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.2, -20, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcWait(0.2),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcTurnPrecise(-75),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightLenient(0.35, -75, sensors.distanceEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube2ToHL.totalDistance - 0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube2ToHL, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//Coop CC HLH
			case OLD_COOP_CC_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR2.totalDistance - 0.42 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR2.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR2, sensors.distanceEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
							new AcPath.ChangeMinMax(wallToHR2, sensors.distanceEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(10),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightLenient(-1.3, 10, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.6, -0.6),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightLenient(-0.65, 80, sensors.distanceEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcStraightLenient(1.5, 90, sensors.distanceEncoder, true),
					new AcTurn(0),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightLenient(2, 0, sensors.distanceEncoder, true),
					new AcSetArm(armScalePos),
					new AcTurn(-90),
					new AcSeq.Parallel(
							new AcWait(0.3),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcTankDrive(new ChTime(1), 0.5, 0.5)
				);
			//Coop CF LHP
			case OLD_COOP_CF_LHP:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightLenient(3, -12, sensors.distanceEncoder, true),
					//2nd cube
					new AcStraightLenient(5, 5, sensors.distanceEncoder, false),
					new AcTurn(40),
					new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
					new AcStraightLenient(-0.7, 40, sensors.distanceEncoder, true),
					new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
					new AcWait(0.25),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(130),
					new AcStraightLenient(-0.7, 130, sensors.averageEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcStraightFinishOnTarget(-4.75, 90, sensors.averageEncoder, false,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
					new AcInterrupt.KillSubsystem(intake),
					new AcTurn(12),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightLenient(1.05, 12, sensors.distanceEncoder, true),
					//3rd cube
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcStraightLenient(0.3, 0, sensors.distanceEncoder, false,
							new AcStraight.ChangeMinMax(autoDistanceEncoder, (int) (0.3 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(autoDistanceEncoder, (int) (0.3 * encoderTicksPerMetre), 0.6)),
					new AcTurn(90),
					new AcSetArm(-armSwitchPos),
					new AcStraightFinishOnTarget(3.2, 90, sensors.averageEncoder, true,
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
							new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
					new AcSetArm(-armBasePos),
					new AcTurn(0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcTankDrive(new ChTime(1.5), -0.5, -0.5),
					new AcSetArm(0),
					new AcInterrupt.KillSubsystem(intake)
				);
			//1 cube C_ L
			case OLD_C__L:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder)),
								new AcSeq.Parallel(solExtendIntakeL),
								new AcWait(1),
								new AcSeq.Parallel(solRetractIntakeL)
						),
						new AcStraightLenient(3, -12, sensors.distanceEncoder, true),
						new AcStraightLenient(-0.5, -40, sensors.distanceEncoder, true)
				);
			//1 cube _C H
			case OLD__C_H:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.65 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.distanceEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.distanceEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55))
				);
			case OLD_CC_LHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightLenient(3, -11, sensors.distanceEncoder, true),
					//2nd cube
					new AcStraightLenient(4.85, 5, sensors.distanceEncoder, false),
					new AcTurn(40),
					new AcSeq.Parallel(sequencer,
							new AcTankDrive(new ChTime(0.25), 0.5, 0.5)
					),
					new AcDoNothing(new ChMulti(
							LogicOperators.OR,
							new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false),
							new ChNumSen(3, new SenTimer())
					)),
					new SwitchAction(
							new AcSeq.Parallel(
									new AcInterrupt.KillSubsystem(sequencer),
									new AcStraightLenient(-0.7, 40, sensors.distanceEncoder, true,
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -0.7),
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, 0.7)),
									new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
									new AcWait(0.25),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcTurn(20),
									new AcSeq.Parallel(highFirePrime),
									new AcStraightLenient(1.5, 20, sensors.distanceEncoder, true),
									new AcSetArm(-armScalePos),
									new AcTurn(100),
									new AcWait(0.25),
									new AcSeq.Parallel(highFireRelease),
									new AcWait(0.5),
									new AcSetArm(-armBasePos),
									new AcTurn(35),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
											new AcSeq.Parallel(intakeCube)
									),
									new AcStraightLenient(-2.2, 35, sensors.distanceEncoder, true,
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
									new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
									new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcSeq.Parallel(highFirePrime),
									new AcStraightLenient(2.2, 35, sensors.distanceEncoder, true,
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.7),
											new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.7)),
									new AcSetArm(-armScalePos),
									new AcTurn(90),
									new AcWait(0.25),
									new AcSeq.Parallel(highFireRelease)
							),
							new OptionAction(new ChNumSen(-armBasePos + 50, sensors.armEncoder, true, false, false),
									new AcSeq.Parallel(
											new AcInterrupt.KillSubsystem(sequencer),
											new AcSetArm(-armSwitchPos),
											new AcWait(0.25),
											new AcTankDrive(new ChTime(0.75), 0.5, 0.5),
											new AcSetArm(-armBasePos),
											new AcTankDrive(new ChTime(1.25), -0.5, -0.5),
											new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
											new AcWait(0.25),
											new AcSetArm(0),
											new AcSeq.Parallel(intakeCubeSlow),
											new AcTurn(20),
											new AcSeq.Parallel(highFirePrime),
											new AcStraightLenient(1.5, 20, sensors.distanceEncoder, true),
											new AcSetArm(-armScalePos),
											new AcTurn(100),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//2 cube FF HL
			case OLD_FF_HL:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraightLenient(4.9, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
						new AcTurnLenient(90),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcStraightFinishOnTarget(-5.3, 90, sensors.averageEncoder, true,
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.6)),
						new AcInterrupt.KillSubsystem(intake),
						new AcTurn(155),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightLenient(-1.4, 155, sensors.distanceEncoder, true),
						new AcTurn(100),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//2nd cube
						new AcWait(0.5),
						new AcSetArm(armBasePos),
						new AcTurn(160),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightLenient(1.6, 160, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
						new AcTankDrive(new ChTime(0.7), 0.5, 0.5),
						new AcSetArm(armSwitchPos + 75),
						new AcSeq.Parallel(
								new AcWait(0.5),
								new AcInterrupt.KillSubsystem(intake),
								new AcWait(0.25),
								new AcSeq.Parallel(outtakeCube)
						),
						new AcWait(0.25),
						new AcTankDrive(new ChTime(0.75), 0.5, 0.5),
						new AcInterrupt.KillSubsystem(intake)
					);
//			case FF_HH:
//				return createAuto(
//						new AcResetSensors(),
//						//1st cube
//						new AcStraight(5.1, 0, sensors.driveCentreEncoder, true,
//								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
//								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
//						new AcTurnLenient(90),
//						new AcSeq.Parallel(intakeCubeSlow),
//						new AcStraightSafe(-5.15, 90, sensors.averageEncoder, true,
//								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
//								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.6)),
//						new AcInterrupt.KillSubsystem(intake),
//						new AcSeq.Parallel(highFirePrime),
//						new AcTurn(150),
//						new AcSetArm(armScalePos),
//						new AcStraight(-1.4, 150, sensors.driveCentreEncoder, true),
//						new AcTurn(90),
//						new AcWait(0.25),
//						new AcSeq.Parallel(highFireRelease),
//						//2nd cube
//						new AcWait(0.5),
//						new AcSetArm(armBasePos),
//						new AcTurn(160),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
//								new AcSeq.Parallel(intakeCube)
//						),
//						new AcStraight(1.6, 160, sensors.driveCentreEncoder, true,
//								new AcStraight.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
//								new AcStraight.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
//						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
//						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
//						new AcSetArm(0),
//						new AcSeq.Parallel(intakeCubeSlow),
//						new AcSeq.Parallel(highFirePrime),
//						new AcStraight(-1.6, 155, sensors.driveCentreEncoder, true,
//								new AcStraight.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
//								new AcStraight.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
//						new AcSetArm(armScalePos),
//						new AcTurn(90),
//						new AcWait(0.25),
//						new AcSeq.Parallel(highFireRelease)
//					);
				//2 cube _F HH
				case OLD__F_HH:
					return createAuto(
							new AcResetSensors(),
							//1st cube
							new AcStraightLenient(4.9, 0, sensors.distanceEncoder, true,
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
							new AcTurnLenient(90),
							new AcSeq.Parallel(intakeCubeSlow),
							new AcStraightFinishOnTarget(-5.3, 90, sensors.averageEncoder, true,
									new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
									new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.6)),
							new AcInterrupt.KillSubsystem(intake),
							new AcTurn(155),
							new AcSetArm(armScalePos),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
									new AcSeq.Parallel(highFirePrime)
							),
							new AcStraightLenient(-1.4, 155, sensors.distanceEncoder, true),
							new AcTurn(100),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease),
							//2nd cube
							new AcWait(0.5),
							new AcSetArm(armBasePos),
							new AcTurn(160),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
									new AcSeq.Parallel(intakeCube)
							),
							new AcStraightLenient(1.6, 160, sensors.distanceEncoder, true,
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
							new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
							new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
							new AcSetArm(0),
							new AcSeq.Parallel(intakeCubeSlow),
							new AcSeq.Parallel(highFirePrime),
							new AcStraightLenient(-1.6, 155, sensors.distanceEncoder, true,
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
									new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
							new AcSetArm(armScalePos),
							new AcTurn(90),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease)
						);
			//2 cube FC HL
			case OLD_FC_HL:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightLenient(6.25, 0.5, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (5 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (5 * encoderTicksPerMetre), 0.6)),
						new AcSetArm(-armScalePos),
						new AcTurn(105),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//2nd cube
						new AcWait(0.5),
						new AcTurn(20),
						new AcStraightFinishOnTarget(5.55, 20, sensors.distanceEncoder, false,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -0.65),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, 0.65)),
						new AcTurn(90),
						new AcStraightFinishOnTarget(-3.3, 90, sensors.averageEncoder, true,
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
						new AcTurn(5),
						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
						new AcSetArm(-armBasePos),
						new AcSeq.Parallel(intakeCubeFast),
						new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
						new AcStraightLenient(-0.9, 5, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -0.5),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, 0.5)),
						new AcSetArm(-armSwitchPos - 75),
						new AcSeq.Parallel(
								new AcWait(0.5),
								new AcInterrupt.KillSubsystem(intake)
						),
						new AcWait(0.25),
						new AcTankDrive(new ChTime(0.75), -0.5, -0.5),
						new AcSeq.Parallel(outtakeCubeSpinRight),
						new AcWait(0.25),
						new AcInterrupt.KillSubsystem(intake)
				);
			case OLD_CF_LHH_2:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1.5 * encoderTicksPerMetre, sensors.distanceEncoder)),
								new AcSeq.Parallel(solExtendIntakeL),
								new AcWait(1),
								new AcSeq.Parallel(solRetractIntakeL),
								new AcSetArm(-armBasePos),
								new AcSeq.Parallel(intakeCubeSpin)
						),
						new AcStraightLenient(3, -11, sensors.distanceEncoder, true),
						//2nd cube
						new AcStraightLenient(5.1, 5, sensors.distanceEncoder, false),
						new AcTurn(33),
						new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
						new AcStraightLenient(-0.7, 33, sensors.distanceEncoder, true),
						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
						new AcWait(0.25),
						new AcSetArm(0),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcTurn(130),
						new AcStraightLenient(-0.7, 130, sensors.averageEncoder, true),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightFinishOnTarget(-4.75, 90, sensors.averageEncoder, false,
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.6)),
						new AcInterrupt.KillSubsystem(intake),
						new AcTurn(12),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.475 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightLenient(1.05, 12, sensors.distanceEncoder, true),
						//3rd cube
						new AcSetArm(-armBasePos),
						new AcTurn(-4),
						new AcSeq.Parallel(intakeCubeFast),
						new AcStraightLenient(-1, -4, sensors.distanceEncoder, true),
						new AcTankDrive(new ChTime(1), -0.6, -0.6),
						new AcSeq.Parallel(highFirePrime),
						new AcSetArm(0),
						new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-0.85 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightLenient(0, -4, sensors.distanceEncoder, false)
					);
			//3 cube _C HHH
			case OLD__C_HHH:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(6 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightLenient(6.25, 0.5, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (5 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (5 * encoderTicksPerMetre), 0.6)),
						new AcSetArm(armScalePos),
						new AcTurn(-70),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//2nd cube
						new AcWait(0.5),
						new AcSetArm(armBasePos),
						new AcTurn(-150),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightLenient(1.8, -150, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
						new AcTankDrive(new ChTime(0.7), 0.5, 0.5),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, false)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightLenient(0.1, -150, sensors.distanceEncoder, false),
						new AcTurn(-80),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//3rd cube
						new AcWait(0.5),
						new AcSetArm(armBasePos),
						new AcTurn(-140),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightLenient(2.4, -140, sensors.averageEncoder, true,
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), -0.6),
								new AcStraight.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), 0.6)),
						new AcTankDrive(new ChTime(1), 0.5, 0.5),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightLenient(0.3, -140, sensors.averageEncoder, false),
						new AcTurn(-80),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease)
				);
			default: return null;
		}
	}
}
