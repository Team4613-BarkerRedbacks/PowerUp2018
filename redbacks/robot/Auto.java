package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import static redbacks.arachne.lib.override.MotionSettings2.encoderTicksPerMetre;

import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.actions.AcDriveDirection;
import redbacks.robot.actions.AcIntakeRightSide;
import redbacks.robot.actions.AcResetSensors;
import redbacks.robot.actions.AcSetArm;
import redbacks.robot.actions.AcStraight;
import redbacks.robot.actions.AcTankDrive;
import redbacks.robot.actions.AcTurn;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;
import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;
//NOTE ON NAMING CONVENTION
//1 IS CLOSE, 2 IS FAR,0 AFTER IS RIGHT SIDE, 1 AFTER IS LEFT SIDE, FOURTH DIGIT 0 IS INDIVIDUAL, FOURTH DIGIT 1 IS COOPERATIVE
public class Auto extends AutoStart
{
	public static SenCANEncoder.Displacement autoDistanceEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			case(200):
				return createAuto(
					new AcResetSensors(),
					new AcTurn(90),
					new AcTurn(0),
					new AcTurn(135)
				);
			case(201):
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.42 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(12),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-1.25, 12, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.6, -0.6),
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
					new AcSeq.Parallel(
							new AcWait(0.7),
							new AcSetArm(-armBasePos)
					),
					new AcStraight(0.7, 12, sensors.driveCentreEncoder, true),
					new AcTurn(45),
					new AcSeq.Parallel(intakeCubeSpin),
					new AcStraight(-0.9, 45, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcTurn(60),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcStraight(-1.2, 55, autoDistanceEncoder, false),
					new AcTurn(-10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(0, -10, autoDistanceEncoder, false)//,
					//4th cube
//					new AcDriveDirection(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, true), -0.7, 10),
//					new AcDriveDirection(new ChNumSen(45, sensors.yaw, true, false, false), 0, 45),
//					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
//							new AcWait(1.5),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
//							new AcSeq.Parallel(intakeCube),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(0)
//					),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.5)
//					), -0.7, 40),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//					new AcTankDrive(new ChNumSen(-5, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcDriveDirection(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, -5)
				);
//			case(1100):
//				//CC RIGHT INDIVIDUAL
//				return createAuto(
//					new AcResetSensors(),
//					//1st cube
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFirePrime),
//							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.42 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcPath(new ChMulti(
//									LogicOperators.AND,
//									new ChTime(4.5),
//									new ChNumSen(wallToHR.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
//							), true, wallToHR, driver.drivetrain, 1, 1,
//							sensors.yaw, sensors.driveCentreEncoder, false, 
//							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
//							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
//					//2nd cube
//					new AcSetArm(-armBasePos),
//					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.9, -0.9),
//					new AcSetNumSen(autoDistanceEncoder, 0),
//					new AcSeq.Parallel(intakeCube),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(-1.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true),
//							new ChTime(2.5)
//					), -0.6, 10),
//					new AcSetArm(-armSwitchPos),
//					new AcSeq.Parallel(
//							new AcWait(0.25),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcWait(0.25),
//							new AcSeq.Parallel(outtakeCube)
//					),
//					new AcWait(0.25),
//					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
//					new AcWait(0.25),
//					new AcInterrupt.KillSubsystem(intake),
//					//3rd cube
//					new AcSeq.Parallel(
//							new AcWait(0.7),
//							new AcSetArm(-armBasePos)
//					),
//					new AcDriveDirection(new ChNumSen(-1.5 * encoderTicksPerMetre, autoDistanceEncoder, false, false, true), 0.8, -1.5),
//					new AcTankDrive(new ChNumSen(45, sensors.yaw, true, false, false), 0.8, -0.8),
//					new AcDriveDirection(new ChNumSen(1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, true), -0.6, 1.2),
//					//new AcTankDrive(new ChNumSen(, sensors.yaw, true, false, false), -0.8, 0.8),
////					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
////							new AcWait(1.25),
////							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(2.25)))
//					),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.25)
//					), -0.7, 37),
//					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//					new AcSeq.Parallel(highFirePrime),
//					new AcSetArm(0),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//					new AcDriveDirection(new ChNumSen(0, sensors.yaw, false, false, false), 0, 0),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//					//4th cube
//					new AcDriveDirection(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, true), -0.7, 10),
//					new AcDriveDirection(new ChNumSen(45, sensors.yaw, true, false, false), 0, 45),
//					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
//							new AcWait(1.5),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
//							new AcSeq.Parallel(intakeCube),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(0)
//					),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.5)
//					), -0.7, 40),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//					new AcTankDrive(new ChNumSen(-5, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcDriveDirection(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, -5)
//				);
//			case(100):
//				return createAuto(
//						new AcResetSensors(), 
//						new AcDriveDirection(new ChGettableNumber(2.0, sensors.speedForward, true, false), 0.8, 0)
//				);
//						
//			/*case(1101):
//				//CC RIGHT COOPERATIVE
//				return createAuto(
//					new AcResetSensors(),
//					//1st cube
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFirePrime),
//							new AcDoNothing(new ChNumSen(wallToHRWIDE.totalDistance - 0.42 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcPath(new ChMulti(
//									LogicOperators.AND,
//									new ChTime(4.5),
//									new ChNumSen(wallToHRWIDE.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
//							), true, wallToHRWIDE, driver.drivetrain, 1, 1,
//							//MAKE PATH THAT GOES WIDE
//							sensors.yaw, sensors.driveCentreEncoder, false, 
//							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//							new AcPath.ChangeMinMax(wallToHRWIDE, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
//							new AcPath.ChangeMinMax(wallToHRWIDE, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
//					//2nd cube
//					new AcSetArm(-armBasePos),
//					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.9, -0.9),
//					new AcSetNumSen(autoDistanceEncoder, 0),
//					new AcSeq.Parallel(intakeCube),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(-1.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true),
//							new ChTime(2.5)
//					), -0.6, 10),
//					new AcSetArm(-armSwitchPos),
//					new AcSeq.Parallel(
//							new AcWait(0.25),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcWait(0.25),
//							new AcSeq.Parallel(outtakeCube)
//					),
//					new AcWait(0.25),
//					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
//					new AcWait(0.25),
//					new AcInterrupt.KillSubsystem(intake),
//					//3rd cube
//					new AcSeq.Parallel(
//							new AcWait(0.7),
//							new AcSetArm(-armBasePos)
//					),
//					new AcDriveDirection(new ChNumSen(-1.5 * encoderTicksPerMetre, autoDistanceEncoder, false, false, true), 0.8, -1.5),
//					new AcTankDrive(new ChNumSen(45, sensors.yaw, true, false, false), 0.8, -0.8),
//					new AcDriveDirection(new ChNumSen(1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, true), -0.6, 1.2),
//					//new AcTankDrive(new ChNumSen(, sensors.yaw, true, false, false), -0.8, 0.8),
////					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
////							new AcWait(1.25),
////							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(2.25)))
//					),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.25)
//					), -0.7, 37),
//					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//					new AcSeq.Parallel(highFirePrime),
//					new AcSetArm(0),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//					new AcDriveDirection(new ChNumSen(0, sensors.yaw, false, false, false), 0, 0),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//					//4th cube
//					new AcDriveDirection(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, true), -0.7, 10),
//					new AcDriveDirection(new ChNumSen(45, sensors.yaw, true, false, false), 0, 45),
//					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
//							new AcWait(1.5),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
//							new AcSeq.Parallel(intakeCube),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(0)
//					),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.5)
//					), -0.7, 40),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//					new AcTankDrive(new ChNumSen(-5, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcDriveDirection(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, -5)
//				);*/
//			case(1200):
//				//CF RIGHT INDIVIDUAL
//				return createAuto(
//					new AcResetSensors(),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(3.5 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
//							new AcSeq.Parallel(solExtendIntakeL),
//							new AcWait(0.5),
//							new AcSeq.Parallel(solRetractIntakeL)
//					),
//					new AcDriveDirection(new ChNumSen(5.5 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.8, 0),
//					new AcSetArm(-armBasePos),
//					new AcDriveDirection(new ChNumSen(40, sensors.yaw, true, false, false), 0, 40),
//					new AcSeq.Parallel(intakeCube),
//					new AcDriveDirection(new ChNumSen(-0.9 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true), -0.6, 40),
//					new AcSetArm(0),
//					new AcDriveDirection(new ChNumSen(0.9 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.8, 40),
//					new AcInterrupt.KillSubsystem(intake),
//					new AcDriveDirection(new ChNumSen(90, sensors.yaw, true, false, false), 0, 90),
//					new AcDriveDirection(new ChNumSen(-4.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true), -1, 90),
//					new AcSeq.Parallel(highFirePrime),
//					new AcDriveDirection(new ChNumSen(10, sensors.yaw, false, false, false), 0, 10),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(1.7 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcDriveDirection(new ChNumSen(2.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.8, 10),
//					//3rd cube
//					new AcSetArm(-armBasePos),
//					new AcTankDrive(new ChNumSen(-3, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSetNumSen(autoDistanceEncoder, 0),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
//					new AcTankDrive(new ChNumSen(-30, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
//							new AcWait(1),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(1)))
//					),
//					new AcTankDrive(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1)
//					), -0.7, -0.7),
//					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//					new AcSeq.Parallel(highFirePrime),
//					new AcSetArm(0),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.8, -0.8),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//					//4th cube
//					new AcTankDrive(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
//					new AcTankDrive(new ChNumSen(-40, sensors.yaw, false, false, false), -0.8, 0.8),
//					new AcSeq.Parallel(intakeCube),
//					new AcSeq.Parallel(
//							new AcWait(1.25),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
//							new AcSeq.Parallel(intakeCube),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(0)
//					),
//					new AcTankDrive(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//							new ChTime(1.5)
//					), -0.7, -0.7),
//					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.8, -0.8),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8)
//				);
//			case(2100):
//				//FC RIGHT INDIVIDUAL
//				return createAuto(
//						new AcResetSensors(),
//						//1st cube
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//								new AcSeq.Parallel(highFirePrime),
//								new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.55 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcPath(new ChMulti(
//										LogicOperators.AND,
//										new ChTime(4.5),
//										new ChNumSen(wallToHR.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
//								), true, wallToHR, driver.drivetrain, 1, 1,
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//								new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
//								new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
//						//2nd cube
//						new AcSetNumSen(autoDistanceEncoder, 0),
//						new AcSetArm(-armBasePos),
//						new AcSeq.Parallel(intakeCube),
//						new AcDriveDirection(new ChNumSen(-0.95 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true), -0.8, 0),
//						new AcDriveDirection(new ChNumSen(90, sensors.yaw, true, false, false), 0, 90),
//						new AcPath(new ChFalse(), true, RSideToCube3, driver.drivetrain, 1, 1,
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//								new AcPath.ChangeMinMax(RSideToCube3, sensors.driveCentreEncoder, (int) (0.5 * encoderTicksPerMetre), -0.6),
//								new AcPath.ChangeMinMax(RSideToCube3, sensors.driveCentreEncoder, (int) (0.5 * encoderTicksPerMetre), 0.6)),
//						new AcInterrupt.KillSubsystem(intake),
//						new AcSetArm(-armSwitchPos),
//						new AcDoNothing(new ChNumSen(-armSwitchPos, sensors.armEncoder, true, false, false)),
//						new AcSeq.Parallel(outtakeCubeFast),
//						new AcSeq.Parallel(
//								new AcWait(0.25),
//								new AcSetArm(-armBasePos),
//								new AcInterrupt.KillSubsystem(intake)
//						),
//						new AcDriveDirection(new ChNumSen(-10, sensors.yaw, true, false, false), 0, -10),
//						new AcSeq.Parallel(intakeCube),
//						new AcTankDrive(new ChNumSen(encoderTicksPerMetre, sensors.driveCentreEncoder), -0.6, -0.6),
//						new AcInterrupt.KillSubsystem(intake),
//						new AcSetArm(0),
//						new AcPath(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), true, cube2ToRSide, driver.drivetrain, 1, 1,
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//								new AcPath.ChangeMinMax(cube2ToRSide, sensors.driveCentreEncoder, (int) (0.5 * encoderTicksPerMetre), -0.6),
//								new AcPath.ChangeMinMax(cube2ToRSide, sensors.driveCentreEncoder, (int) (0.5 * encoderTicksPerMetre), 0.6)),
//						new AcSeq.Parallel(highFireRelease),
//						new AcDriveDirection(new ChNumSen(-100, sensors.yaw, false, false, false), 0, -100),
//						new AcSetNumSen(sensors.yaw, -10),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(-0.6 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcDriveDirection(new ChNumSen(0, autoDistanceEncoder), 0.6, -10),
//						new AcSetArm(-armBasePos),
//						new AcTankDrive(new ChNumSen(3, sensors.yaw, true, false, false), 0.8, -0.8),
//						new AcSetNumSen(autoDistanceEncoder, 0),
//						new AcSeq.Parallel(intakeCube),
//						new AcPath(new ChTime(1.25), true, HRToCube6, driver.drivetrain, 1, 1, 
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
//						new AcSetArm(0),
//						new AcSeq.Parallel(highFirePrime),
//						new AcSeq.Parallel(
//								new AcWait(0.25),
//								new AcInterrupt.KillSubsystem(intake)
//						),
//						new AcDriveDirection(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.8, 15),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(-0.6 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcDriveDirection(new ChNumSen(0.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.8, -10)
//				);
//			
//			case(2200):
//				//FF RIGHT INDIVIDUAL
//				return createAuto(
//					new AcResetSensors(),
//					new AcDriveDirection(new ChNumSen(4.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.9, 0),
//					new AcDriveDirection(new ChNumSen(-90, sensors.yaw, false, false, false), 0, -90),
//					new AcDriveDirection(new ChNumSen(4.5 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.9, -90),
//					new AcSeq.Parallel(highFirePrime),
//					new AcDriveDirection(new ChNumSen(10, sensors.yaw, true, false, false), 0, 10),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(0.7 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcDriveDirection(new ChNumSen(1.25 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true), 0.7, 10),
//					//2nd cube
//					new AcSetArm(-armBasePos),
//					new AcTankDrive(new ChNumSen(-10, sensors.yaw, false, false, false), -0.9, 0.9),
//					new AcSetNumSen(autoDistanceEncoder, 0),
//					new AcSeq.Parallel(intakeCube),
//					new AcDriveDirection(new ChMulti(
//							LogicOperators.OR,
//							new ChNumSen(-1.32 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true),
//							new ChTime(2)
//					), -0.65, 10),
//					new AcSetArm(-armSwitchPos),
//					new AcSeq.Parallel(
//							new AcWait(0.25),
//							new AcInterrupt.KillSubsystem(intake),
//							new AcWait(0.25),
//							new AcSeq.Parallel(outtakeCube)
//					),
//					new AcWait(0.25),
//					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
//					new AcInterrupt.KillSubsystem(intake)//,
////					//3rd cube
////					new AcSetArm(-armBasePos),
////					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
////					new AcTankDrive(new ChNumSen(-30, sensors.yaw, false, false, false), -0.8, 0.8),
////					new AcSeq.Parallel(intakeCube),
////					new AcSeq.Parallel(
////							new AcWait(1),
////							new AcInterrupt.KillSubsystem(intake),
////							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(1)))
////					),
////					new AcTankDrive(new ChMulti(
////							LogicOperators.OR,
////							new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
////							new ChTime(1)
////					), -0.7, -0.7),
////					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
////					new AcSeq.Parallel(highFirePrime),
////					new AcSetArm(0),
////					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
////					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.8, -0.8),
////					new AcSeq.Parallel(
////							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
////							new AcSeq.Parallel(highFireRelease)
////					),
////					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
////					//4th cube
////					new AcTankDrive(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
////					new AcTankDrive(new ChNumSen(-40, sensors.yaw, false, false, false), -0.8, 0.8),
////					new AcSeq.Parallel(intakeCube),
////					new AcSeq.Parallel(
////							new AcWait(1.25),
////							new AcInterrupt.KillSubsystem(intake),
////							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
////							new AcSeq.Parallel(intakeCube),
////							new AcSeq.Parallel(highFirePrime),
////							new AcSetArm(0)
////					),
////					new AcTankDrive(new ChMulti(
////							LogicOperators.OR,
////							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
////							new ChTime(1.5)
////					), -0.7, -0.7),
////					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
////					new AcTankDrive(new ChNumSen(10, sensors.yaw, true, false, false), 0.8, -0.8),
////					new AcSeq.Parallel(
////							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
////							new AcSeq.Parallel(highFireRelease)
////					),
////					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8)
//				);
//			
//			case(50):
//				return createAuto(
//						new AcResetSensors(),
//						//1st cube
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//								new AcSeq.Parallel(highFirePrime),
//								new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.55 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcPath(new ChMulti(
//										LogicOperators.AND,
//										new ChTime(4.5),
//										new ChNumSen(wallToHR.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
//								), true, wallToHR, driver.drivetrain, 1, 1,
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
//								new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.6),
//								new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.6)),
//						//2nd cube
//						new AcSetArm(-armBasePos),
//						new AcTankDrive(new ChNumSen(3, sensors.yaw, true, false, false), 0.8, -0.8),
//						new AcSetNumSen(autoDistanceEncoder, 0),
//						new AcSeq.Parallel(intakeCube),
//						new AcPath(new ChTime(1.25), true, HRToCube6, driver.drivetrain, 1, 1, 
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
//						new AcSetArm(0),
//						new AcSeq.Parallel(highFirePrime),
//						new AcSeq.Parallel(
//								new AcWait(0.25),
//								new AcInterrupt.KillSubsystem(intake)
//						),
//						new AcDriveDirection(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.8, 15),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(-0.6 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcDriveDirection(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, -10),
//						//3rd cube
//						new AcSetArm(-armBasePos),
//						new AcTankDrive(new ChNumSen(3, sensors.yaw, true, false, false), 0.8, -0.8),
//						new AcSetNumSen(autoDistanceEncoder, 0),
//						new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
//						new AcTankDrive(new ChNumSen(30, sensors.yaw, true, false, false), 0.8, -0.8),
//						new AcSeq.Parallel(intakeCube),
//						new AcSeq.Parallel(
//								new AcWait(1),
//								new AcInterrupt.KillSubsystem(intake),
//								new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(1)))
//						),
//						new AcTankDrive(new ChMulti(
//								LogicOperators.OR,
//								new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//								new ChTime(1)
//						), -0.7, -0.7),
//						new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//						new AcSeq.Parallel(highFirePrime),
//						new AcSetArm(0),
//						new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//						new AcTankDrive(new ChNumSen(-10, sensors.yaw, false, false, false), -0.8, 0.8),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
//						//4th cube
//						new AcTankDrive(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
//						new AcTankDrive(new ChNumSen(40, sensors.yaw, true, false, false), 0.8, -0.8),
//						new AcSeq.Parallel(intakeCube),
//						new AcSeq.Parallel(
//								new AcWait(1.25),
//								new AcInterrupt.KillSubsystem(intake),
//								new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
//								new AcSeq.Parallel(intakeCube),
//								new AcSeq.Parallel(highFirePrime),
//								new AcSetArm(0)
//						),
//						new AcTankDrive(new ChMulti(
//								LogicOperators.OR,
//								new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
//								new ChTime(1.5)
//						), -0.7, -0.7),
//						new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
//						new AcTankDrive(new ChNumSen(-10, sensors.yaw, false, false, false), -0.8, 0.8),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//								new AcSeq.Parallel(highFireRelease)
//						),
//						new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8)
//				);
////			case(60):
////				return createAuto(
////						new AcResetSensors(),
////						
////				);
//			case(3):
//				return createAuto(
//						new AcResetSensors(),
//						new AcSeq.Parallel(
//								new AcDoNothing(new ChNumSen(wallToHL2.totalDistance - 2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//								new AcSeq.Parallel(quickFire),
//								new AcSetArm(75)
//						),
//						new AcPath(new ChFalse(), true, wallToHL2, driver.drivetrain, 1, 1, 
//								sensors.yaw, sensors.driveCentreEncoder, false, 
//								new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
//				);
//			case(4):
//				return createAuto(
//					new AcResetSensors(),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFirePrime),
//							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 1.25 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcPath(new ChFalse(), true, wallToHL3, driver.drivetrain, 1, 1, 
//							sensors.yaw, sensors.driveCentreEncoder, false, 
//							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
//				);
//			case(5):
//				return createAuto(
//					new AcResetSensors(),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(wallToHL5.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(25),
//							new AcDoNothing(new ChNumSen(wallToHL5.totalDistance - 0.9 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcPath(new ChFalse(), true, wallToHL5, driver.drivetrain, 1, 1, 
//							sensors.yaw, sensors.driveCentreEncoder, false, 
//							new Tolerances.Absolute(0.1 * encoderTicksPerMetre),
//							new AcPath.ChangeMinMax(wallToHL5, sensors.driveCentreEncoder, (int) (1.25 * encoderTicksPerMetre), -0.6),
//							new AcPath.ChangeMinMax(wallToHL5, sensors.driveCentreEncoder, (int) (1.25 * encoderTicksPerMetre), 0.6))
//				);
//			case(6):
//				return createAuto(
//					new AcResetSensors(),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(wallToHL6.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFirePrime),
//							new AcSetArm(25),
//							new AcDoNothing(new ChNumSen(wallToHL6.totalDistance - 1.3 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcPath(new ChFalse(), true, wallToHL6, driver.drivetrain, 1, 1, 
//							sensors.yaw, sensors.driveCentreEncoder, false, 
//							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
//				);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
