package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import static redbacks.arachne.lib.override.MotionSettings2.encoderTicksPerMetre;

import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.actions.*;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;
import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart
{
	public static SenCANEncoder.Displacement autoDistanceEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			//3 cube FC HLH
//			case(1203):
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
//					new AcSetNumSen(autoDistanceEncoder, 0),
//					new AcStraight(-0.7, 0, sensors.driveCentreEncoder, true),
//					new AcTurn(90),
//					new AcSetArm(-armBasePos + 100),
//					new AcStraight(-1.75, 90, sensors.driveCentreEncoder, true),
//					new AcTurnPrecise(25),
//					new AcSeq.Parallel(intakeCubeSpin),
//					new AcSetArm(-armBasePos),
//					new AcStraight(-0.6, 25, sensors.driveCentreEncoder, true),
//					new AcWait(0.25),
//					new AcSetArm(-armSwitchPos),
//					new AcTankDrive(new ChTime(0.25), 0.6, 0.6),
//					new AcWait(0.25),
//					new AcInterrupt.KillSubsystem(intake),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
//							new AcSeq.Parallel(outtakeCubeFast),
//							new AcWait(0.5),
//							new AcInterrupt.KillSubsystem(intake)
//					),
//					new AcTurn(80)
//				);
			case(203):
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
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcStraight(-0.85, 0, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcWait(0.2),
							new AcPauseNumSen(autoDistanceEncoder, true)
					),
					new AcTurn(90),
					new AcSetArm(armSwitchPos),
					new AcStraight(-2.35, 90, sensors.averageEncoder, true),
					new AcTankDrive(new ChTime(0.2), 0.1, 0.1),
					new AcStraightPrecise(-2.35, 90, sensors.averageEncoder, false),
					new AcSetArm(-armBasePos),
					new AcTurn(30),
					new AcSeq.Parallel(intakeCube),
					new AcPauseNumSen(autoDistanceEncoder, false),
					new AcStraight(-1.7, 30, autoDistanceEncoder, false),
					new AcTankDrive(new ChTime(0.25), -0.6, -0.6),
					new AcSeq.Parallel(intakeCubeSlow),
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
					new AcStraight(0.3, -70, sensors.driveCentreEncoder, true),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-0.4, -70, sensors.driveCentreEncoder, false),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(-125),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-2 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, false)),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-50, sensors.yaw, false, false, false)),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube3ToHR, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
//					new AcStraight(-2.9, -110, sensors.averageEncoder, true),
//					new AcTankDrive(new ChTime(0.2), 0.1, 0.1),
//					new AcTurn(-10),
//					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
//					),
//					new AcStraight(1, -10, sensors.driveCentreEncoder, true)
				);
			//4 cube CC HLHH
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
					//FIXME Should be able to remove this.
					new AcWait(0.25),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-0.6, 80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.6 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraight(-0.6, 0, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(65),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-1.5, 65, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
//							new AcDoNothing(new ChNumSen(cube4ToHR3.totalDistance - 0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcDoNothing(new ChNumSen(-70, sensors.yaw, false, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube4ToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//3 cube FF HLH
			case(101):
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraight(4.7, 0, sensors.driveCentreEncoder, true),
					new AcTurnLenient(90),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraight(-4.75, 90, sensors.driveCentreEncoder, true),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(25),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(1, 10, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-5),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-0.7, -5, sensors.driveCentreEncoder, true),
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
					new AcTurn(-80),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(0.3, -80, sensors.driveCentreEncoder, true),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcStraight(-0.6, -80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube2ToHL.totalDistance - 0.6 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube2ToHL, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			case(202):
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2.5 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCube)
					),
					new AcStraight(3, -15, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcStraight(5.5, 5, sensors.driveCentreEncoder, false),
					new AcTurn(40),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcStraight(-0.7, 40, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.6, -0.6),
					new AcSetArm(0),
					new AcInterrupt.KillSubsystem(intake),
					new AcTurn(130),
					new AcStraight(-0.7, 130, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(intakeCube),
					new AcStraight(-4, 90, sensors.driveCentreEncoder, false),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(25),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraight(1, 10, sensors.driveCentreEncoder, true)
				);
			case(200):
				return createAuto(
					new AcResetSensors(),
					new AcEfficientTurn(90),
					new AcEfficientTurn(0),
					new AcEfficientTurn(45),
					new AcEfficientTurn(-90)
				);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
