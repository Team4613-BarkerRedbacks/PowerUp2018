package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import static redbacks.arachne.lib.override.MotionSettings2.encoderTicksPerMetre;

import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.actions.AcIntakeRightSide;
import redbacks.robot.actions.AcResetSensors;
import redbacks.robot.actions.AcSetArm;
import redbacks.robot.actions.AcTankDrive;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;
import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart
{
	public static SenCANEncoder.Displacement autoDistanceEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			case(2):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHR, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.5),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.5))
				);
			case(12):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							//FIXME Fire pneumatic
//							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.55 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
							new AcSeq.Parallel(new AcMotor.Set(intake.intakeMotor, -1, new ChTime(2)))
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
					new AcSetArm(-armBasePos),
					new AcTankDrive(new ChNumSen(3, sensors.yaw, true, false, false), 0.8, -0.8),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCube),
					new AcPath(new ChTime(1.2), true, HRToCube6, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
					new AcInterrupt.KillSubsystem(intake),
					new AcSetArm(-armBasePos),
					new AcTankDrive(new ChNumSen(-0.75 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
					new AcTankDrive(new ChNumSen(20, sensors.yaw, true, false, false), 0.8, -0.8),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake),
							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(1)))
					),
					new AcPath(new ChTime(1.25), true, HRToCube5, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					new AcTankDrive(new ChTime(0.3), -0.6, -0.6),
					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(0),
					new AcTankDrive(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
					new AcTankDrive(new ChNumSen(-25, sensors.yaw, false, false, false), -0.8, 0.8),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.5 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
							new AcSeq.Parallel(new AcMotor.Set(intake.intakeMotor, -1, new ChTime(1)))
					),
					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.7, 0.7)
				);
			case(13):
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							//FIXME Fire pneumatic
//							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.55 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
//							new AcSeq.Parallel(highFireRelease)
							new AcSeq.Parallel(new AcMotor.Set(intake.intakeMotor, -1, new ChTime(2)))
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
					new AcTankDrive(new ChNumSen(3, sensors.yaw, true, false, false), 0.8, -0.8),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCube),
					new AcPath(new ChTime(1.25), true, HRToCube6, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSetArm(-armBasePos),
					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
					new AcTankDrive(new ChNumSen(30, sensors.yaw, true, false, false), 0.8, -0.8),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake),
							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(1)))
					),
					new AcTankDrive(new ChMulti(
							LogicOperators.OR,
							new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder),
							new ChTime(1)
					), -0.7, -0.7),
					new AcTankDrive(new ChTime(0.3), 0.6, 0.6),
//					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(0),
					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
					new AcTankDrive(new ChNumSen(-10, sensors.yaw, false, false, false), -0.8, 0.8),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
							new AcSeq.Parallel(new AcMotor.Set(intake.intakeMotor, -1, new ChTime(1)))
					),
					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8),
					//4th cube
					new AcTankDrive(new ChNumSen(-1 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.7, -0.7),
					new AcTankDrive(new ChNumSen(40, sensors.yaw, true, false, false), 0.8, -0.8),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcSeq.Parallel(new AcIntakeRightSide(new ChTime(0.5))),
							new AcSeq.Parallel(intakeCube),
//							new AcSeq.Parallel(highFirePrime),
							new AcSetArm(0)
					),
					new AcTankDrive(new ChMulti(
							LogicOperators.OR,
							new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder),
							new ChTime(1.5)
					), -0.7, -0.7),
					new AcTankDrive(new ChNumSen(-1.2 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false), 0.7, 0.7),
					new AcTankDrive(new ChNumSen(-10, sensors.yaw, false, false, false), -0.8, 0.8),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.35 * encoderTicksPerMetre, autoDistanceEncoder, true, false, false)),
//							new AcSeq.Parallel(highFireRelease)
							new AcSeq.Parallel(new AcMotor.Set(intake.intakeMotor, -1, new ChTime(1)))
					),
					new AcTankDrive(new ChNumSen(0, autoDistanceEncoder, true, false, false), 0.8, 0.8)
				);
			case(3):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL2.totalDistance - 2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(quickFire),
							new AcSetArm(75)
					),
					new AcPath(new ChFalse(), true, wallToHL2, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
				);
			case(4):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 1.25 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHL3, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
				);
			case(5):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL5.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcSetArm(25),
							new AcDoNothing(new ChNumSen(wallToHL5.totalDistance - 0.9 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHL5, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHL5, sensors.driveCentreEncoder, (int) (1.25 * encoderTicksPerMetre), -0.6),
							new AcPath.ChangeMinMax(wallToHL5, sensors.driveCentreEncoder, (int) (1.25 * encoderTicksPerMetre), 0.6))
				);
			case(6):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL6.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcSetArm(25),
							new AcDoNothing(new ChNumSen(wallToHL6.totalDistance - 1.3 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHL6, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
				);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
