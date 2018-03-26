package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import static redbacks.arachne.lib.override.MotionSettings2.encoderTicksPerMetre;

import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.SwitchAction.OptionAction;
import redbacks.arachne.lib.actions.actuators.AcSolenoid;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.actions.*;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DriverStation;

import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart
{
	public static SenCANEncoder.Displacement autoDistanceEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	
	public static CommandBase getAutonomous(int autoNumber) {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		switch(autoNumber) {
			case 1: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent.CC_HLHH_1);
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);
				case "LRL": return getAutoComponent(AutoComponent.FC_HLH);
				case "LLL": return getAutoComponent(AutoComponent.FF_HLH);
				default: return null;
			}
			case 2: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent._C_HHHH);
				case "RLR": return getAutoComponent(AutoComponent._F_HHH);
				case "LRL": return getAutoComponent(AutoComponent._C_HHHH);
				case "LLL": return getAutoComponent(AutoComponent._F_HHH);
				default: return null;
			}
			case 3: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent.CC_HLHH_1);
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);
				case "LRL": return getAutoComponent(AutoComponent._C_HHHH);
				case "LLL": return getAutoComponent(AutoComponent.FF_HLH);
				default: return null;
			}
			case 4: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent.C__L);//CC Side kick then stop
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);//CF
				case "LRL": return getAutoComponent(AutoComponent._C_HHHH);//FC
				case "LLL": return getAutoComponent(AutoComponent.FF_HLH);//FF
				default: return null;
			}
			case 5: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent.CC_LHH);
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);
				case "LRL": return getAutoComponent(AutoComponent.FC_HL);
				case "LLL": return getAutoComponent(AutoComponent.FF_HL);
				default: return null;
			}
			case 6:
			switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent._C_HHH);
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);
				case "LRL": return getAutoComponent(AutoComponent._C_HHH);
				case "LLL": return getAutoComponent(AutoComponent.FF_HL);
				default: return null;
			}
			case 7:
			default: switch(gameData) {
				case "RRR": return getAutoComponent(AutoComponent._C_HHH);
				case "RLR": return getAutoComponent(AutoComponent.CF_LHH_2);
				case "LRL": return getAutoComponent(AutoComponent._C_HHH);
				case "LLL": return getAutoComponent(AutoComponent._F_HH);
				default: return null;
			}
		}
	}
	
	public static enum AutoComponent {
		CC_HLHH_1,
		CC_HLHH_2,
		_C_HHHH,
		FC_HLH,
		CF_LHH_1,
		FF_HLH,
		FF_HL,
		_F_HHH,
		COOP_CC_HLH,
		COOP_CF_LHP,
		C__L,
		_C_H,
		CC_LHH,
		_F_HH,
		FC_HL,
		CF_LHH_2,
		_C_HHH,
	}
	
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
			//4 cube CC HLHH
			case CC_HLHH_1:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.45 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(9),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.5, 9, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.3), -0.55, -0.55),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCube),
					new AcStraightSafe(-0.45, 80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
//					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightSafe(0.35, 80, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightSafe(-0.5, 0, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(65),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.4, 60, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
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
									new OptionAction(new ChNumSen(cube4ToHR3.totalDistance - 0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false), new AcSeq.Parallel(highFireRelease))
							)
					),
					new AcPath(new ChFalse(), true, cube4ToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//3 cube CF LHH
			case CF_LHH_1:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightSafe(3, -11, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcStraightSafe(4.85, 5, sensors.driveCentreEncoder, false),
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
									new AcStraightSafe(-0.7, 40, sensors.driveCentreEncoder, true),
									new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
									new AcWait(0.25),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcTurn(130),
									new AcStraightSafe(-0.7, 130, sensors.averageEncoder, true),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
											new AcSeq.Parallel(highFirePrime)
									),
									new AcStraightSafeSafe(-4.75, 90, sensors.averageEncoder, false,
											new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
											new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
									new AcInterrupt.KillSubsystem(intake),
									new AcTurn(12),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
											new AcSeq.Parallel(highFireRelease)
									),
									new AcStraightSafe(1.05, 12, sensors.driveCentreEncoder, true),
									//3rd cube
									new AcSetArm(-armBasePos),
									new AcTurn(-2),
									new AcSeq.Parallel(intakeCubeFast),
									new AcStraightSafe(-1, -2, sensors.driveCentreEncoder, true),
									new AcTankDrive(new ChTime(1), -0.4, -0.4),
									new AcSeq.Parallel(highFirePrime),
									new AcSetArm(0),
									new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
									new AcInterrupt.KillSubsystem(intake),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-0.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
											new AcSeq.Parallel(highFireRelease)
									),
									new AcStraightSafe(0, -2, sensors.driveCentreEncoder, false)),
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
											new AcStraightSafe(-0.7, 130, sensors.averageEncoder, true),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
													new AcSeq.Parallel(highFirePrime)
											),
											new AcStraightSafeSafe(-4.75, 90, sensors.averageEncoder, false,
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
											new AcInterrupt.KillSubsystem(intake),
											new AcTurn(12),
											new AcSeq.Parallel(
													new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
													new AcSeq.Parallel(highFireRelease)
											),
											new AcStraightSafe(1.05, 12, sensors.driveCentreEncoder, true)
									)
							)
					)
				);
			//3 cube FC HLH
			case FC_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.45 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcStraightSafePrecise(new ChNumSen(1.2, new SenTimer()), wallToHR2.totalDistance / encoderTicksPerMetre - 0.7, 0, sensors.driveCentreEncoder, false,
							new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, -0.7),
							new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, 0.7)),
					new AcTurn(90),
					new AcSetArm(-armSwitchPos),
					new AcStraightSafe(-2.45, 90, sensors.averageEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
					new AcTankDrive(new ChTime(0.1), 0.1, 0.1),
					new AcSetArm(-armBasePos),
					new AcTurn(30),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-0.5, 30, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcWait(0.25),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcTurn(-75),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(sequencer, new AcStraightSafe(0.3, -70, sensors.driveCentreEncoder, true)),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcInterrupt.KillSubsystem(sequencer),
					new AcSeq.Parallel(intakeCube),
					new AcStraightSafe(-0.7, -70, sensors.driveCentreEncoder, false),
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
											new AcStraightSafeSafe(-4.8, -120, sensors.averageEncoder, true,
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.2 * encoderTicksPerMetre), -0.4),
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.2 * encoderTicksPerMetre), 0.4)),
											new AcSetArm(armScalePos),
											new AcTurn(-90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//3 cube FF HLH
			case FF_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraightSafe(5.1, 0, sensors.driveCentreEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
					new AcTurnLenient(90),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightSafeSafe(-5.15, 90, sensors.averageEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.4)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.45 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightSafe(1, 10, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-7),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.2, -7, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(1), -0.4, -0.4),
					new AcSetArm(-armSwitchPos),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
					new AcTurn(-80),
					new AcSetArm(-armBasePos),
					new AcSeq.Parallel(intakeCubeFast),
					new AcSeq.Parallel(sequencer, new AcStraightSafe(0.3, -80, sensors.driveCentreEncoder, true)),
					new AcDoNothing(new ChNumSen(-armBasePos + 50, sensors.armEncoder, false, false, false)),
					new AcInterrupt.KillSubsystem(sequencer),
					new AcStraightSafe(-0.7, -80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(
							new AcWait(0.5),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube2ToHL.totalDistance - 0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube2ToHL, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//4 cube CC HLHH
			case CC_HLHH_2:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR2.totalDistance - 0.42 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR2.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR2, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.4),
							new AcPath.ChangeMinMax(wallToHR2, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.4)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(8),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.3, 8, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.4, -0.4),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightSafe(-0.45, 80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
//					new AcSeq.Parallel(intakeCubeSpin),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightSafe(0.35, 80, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightSafe(-0.5, 0, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(70),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.4, 65, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
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
											new AcStraightSafeSafe(3.6, 52, sensors.averageEncoder, true,
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), -0.4),
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), 0.4)),
											new AcSetArm(-armScalePos),
											new AcTurn(90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//4 cube _C HHHH
			case _C_HHHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.45 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(10),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.5, 10, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.3), -0.55, -0.55),
					new AcSeq.Parallel(highFirePrime),
					new AcSetArm(0),
					new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightSafe(0, 5, sensors.driveCentreEncoder, false),
					//3rd cube
					new AcSetArm(-armBasePos),
					new AcTurn(20),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.2, 20, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcTurn(80),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightSafe(0.35, 80, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube5ToHR2.totalDistance - 0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube5ToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre)),
					//4th cube
					new AcSetNumSen(sensors.yaw, 0),
					new AcStraightSafe(-0.5, 0, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(63),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.4, 58, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
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
											new AcStraightSafeSafe(3.6, 50, sensors.averageEncoder, true,
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), -0.4),
													new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (2.4 * encoderTicksPerMetre), 0.4)),
											new AcSetArm(-armScalePos),
											new AcTurn(90),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//3 cube _F HHH
			case _F_HHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcStraightSafe(5, 0, sensors.driveCentreEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
					new AcTurnLenient(90),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightSafeSafe(-5.15, 90, sensors.averageEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.4)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(highFirePrime),
					new AcTurn(10),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightSafe(1, 10, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-5),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1, -5, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.7), -0.4, -0.4),
					new AcSeq.Parallel(highFirePrime),
					new AcSeq.Parallel(intakeCube),
					new AcSetArm(0),
					new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
					new AcInterrupt.KillSubsystem(intake),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightSafe(0, -5, sensors.driveCentreEncoder, false),
					//3rd cube
					new AcSetArm(-armBasePos),
					new AcTurn(-20),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.2, -20, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcWait(0.2),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcTurnPrecise(-75),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightSafe(0.35, -75, sensors.driveCentreEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(cube2ToHL.totalDistance - 0.8 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, cube2ToHL, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre))
				);
			//Coop CC HLH
			case COOP_CC_HLH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR2.totalDistance - 0.42 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR2.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR2, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR2, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.4),
							new AcPath.ChangeMinMax(wallToHR2, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.4)),
					//2nd cube
					new AcSetArm(-armBasePos),
					new AcTurn(10),
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcStraightSafe(-1.3, 10, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.25), -0.4, -0.4),
					new AcSetArm(-armSwitchPos - 75),
					new AcSeq.Parallel(
							new AcWait(0.25),
							new AcInterrupt.KillSubsystem(intake),
							new AcWait(0.25),
							new AcSeq.Parallel(outtakeCube)
					),
					new AcWait(0.25),
					new AcTankDrive(new ChTime(0.5), -0.4, -0.4),
					new AcInterrupt.KillSubsystem(intake),
					//3rd cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(45, sensors.yaw, true, false, false)),
							new AcSetArm(-armBasePos)
					),
					new AcTurn(80),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcStraightSafe(-0.45, 80, sensors.driveCentreEncoder, true),
					new AcWait(0.3),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCube),
					new AcSeq.Parallel(
							new AcWait(1),
							new AcInterrupt.KillSubsystem(intake)
					),
					new AcStraightSafe(1.5, 90, sensors.driveCentreEncoder, true),
					new AcTurn(0),
					new AcSeq.Parallel(highFirePrime),
					new AcStraightSafe(2, 0, sensors.driveCentreEncoder, true),
					new AcSetArm(armScalePos),
					new AcTurn(-90),
					new AcSeq.Parallel(
							new AcWait(0.3),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcTankDrive(new ChTime(1), 0.5, 0.5)
				);
			//Coop CF LHP
			case COOP_CF_LHP:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightSafe(3, -12, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcStraightSafe(5, 5, sensors.driveCentreEncoder, false),
					new AcTurn(40),
					new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
					new AcStraightSafe(-0.7, 40, sensors.driveCentreEncoder, true),
					new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
					new AcWait(0.25),
					new AcSetArm(0),
					new AcSeq.Parallel(intakeCubeSlow),
					new AcTurn(130),
					new AcStraightSafe(-0.7, 130, sensors.averageEncoder, true),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
							new AcSeq.Parallel(highFirePrime)
					),
					new AcStraightSafeSafe(-4.75, 90, sensors.averageEncoder, false,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
					new AcInterrupt.KillSubsystem(intake),
					new AcTurn(12),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcStraightSafe(1.05, 12, sensors.driveCentreEncoder, true),
					//3rd cube
					new AcSetNumSen(autoDistanceEncoder, 0),
					new AcStraightSafe(0.3, 0, sensors.driveCentreEncoder, false,
							new AcStraightSafe.ChangeMinMax(autoDistanceEncoder, (int) (0.3 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(autoDistanceEncoder, (int) (0.3 * encoderTicksPerMetre), 0.4)),
					new AcTurn(90),
					new AcSetArm(-armSwitchPos),
					new AcStraightSafeSafe(3.2, 90, sensors.averageEncoder, true,
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), -0.4),
							new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), 0.4)),
					new AcSetArm(-armBasePos),
					new AcTurn(0),
					new AcSeq.Parallel(intakeCubeFast),
					new AcTankDrive(new ChTime(1.5), -0.5, -0.5),
					new AcSetArm(0),
					new AcInterrupt.KillSubsystem(intake)
				);
			//1 cube C_ L
			case C__L:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
								new AcSeq.Parallel(solExtendIntakeL),
								new AcWait(1),
								new AcSeq.Parallel(solRetractIntakeL)
						),
						new AcStraightSafe(3, -12, sensors.driveCentreEncoder, true),
						new AcStraightSafe(-0.5, -40, sensors.driveCentreEncoder, true)
				);
			//1 cube _C H
			case _C_H:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.45 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChMulti(
									LogicOperators.AND,
									new ChTime(4.5),
									new ChNumSen(wallToHR3.totalDistance - 0.2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)
							), true, wallToHR3, driver.drivetrain, 1, 1,
							sensors.yaw, sensors.driveCentreEncoder, false,
							new Tolerances.Absolute(0.15 * encoderTicksPerMetre),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), -0.55),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.75 * encoderTicksPerMetre), 0.55))
				);
			case CC_LHH:
				return createAuto(
					new AcResetSensors(),
					//1st cube
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
							new AcSeq.Parallel(solExtendIntakeL),
							new AcWait(1),
							new AcSeq.Parallel(solRetractIntakeL),
							new AcSetArm(-armBasePos),
							new AcSeq.Parallel(intakeCubeSpin)
					),
					new AcStraightSafe(3, -11, sensors.driveCentreEncoder, true),
					//2nd cube
					new AcStraightSafe(4.85, 5, sensors.driveCentreEncoder, false),
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
									new AcStraightSafe(-0.7, 40, sensors.driveCentreEncoder, true,
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, -0.7),
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, 0.7)),
									new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
									new AcWait(0.25),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcTurn(20),
									new AcSeq.Parallel(highFirePrime),
									new AcStraightSafe(1.5, 20, sensors.driveCentreEncoder, true),
									new AcSetArm(-armScalePos),
									new AcTurn(100),
									new AcWait(0.25),
									new AcSeq.Parallel(highFireRelease),
									new AcWait(0.5),
									new AcSetArm(-armBasePos),
									new AcTurn(35),
									new AcSeq.Parallel(
											new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true)),
											new AcSeq.Parallel(intakeCube)
									),
									new AcStraightSafe(-2.2, 35, sensors.driveCentreEncoder, true,
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
									new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
									new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
									new AcSetArm(0),
									new AcSeq.Parallel(intakeCubeSlow),
									new AcSeq.Parallel(highFirePrime),
									new AcStraightSafe(2.2, 35, sensors.driveCentreEncoder, true,
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.7),
											new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.7)),
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
											new AcStraightSafe(1.5, 20, sensors.driveCentreEncoder, true),
											new AcSetArm(-armScalePos),
											new AcTurn(100),
											new AcWait(0.25),
											new AcSeq.Parallel(highFireRelease)
									)
							)
					)
				);
			//2 cube FF HL
			case FF_HL:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraightSafe(4.9, 0, sensors.driveCentreEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
						new AcTurnLenient(90),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcStraightSafeSafe(-5.3, 90, sensors.averageEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.4)),
						new AcInterrupt.KillSubsystem(intake),
						new AcTurn(155),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightSafe(-1.4, 155, sensors.driveCentreEncoder, true),
						new AcTurn(100),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//2nd cube
						new AcWait(0.5),
						new AcSetArm(armBasePos),
						new AcTurn(160),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightSafe(1.6, 160, sensors.driveCentreEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
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
//						new AcStraightSafe(5.1, 0, sensors.driveCentreEncoder, true,
//								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
//								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
//						new AcTurnLenient(90),
//						new AcSeq.Parallel(intakeCubeSlow),
//						new AcStraightSafeSafe(-5.15, 90, sensors.averageEncoder, true,
//								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.4),
//								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.4)),
//						new AcInterrupt.KillSubsystem(intake),
//						new AcSeq.Parallel(highFirePrime),
//						new AcTurn(150),
//						new AcSetArm(armScalePos),
//						new AcStraightSafe(-1.4, 150, sensors.driveCentreEncoder, true),
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
//						new AcStraightSafe(1.6, 160, sensors.driveCentreEncoder, true,
//								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
//								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
//						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
//						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
//						new AcSetArm(0),
//						new AcSeq.Parallel(intakeCubeSlow),
//						new AcSeq.Parallel(highFirePrime),
//						new AcStraightSafe(-1.6, 155, sensors.driveCentreEncoder, true,
//								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
//								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
//						new AcSetArm(armScalePos),
//						new AcTurn(90),
//						new AcWait(0.25),
//						new AcSeq.Parallel(highFireRelease)
//					);
				//2 cube _F HH
				case _F_HH:
					return createAuto(
							new AcResetSensors(),
							//1st cube
							new AcStraightSafe(4.9, 0, sensors.driveCentreEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
							new AcTurnLenient(90),
							new AcSeq.Parallel(intakeCubeSlow),
							new AcWait(2),
							new AcStraightSafeSafe(-5.3, 90, sensors.averageEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (4 * encoderTicksPerMetre), 0.4)),
							new AcInterrupt.KillSubsystem(intake),
							new AcTurn(155),
							new AcSetArm(armScalePos),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, true)),
									new AcSeq.Parallel(highFirePrime)
							),
							new AcStraightSafe(-1.4, 155, sensors.driveCentreEncoder, true),
							new AcTurn(100),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease),
							//2nd cube
							new AcWait(0.5),
							new AcSetArm(armBasePos),
							new AcTurn(160),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
									new AcSeq.Parallel(intakeCube)
							),
							new AcStraightSafe(1.6, 160, sensors.driveCentreEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
							new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
							new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
							new AcSetArm(0),
							new AcSeq.Parallel(intakeCubeSlow),
							new AcSeq.Parallel(highFirePrime),
							new AcStraightSafe(-1.6, 155, sensors.driveCentreEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
							new AcSetArm(armScalePos),
							new AcTurn(90),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease)
						);
			//2 cube FC HL
			case FC_HL:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(6 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightSafe(6.25, 0.5, sensors.driveCentreEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (5 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (5 * encoderTicksPerMetre), 0.4)),
						new AcSetArm(-armScalePos),
						new AcTurn(105),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//2nd cube
						new AcWait(0.5),
						new AcTurn(20),
						new AcStraightSafeSafe(5.55, 20, sensors.driveCentreEncoder, false,
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, -0.45),
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, 0.45)),
						new AcTurn(90),
						new AcStraightSafeSafe(-3.3, 90, sensors.averageEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1.5 * encoderTicksPerMetre), 0.4)),
						new AcTurn(5),
						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
						new AcSetArm(-armBasePos),
						new AcSeq.Parallel(intakeCubeFast),
						new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
						new AcStraightSafe(-0.9, 5, sensors.driveCentreEncoder, true,
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, -0.5),
								new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, 0, 0.5)),
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
			case CF_LHH_2:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1.5 * encoderTicksPerMetre, sensors.driveCentreEncoder)),
								new AcSeq.Parallel(solExtendIntakeL),
								new AcWait(1),
								new AcSeq.Parallel(solRetractIntakeL),
								new AcSetArm(-armBasePos),
								new AcSeq.Parallel(intakeCubeSpin)
						),
						new AcStraightSafe(3, -11, sensors.driveCentreEncoder, true),
						//2nd cube
						new AcStraightSafe(5.1, 5, sensors.driveCentreEncoder, false),
						new AcTurn(33),
						new AcDoNothing(new ChNumSen(-armBasePos + 100, sensors.armEncoder, false, false, false)),
						new AcStraightSafe(-0.7, 33, sensors.driveCentreEncoder, true),
						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
						new AcWait(0.25),
						new AcSetArm(0),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcTurn(130),
						new AcStraightSafe(-0.7, 130, sensors.averageEncoder, true),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-3 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraightSafe(-4.75, 90, sensors.averageEncoder, false,
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), -0.4),
								new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (3.5 * encoderTicksPerMetre), 0.4)),
						new AcInterrupt.KillSubsystem(intake),
						new AcTurn(12),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.475 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightSafe(1.05, 12, sensors.driveCentreEncoder, true),
						//3rd cube
						new AcSetArm(-armBasePos),
						new AcTurn(-4),
						new AcSeq.Parallel(intakeCubeFast),
						new AcStraightSafe(-1, -4, sensors.driveCentreEncoder, true),
						new AcTankDrive(new ChTime(1), -0.4, -0.4),
						new AcSeq.Parallel(highFirePrime),
						new AcSetArm(0),
						new AcDoNothing(new ChNumSen(-25, sensors.armEncoder, true, false, false)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-0.85 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, false)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightSafe(0, -4, sensors.driveCentreEncoder, false)
					);
				//3 cube _C HHH
				case _C_HHH:
					return createAuto(
							new AcResetSensors(),
							//1st cube
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(6 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
									new AcSeq.Parallel(highFirePrime)
							),
							new AcStraightSafe(6.25, 0.5, sensors.driveCentreEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (5 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (5 * encoderTicksPerMetre), 0.4)),
							new AcSetArm(armScalePos),
							new AcTurn(-70),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease),
							//2nd cube
							new AcWait(0.5),
							new AcSetArm(armBasePos),
							new AcTurn(-150),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
									new AcSeq.Parallel(intakeCube)
							),
							new AcStraightSafe(1.8, -150, sensors.driveCentreEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.driveCentreEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
							new AcTankDrive(new ChTime(0.7), 0.5, 0.5),
							new AcSetArm(armScalePos),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.driveCentreEncoder, false, false, false)),
									new AcSeq.Parallel(highFirePrime)
							),
							new AcStraightSafe(0.1, -150, sensors.driveCentreEncoder, false),
							new AcTurn(-80),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease),
							//3rd cube
							new AcWait(0.5),
							new AcSetArm(armBasePos),
							new AcTurn(-140),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(1 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, false, true)),
									new AcSeq.Parallel(intakeCube)
							),
							new AcStraightSafe(2.4, -140, sensors.averageEncoder, true,
									new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), -0.4),
									new AcStraightSafe.ChangeMinMax(sensors.averageEncoder, (int) (1 * encoderTicksPerMetre), 0.4)),
							new AcTankDrive(new ChTime(1), 0.5, 0.5),
							new AcSetArm(armScalePos),
							new AcSeq.Parallel(
									new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.averageEncoder, false, false, false)),
									new AcSeq.Parallel(highFirePrime)
							),
							new AcStraightSafe(0.3, -140, sensors.averageEncoder, false),
							new AcTurn(-80),
							new AcWait(0.25),
							new AcSeq.Parallel(highFireRelease)
					);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
