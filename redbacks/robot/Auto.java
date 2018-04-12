package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;

import static redbacks.arachne.ext.motion.MotionSettings.encoderTicksPerMetre;
import static redbacks.arachne.lib.override.MotionSettings2.*;

import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.actions.actuators.AcSolenoid;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.robot.actions.*;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DriverStation;

import static redbacks.robot.Auto.createAuto;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart
{
	public static final String LSWITCH_LSCALE = "LLL", LSWITCH_RSCALE = "LRL", RSWITCH_LSCALE = "RLR", RSWITCH_RSCALE = "RRR";
	public static SenCANEncoder.Displacement autoDistanceEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	
	public static CommandBase getAutonomous(int autoNumber) {
		if(autoNumber < 0) {
			return getAutoComponent(AutoComponent.getAutoFromID(-autoNumber));
		}
		
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		switch(autoNumber) {
			case 1:
			default: switch(gameData) {
				case RSWITCH_RSCALE: return getAutoComponent(AutoComponent.R__R_HHH_65);
				case RSWITCH_LSCALE: return getAutoComponent(AutoComponent.R_R__LLLL_654);
				case LSWITCH_RSCALE: return getAutoComponent(AutoComponent.R__R_HHH_65);
				case LSWITCH_LSCALE: return getAutoComponent(AutoComponent.R_L__LLL_34);
			}
			case 2: switch(gameData) {
				case RSWITCH_RSCALE: return getAutoComponent(AutoComponent.R_RR_LHH_65);
				case RSWITCH_LSCALE: return getAutoComponent(AutoComponent.R_R__LLLL_654);
				case LSWITCH_RSCALE: return getAutoComponent(AutoComponent.R__R_HHH_65);
				case LSWITCH_LSCALE: return getAutoComponent(AutoComponent.R_L__LLL_34);
			}
			case 3: switch(gameData) {
				case RSWITCH_RSCALE: return getAutoComponent(AutoComponent.C_R__LLL_SS);
				case RSWITCH_LSCALE: return getAutoComponent(AutoComponent.C_R__LLL_SS);
				case LSWITCH_RSCALE: return getAutoComponent(AutoComponent.C_L__LLL_SS);
				case LSWITCH_LSCALE: return getAutoComponent(AutoComponent.C_L__LLL_SS);
			}
		}
		return createAuto(new AcStraightFinishOnTarget(3, 0, sensors.distanceEncoder, true));
	}
	
	/**
	 * Naming conventions:
	 * 
	 * Prefix/Position		Switch,Scale Pos		Cube shot order		Cubes picked up
	 * L - Left start		L - Left				H - Scale			1 - Far left
	 * C - Center start		R - Right				L - Switch			6 - Far right
	 * R - Right start		_ - Unspecified			E - Exchange		S - Stack
	 * OLD - Untuned								P - Pickup
	 * SAFE - Slow+reliable
	 *
	 * @author Sean Zammit
	 */
	public static enum AutoComponent {
		OLD_CC_HLHH_1(1),
		OLD_CC_HLHH_2(2),
		OLD__C_HHHH(3),
		OLD_FC_HLH(4),
		OLD_CF_LHH_1(5),
		OLD_FF_HLH(6),
		OLD_FF_HL(7),
		OLD__F_HHH(8),
		OLD_COOP_CC_HLH(9),
		OLD_COOP_CF_LHP(10),
		OLD_C__L(11),
		OLD__C_H(12),
		OLD_CC_LHH(13),
		OLD__F_HH(14),
		OLD_FC_HL(15),
		OLD_CF_LHH_2(16),
		OLD__C_HHH(17),
		
		R__R_HHH_65(51),	//Done
		R_RR_LHH_65(52),
		R_R__LLLL_654(53),	//Done
		R__L_HH_2(54),
		R_L__LLL_34(55),	//Done
		
		C_R__LLL_SS(61),	//Done
		C_L__LLL_SS(62),
		
		L__L_HHH_12(71),	//Done
//		R_RR_LHH_65(52),
		L_L__LLLL_123(73),
//		R__L_HHH_23(54),
//		R_L__LLL_34(55),
		
		TUNE(100),
		TEST(101),
		
		SAFE__F_H(200),
		SAFE__C_H(201),
		SAFE_R__R_HHH_65(202),
		
		PROGRESS_0(300),
		PROGRESS_1(301),
		PROGRESS_2(302),
		PROGRESS_3(303),
		PROGRESS_4(304)
		;
		
		private int id;
		
		private AutoComponent(int id) {
			this.id = id;
		}
		
		public static AutoComponent getAutoFromID(int id) {
			for(AutoComponent auto : values()) if(auto.id == id) return auto;
			return null;
		}
	}
	
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
			/**
			 * Tuning auto
			 * DO NOT CHANGE THIS CODE!
			 * It must stay the same for mechanical tuning.
			 * 
			 * Mechanical Setup:
			 * 1. Mark down a distance of 5.57m from the front bumpers of the robot.
			 * 2. Mark the location of the back of the robot, and the angle.
			 */
			case TUNE:
				return createAuto(
						new AcResetSensors(),
						new AcStraightFinishOnTarget(5.57, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -0.4),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), 0.4))
				);
			case TEST:
				return createAuto(
						new AcResetSensors(),
						new AcStraightFinishOnTarget(5.57, 0, sensors.distanceEncoder, true,
						new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (4 * encoderTicksPerMetre), -0.6),
						new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (4 * encoderTicksPerMetre), 0.6))
				);
			//1 cube far scale
			//DO NOT CHANGE THIS CODE!
			case SAFE__F_H:
				return createAuto(
						new AcResetSensors(),
						new AcStraightFinishOnTarget(5.57, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -driveSlowVoltage),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), driveSlowVoltage)),
						new AcTurn(90),
						new AcStraightFinishOnTarget(-5.3, 90, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -driveSlowVoltage),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), driveSlowVoltage)),
						new AcSeq.Parallel(highFirePrime),
						new AcTurn(10),
						new AcSeq.Parallel(
								new AcSetArm(100),
								new AcDoNothing(new ChNumSen(0.8 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightFinishOnTarget(1.25, 10, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -driveSlowVoltage - 0.1),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, driveSlowVoltage + 0.1))
				);
			//1 cube close scale
			//DO NOT CHANGE THIS CODE!
			case SAFE__C_H:
				return createAuto(
						new AcResetSensors(),
						new AcStraightFinishOnTarget(5.57, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -driveSlowVoltage),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), driveSlowVoltage)),
						new AcSeq.Parallel(highFirePrime),
						new AcTurn(-30),
						new AcSeq.Parallel(
								new AcSetArm(100),
								new AcDoNothing(new ChNumSen(0.9 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, false)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraightFinishOnTarget(1.25, -30, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, -driveSlowVoltage - 0.1),
								new AcStraight.ChangeMinMax(sensors.distanceEncoder, 0, driveSlowVoltage + 0.1))
				);
			case SAFE_R__R_HHH_65:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
								new AcSeq.Parallel(highFirePrime),
								new AcDoNothing(new ChNumSen(6.3 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraight(6.9, -8.25, sensors.distanceEncoder, true),
						//2nd cube
						new AcSetArm(-armBasePos),
						new AcTankTurn(11),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightLenient(-1.8, 11, sensors.distanceEncoder, true),
						new AcTankDrive(new ChTime(0.4), -0.5, -0.5),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcTankTurn(23),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraight(2.0, 23, sensors.distanceEncoder, true),
						new AcInterrupt.KillSubsystem(intake),
						new AcTankTurn(-70),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease),
						//3rd cube
						new AcWait(0.25),
						new AcSetArm(armBasePos),
						new AcTankTurn(-145),
						new AcSeq.Parallel(intakeCubeFast),
						new AcStraightLenient(2.7, -145, sensors.distanceEncoder, true),
						new AcTankDrive(new ChTime(0.25), 0.5, 0.5),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcStraight(-2.5, -147, sensors.distanceEncoder, true),
						new AcInterrupt.KillSubsystem(intake),
						new AcTankTurn(-90),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease)					
				);
			case R_R__LLLL_654:
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
						new AcSetArm(-armSwitchPos),
						new AcSeq.Parallel(
								new AcWait(0.25),
								new AcInterrupt.KillSubsystem(intake)
						),
						new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
						new AcDoNothing(new ChNumSen(-armSwitchPos - 100, sensors.armEncoder, true, false, false)),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), -intakeFastSpeed, -intakeSlowSpeed)),
						//3rd cube
						new AcWait(0.25),
						new AcTankTurn(75),
						new AcSetArm(-armBasePos),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSpeed, intakeSlowSpeed - 0.1)),
						new AcSetNumSen(autoDistanceEncoder, 0),
						new AcDriveDirection(new ChNumSen(-0.8 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.55, 80),
						new AcSeq.Parallel(intakeCube),
						new AcSetArm(armSlightPosTele),
						new AcDoNothing(new ChNumSen(0, sensors.armEncoder, true, false, false)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSeq.Parallel(solExtendIntakeR),
						new AcSeq.Parallel(new AcTankDrive(new ChTime(0.5), 0.5, 0.5)),
						new AcWait(0.7),
						new AcSeq.Parallel(solRetractIntakeR),
						//4th cube
						new AcSetArm(-armBasePos),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSpeed, intakeSlowSpeed - 0.1)),
						new AcSeq.Parallel(sequencer, new AcDriveDirection(new ChNumSen(-1.6 * encoderTicksPerMetre, autoDistanceEncoder, false, false, false), -0.55, 80)),
						new AcDoNothing(new ChTime(2)),
						new AcInterrupt.KillSubsystem(sequencer),
						new AcSeq.Parallel(intakeCube),
						new AcSetArm(armScalePos),
						new AcSeq.Parallel(new AcTankDrive(new ChTime(0.5), 0.5, 0.5)),
						new AcDoNothing(new ChNumSen(0, sensors.armEncoder, true, false, false)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSeq.Parallel(solExtendIntakeR),
						new AcWait(0.7),
						new AcSeq.Parallel(solRetractIntakeR)
				);
			case R__R_HHH_65:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
								new AcSeq.Parallel(highFirePrime),
								new AcDoNothing(new ChNumSen(6.3 * encoderTicksPerMetre, sensors.distanceEncoder, true, true, false)),
//								//TODO Check the height on this shot
//								new AcSeq.Parallel(newFireCommand(Robot.intake, new AcSplitIntakeControl(new ChTime(0.5), 0.8, 1)))
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraight(6.9, -8.25, sensors.distanceEncoder, true),
						//2nd cube
						new AcSetArm(-armBasePos),
						new AcTankTurn(11),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(-1 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true)),
								new AcSeq.Parallel(intakeCube)
						),
						new AcStraightLenient(-1.8, 11, sensors.distanceEncoder, true),
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
						new AcTankTurn(25),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSlowSpeed)),
						new AcStraightLenient(-2, 25, sensors.averageEncoder, true),
						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcSetArm(-armScalePos),
						new AcSeq.Parallel(intakeCube),
						new AcStraight(2.5, 33, sensors.distanceEncoder, true),
						new AcInterrupt.KillSubsystem(intake),
						new AcTankTurn(90),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease)
				);
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
						new AcSeq.Parallel(highFirePrime),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcSetArm(armSlightPosTele),
						new AcTurn(-5),
						new AcInterrupt.KillSubsystem(intake),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(1.1 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcStraight(1.4, -5, sensors.distanceEncoder, true),
						//3rd cube
						new AcWait(0.25),
						new AcSetArm(-armBasePos),
						new AcTankTurn(25),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSlowSpeed)),
						new AcStraightLenient(-2, 25, sensors.averageEncoder, true),
						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.5 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcSetArm(-armScalePos),
						new AcSeq.Parallel(intakeCube),
						new AcStraight(2.5, 33, sensors.distanceEncoder, true),
						new AcInterrupt.KillSubsystem(intake),
						new AcTankTurn(90),
						new AcWait(0.25),
						new AcSeq.Parallel(highFireRelease)
				);
			case R_L__LLL_34:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraight(5.45, 0, sensors.distanceEncoder, true),
						new AcTurn(-90),
						new AcStraight(2.45, -90, sensors.distanceEncoder, true),
						new AcTurn(-135),
						new AcSetArm(armSwitchPos),
						new AcSeq.Parallel(sequencer,
								new AcStraight(1.2, -135, sensors.distanceEncoder, true)
						),
						new AcDoNothing(new ChNumSen(1.5, new SenTimer())),
						new AcInterrupt.KillSubsystem(sequencer),
						new AcSeq.Parallel(outtakeCubeFast),
						//2nd cube
						new AcWait(0.25),
						new AcInterrupt.KillSubsystem(intake),
						new AcDriveDirection(new ChTime(0.75), -0.6, -135),
						new AcSetArm(armBasePos),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSlowSpeed, intakeFastSpeed)),
						new AcDriveDirection(new ChTime(1.25), 0.6, -145),
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
						new AcDriveDirection(new ChNumSen(-0.7 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 10),
						new AcSetArm(armBasePos),
						new AcTankTurn(-35),
						new AcSeq.Parallel(intakeCube),
						new AcDriveDirection(new ChTime(1), 0.6, -35),
						new AcSetArm(armSwitchPos),
						new AcSeq.Parallel(highFirePrime),
						new AcWait(0.5),
						new AcTankTurn(0),
						new AcDriveDirection(new ChTime(0.5), 0.5, 0),
						new AcSeq.Parallel(highFireRelease)
				);
			case R__L_HH_2:
				return createAuto(
						new AcResetSensors(),
						//1st cube
						new AcStraight(5.3, 0, sensors.distanceEncoder, true,
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -1),
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), 1)),
						new AcTankTurn(-90),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(3 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFirePrime)
						),
						new AcStraight(5.4, -90, sensors.averageEncoder, true,
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), -1),
								new AcStraight.ChangeMinMaxNeg(sensors.distanceEncoder, (int) (3 * encoderTicksPerMetre), 1)),
						new AcInterrupt.KillSubsystem(intake),
						new AcSetArm(armSlightPos),
						new AcTankTurn(5),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.35 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcDriveDirection(new ChNumSen(0.6 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.6, 5),
						//2nd cube
						new AcWait(0.25),
						new AcSetArm(-armBasePos),
						new AcTankTurn(-18),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeSpeed, intakeFastSpeed)),
						new AcStraightLenient(-2, -18, sensors.averageEncoder, true),
						new AcTankDrive(new ChTime(0.25), -0.5, -0.5),
						new AcTankTurn(-45),
						new AcSeq.Parallel(sequencer,
								new AcSeq.Parallel(intakeCube),
								new AcDoNothing(new ChNumSen(armScalePos, sensors.armEncoder, false, false, false)),
								new AcSeq.Parallel(outtakeCubeSlow),
								new AcWait(0.25),
								new AcSeq.Parallel(intakeCube)
						),
						new AcSeq.Parallel(highFirePrime),
						new AcSetArm(armSlightPos),
						new AcDriveDirection(new ChNumSen(0.7 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.65, -45),
						new AcTankTurn(5),
						new AcInterrupt.KillSubsystem(intake),
						new AcInterrupt.KillSubsystem(sequencer),
						new AcWait(0.25),
						new AcSeq.Parallel(
								new AcDoNothing(new ChNumSen(0.05 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true)),
								new AcSeq.Parallel(highFireRelease)
						),
						new AcDriveDirection(new ChNumSen(0.4 * encoderTicksPerMetre, sensors.averageEncoder, true, false, true), 0.6, 10)
				);
			case C_R__LLL_SS:
				SenTimer timer = new SenTimer();
				
				return createAuto(
						new AcResetSensors(),
						new AcSetNumSen(timer, 0),
						//1st cube
						new AcSetArm(armScalePos + 100),
						new AcStraight(2.6, 0, sensors.distanceEncoder, true),
						new AcSeq.Parallel(outtakeCube),
						new AcTankDrive(new ChTime(0.5), 0.5, 0.5),
						new AcInterrupt.KillSubsystem(intake),
						//2nd cube
						new AcDriveDirection(new ChNumSen(-0.4 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 0),
						new AcSetArm(armBasePos),
						new AcTankTurn(-80),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSlowSpeed)),
						new AcDriveDirection(new ChNumSen(0.9 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true), 0.55, -80),
						new AcWait(0.25),
						new AcSeq.Parallel(intakeCubeFast),
						new AcDriveDirection(new ChNumSen(0.2 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, false), -0.6, -80),
						new AcInterrupt.KillSubsystem(intake),
						new AcSetArm(armScalePos + 100),
						new AcTankTurn(-10),
						new AcDriveDirection(new ChTime(1), 0.6, -10),
						new AcSeq.Parallel(outtakeCube),
						new AcWait(0.25),
						new AcInterrupt.KillSubsystem(intake),
						//3nd cube
						new AcDriveDirection(new ChNumSen(-0.4 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, true), -0.6, 0),
						new AcSetArm(armBasePos),
						new AcTankTurn(-65),
						new AcSeq.Parallel(intake, new AcSplitIntakeControl(new ChFalse(), intakeFastSpeed, intakeSpeed)),
						new AcDriveDirection(new ChNumSen(0.85 * encoderTicksPerMetre, sensors.distanceEncoder, true, false, true), 0.6, -65),
						new AcWait(0.5),
						new AcSeq.Parallel(intakeCubeSlow),
						new AcSeq.Parallel(
								new AcWait(0.25),
								new AcSetArm(0),
								new AcWait(0.25),
								new AcInterrupt.KillSubsystem(intake)
						),
						new AcDriveDirection(new ChMulti(LogicOperators.OR,
								new ChNumSen(0.3 * encoderTicksPerMetre, sensors.distanceEncoder, false, false, false),
								new ChNumSen(14.5, timer, true, false, false)
						), -0.6, -110),
						new AcSeq.Parallel(sideKickR),
						new AcWait(0.7),
						new AcSeq.Parallel(solRetractIntakeR)
				);
			default: return Auto2.getAutoComponent(autoComponent);
//			default: return OldAuto.getAutoComponent(autoComponent);
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
