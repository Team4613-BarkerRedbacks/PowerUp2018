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
	public static enum AutoComponent {
		 L_RLLL_43;
	}
	public static CommandBase getAutoComponent(AutoComponent autoComponent) {
		switch(autoComponent) {
		case L_RLLL_43:
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
					));
			default: return null;
		}
	}
}
