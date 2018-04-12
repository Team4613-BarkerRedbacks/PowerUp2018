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
			default: return null;
		}
	}
}
