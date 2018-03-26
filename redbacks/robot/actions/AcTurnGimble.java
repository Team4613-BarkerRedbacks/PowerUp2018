package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.logic.ListOperators;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * @author Sean Zammit
 */
public class AcTurnGimble extends AcPath
{
	double lSpeed, rSpeed;
	
	public AcTurnGimble(Check check, double lSpeed, double rSpeed) {
		super();
		lSpeed = 0.1;
		rSpeed = 0.8;
	}
	
	public void onRun() {
		acRotation.execute();
		
		drivetrain.tankDrive(lSpeed, rSpeed);
	}
}
