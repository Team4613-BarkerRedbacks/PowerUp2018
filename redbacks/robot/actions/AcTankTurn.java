package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;

/**
 * A basic bang-bang turn to a specified heading.
 * 
 * @author Sean Zammit
 */
public class AcTankTurn extends Action
{
	double angleTarget, speed;
	
	public AcTankTurn(double angleTarget, double speed) {
		super(new ChFalse());
		
		this.angleTarget = angleTarget;
		this.speed = speed;
	}
	
	public AcTankTurn(double angleTarget) {
		this(angleTarget, 1);
	}
	
	public void onStart() {
		ArachneRobot.isIndivDriveControl = false;
	}
	
	public void onRun() {
		Robot.driver.drivetrain.tankDrive((angleTarget > Robot.sensors.yaw.get() ? 1 : -1) * speed, (angleTarget > Robot.sensors.yaw.get() ? -1 : 1) * speed);
	}
	
	public boolean isDone() {
		return Robot.sensors.yaw.get() < angleTarget + 3 && Robot.sensors.yaw.get() > angleTarget - 3;
	}
}