package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;

/**
 * @author Sean Zammit
 */
public class AcTurnOnSpot extends Action
{
	double angleTarget, speed;
	
	public AcTurnOnSpot(double angleTarget, double speed) {
		super(new ChFalse());
		
		this.angleTarget = angleTarget;
		this.speed = speed;
	}
	
	public AcTurnOnSpot(double angleTarget) {
		this(angleTarget, 1);
	}
	
	public void onStart() {
		ArachneRobot.isIndivDriveControl = false;
	}
	
	public void onRun() {
		double comp = (Robot.sensors.driveLeftEncoder.get() + Robot.sensors.driveRightEncoder.get()) / 2000;
		
		Robot.driver.drivetrain.tankDrive(
				(angleTarget > Robot.sensors.yaw.get() ? 1 : -1) * speed - comp,
				(angleTarget > Robot.sensors.yaw.get() ? -1 : 1) * speed - comp
		);
	}
	
	public boolean isDone() {
		return Robot.sensors.yaw.get() < angleTarget + 3 && Robot.sensors.yaw.get() > angleTarget - 3;
	}
}
