package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;

/**
 * @author Sean Zammit
 */
public class AcDriveDirection extends Action
{
	double sp, angle;
	
	@Deprecated
	public AcDriveDirection(Check check, double speed, double angle) {
		super(check);
		sp = speed;
		this.angle = angle;
	}
	
	public void onStart() {
		Robot.isIndivDriveControl = false;
	}

	public void onRun() {
		Robot.driver.drivetrain.tankDrive(
				sp - (Robot.sensors.yaw.get() - angle) * MotionSettings2.drivePIDGyrokP,
				sp + (Robot.sensors.yaw.get() - angle) * MotionSettings2.drivePIDGyrokP
		);
	}
}