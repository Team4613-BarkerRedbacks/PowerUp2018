package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.Check;
import redbacks.robot.Robot;

/**
 * A basic bang-bang drivetrain movement until a specified check condition is met.
 *
 * @author Sean Zammit
 */
public class AcTankDrive extends Action
{
	double l, r;
	
	public AcTankDrive(Check check, double lSpeed, double rSpeed) {
		super(check);
		l = lSpeed;
		r = rSpeed;
	}
	
	public void onStart() {
		Robot.isIndivDriveControl = false;
	}

	public void onRun() {
		Robot.driver.drivetrain.tankDrive(l, r);
	}

	public void onFinish() {
		onRun();
	}
}