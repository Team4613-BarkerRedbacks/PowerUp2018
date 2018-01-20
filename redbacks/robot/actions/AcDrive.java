package redbacks.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.OI;
import redbacks.robot.Robot;

public class AcDrive extends Action {

	double minR = 0.6D, difR = 0.15D;
	
	public AcDrive() {
		super(new ChFalse());
	}
	
	public void onRun() {
		//Change drive direction to opposite side
		boolean invertDrive = SmartDashboard.getBoolean("Invert Drive", false);
		SmartDashboard.putBoolean("Invert Drive", invertDrive);
		
		if (!Robot.isIndivDriveControl) {
			arcadeDrive(OI.axis_d_RY.get() * (invertDrive ? 1 : -1), -OI.axis_d_LX.get());
		}
	}
	
	/**
	 * Pseudo-arcade drive method of control. Calculate adjustments to the inputs in this method.
	 * @param sp The parallel speed and direction of the robot.
	 * @param rotation The speed at which the robot should rotate.
	 */
	public void arcadeDrive(double sp, double rotation) {
		double mod = minR + difR * Math.pow(1 - Math.abs(sp), 2);
		double r = Math.pow(rotation, 3) * mod;
		Robot.driver.drivetrain.tankDrive(- sp - r, - sp + r);
}

}
