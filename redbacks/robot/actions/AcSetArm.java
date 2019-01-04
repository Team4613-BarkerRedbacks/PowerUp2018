package redbacks.robot.actions;

import edu.wpi.first.wpilibj.PIDController;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.robot.Robot;

/**
 * Changes the setpoint of the {@link PIDController} controlling the arm.
 * Do not run this action on the arm subsystem - you'll interrupt the actual command to control the arm.
 *
 * @author Sean Zammit
 */
public class AcSetArm extends Action
{
	public int setpoint;
	
	public AcSetArm(int setpoint) {
		super(new ChTrue());
		this.setpoint = setpoint;
	}
	
	public void onFinish() {
		Robot.arm.armPIDControl.setSetpoint(setpoint);
	}
}
