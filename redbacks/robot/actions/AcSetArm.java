package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.robot.Robot;

public class AcSetArm extends Action
{
	public int setpoint;
	
	public AcSetArm(int setpoint) {
		super(new ChTrue());
		this.setpoint = setpoint;
	}
	
	public void onFinish() {
		Robot.arm.setpoint = setpoint;
	}
}
