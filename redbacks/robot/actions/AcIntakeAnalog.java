package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.Robot;

public abstract class AcIntakeAnalog extends Action
{
	public AcIntakeAnalog() {
		super(new ChFalse());
	}
	
	public abstract double getSpeed();
	
	public void onRun() {
		Robot.intake.intakeMotor.set(getSpeed(), command);
	}
}
