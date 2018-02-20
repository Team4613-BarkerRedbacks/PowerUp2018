package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.robot.OI;
import redbacks.robot.Robot;

public class AcClimber extends Action
{
	public AcClimber() {
		super(new ChFalse());
	}

	public void onRun() {
		Robot.climber.climberMotor.set(OI.axis_o_LY.get(), command);
	}
}
