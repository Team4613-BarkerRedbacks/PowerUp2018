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
		double val = OI.axis_d_RT.get() - OI.axis_d_LT.get();
		
		Robot.climber.climberMotor.set(Math.abs(val) > 0.1 ? val : 0, command);
	}
}
