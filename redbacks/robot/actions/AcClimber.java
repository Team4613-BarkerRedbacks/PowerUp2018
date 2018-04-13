package redbacks.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		double val = OI.axis_c_RT.get() - OI.axis_c_LT.get();
		
		Robot.climber.climberMotor.set(Math.abs(val) > 0.1 ? val : 0, command);
		SmartDashboard.putNumber("Climber", Robot.climber.climberMotor.get());
	}
}
