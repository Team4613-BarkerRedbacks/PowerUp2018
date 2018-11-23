package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolDouble;

import static redbacks.robot.RobotMap.*;

public class SubsystemClimber extends SubsystemBase
{
	public CtrlMotor climberMotor = new CtrlMotor(idMotClimb);
	public SolDouble climberSol = new SolDouble(idSolClimbRelease1, idSolClimbRelease2);
}
