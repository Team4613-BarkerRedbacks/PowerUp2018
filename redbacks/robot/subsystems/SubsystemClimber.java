package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.IMotorController;

public class SubsystemClimber extends SubsystemBase
{
	public CtrlMotor climberMotor = new CtrlMotor(idMotClimb1);
	public SolSingle climberSol = new SolSingle(idSolClimbRelease);

	public SubsystemClimber(SubsystemBase... childSystems) {
		super(childSystems);

		idMotClimb2.follow((IMotorController) climberMotor.controller);
	}
}
