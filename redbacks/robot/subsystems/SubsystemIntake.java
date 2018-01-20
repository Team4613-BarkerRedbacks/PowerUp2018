package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import static redbacks.robot.RobotMap.*;

public class SubsystemIntake extends SubsystemBase {
	
	// Question: Should we map these to a Joystick?
	public CtrlMotor intakeLeftMotor = new CtrlMotor(idMotIntakeL);
	public CtrlMotor intakeRightMotor = new CtrlMotor(idMotIntakeR);

	public SubsystemIntake() {
		super();
	}
}
