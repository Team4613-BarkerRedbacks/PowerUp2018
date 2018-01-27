package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

public class SubsystemIntake extends SubsystemBase {
	
	// Question: Should we map these to a Joystick?
	public CtrlMotor intakeMotor = new CtrlMotor(idMotIntakeL);
	
	public SolSingle intakeLeftSol = new SolSingle(idSolLeftIntake);
	public SolSingle intakeRightSol = new SolSingle(idSolRightIntake);
	
	public SubsystemIntake() {
		super();
		idMotIntakeR.follow(idMotIntakeL);
	}
}
