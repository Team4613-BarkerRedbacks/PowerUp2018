package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;

public class SubsystemShooter extends SubsystemBase {
	//TODO Change the vaulues to the ports for the solenoid.
	public SolSingle shooterSol = new SolSingle(0); 
	public SolSingle shooterLockSol = new SolSingle(1);

	
	public SubsystemShooter() {
		super();
	}
}
