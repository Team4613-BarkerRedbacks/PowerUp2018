package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

public class SubsystemShooter extends SubsystemBase
{
	public SolSingle shooterSol1 = new SolSingle(idSolShooter1);
	public SolSingle shooterSol2 = new SolSingle(idSolShooter2);
	public SolSingle shooterSolHigh = new SolSingle(idSolShooterHigh);
	public SolSingle shooterLockSol = new SolSingle(idSolLock);

	public SubsystemShooter(SubsystemBase... childSystems) {
		super(childSystems);
	}
}
