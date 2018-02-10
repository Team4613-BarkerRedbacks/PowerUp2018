package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

public class SubsystemShooter extends SubsystemBase
{
	public SolSingle shooterSol = new SolSingle(idSolShooter);
	public SolSingle shooterLockSol = new SolSingle(idSolLock);

	public SubsystemShooter() {
		super();
	}
}
