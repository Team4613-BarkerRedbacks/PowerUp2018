package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

/**
 * Subsystem to control the shooter on the arm.
 *
 * @author Ben Schwarz, Sean Zammit
 */
public class SubsystemShooter extends SubsystemBase
{
	public SolSingle shooterSol1 = new SolSingle(idSolShooter1);
	public SolSingle shooterSol2 = new SolSingle(idSolShooter2);
	public SolSingle shooterSolHigh = new SolSingle(idSolShooterHigh);
	public SolSingle shooterLockSol = new SolSingle(idSolLock);
}
