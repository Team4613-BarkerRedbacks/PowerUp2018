package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

public class SubsystemPneumatics extends SubsystemBase {
	
	public SolSingle centreEncoderSol = new SolSingle(idSolCentreEncoder);
	
	public SubsystemPneumatics() {
		super();
	}

}
