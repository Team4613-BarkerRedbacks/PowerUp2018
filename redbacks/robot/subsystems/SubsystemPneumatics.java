package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

public class SubsystemPneumatics extends SubsystemBase {
	
	public SolSingle drivetrainEncoderSol = new SolSingle(2);
	
	public SubsystemPneumatics() {
		super();
	}

}
