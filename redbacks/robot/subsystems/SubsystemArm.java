package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;

public class SubsystemArm extends SubsystemBase {
	public CtrlMotor aMotor = new CtrlMotor(idMotArm);
	
	public SubsystemArm() {
		super();
	}
}
