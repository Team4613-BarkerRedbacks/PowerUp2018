package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;

public class SubsystemArm extends SubsystemBase {
	public CtrlMotor armMotor = new CtrlMotor(idMotArm);
	
	public int setpoint = 0;
	
	public SubsystemArm() {
		super();
	}
}
