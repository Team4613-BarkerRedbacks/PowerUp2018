package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;

public class SubsystemSensors extends SubsystemBase {
	
public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotArm);
	
	public SubsystemSensors(){
		super();
		resetSensors();
	}
	
	public void resetSensors() {
		armEncoder.set(0);
	}
}
