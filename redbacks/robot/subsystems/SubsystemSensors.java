package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.idMotArm;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.robot.Robot;

public class SubsystemSensors extends SubsystemBase {
	
public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotArm);
	
	public SubsystemSensors(){
		super();
	}
	
	public void resetSensors() {
		Robot.sensors.armEncoder.set(0);
	}
}
