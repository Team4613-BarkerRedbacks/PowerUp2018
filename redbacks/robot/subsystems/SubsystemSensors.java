package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;

public class SubsystemSensors extends SubsystemBase {
	
public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotDriveL2);
public SenCANEncoder.Displacement drivetrainEncoder = new SenCANEncoder.Displacement(idMotDriveR1);

	public SubsystemSensors(){
		super();
		resetSensors();
	}
	
	public void resetSensors() {
		armEncoder.set(0);
		drivetrainEncoder.set(0);
	
	}
}
