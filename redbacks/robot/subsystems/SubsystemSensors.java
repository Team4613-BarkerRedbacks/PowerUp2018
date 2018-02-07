package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;

public class SubsystemSensors extends SubsystemBase {
	
public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotDriveR2);
public SenCANEncoder.Displacement driveCentreEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
public SenCANEncoder.Displacement driveLeftEncoder = new SenCANEncoder.Displacement(idMotDriveL2);
public SenCANEncoder.Displacement driveRightEncoder = new SenCANEncoder.Displacement(idMotDriveR1);

	public SubsystemSensors(){
		super();
		resetSensors();
	}
	
	public void resetSensors() {
		armEncoder.set(0);
		driveCentreEncoder.set(0);
		driveLeftEncoder.set(0);
		driveRightEncoder.set(0);
	}
}
