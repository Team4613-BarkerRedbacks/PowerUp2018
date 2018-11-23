package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;

public class SubsystemSensors extends SubsystemBase
{
	public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotDriveR);
}
