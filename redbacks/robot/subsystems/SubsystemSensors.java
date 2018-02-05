package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.navx.sensors.NavX;
import redbacks.arachne.ext.navx.sensors.NavXReading;

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
	
	public NavX.Sensor pitch = new NavX.Sensor(NavXReading.ANGLE_PITCH);
	public NavX.Sensor roll = new NavX.Sensor(NavXReading.ANGLE_ROLL);
	public NavX.Yaw yaw = new NavX.Yaw();

	public NavX.Sensor ratePitch = new NavX.Sensor(NavXReading.RATE_PITCH);
	public NavX.Sensor rateRoll = new NavX.Sensor(NavXReading.RATE_ROLL);
	public NavX.Sensor rateYaw = new NavX.Sensor(NavXReading.RATE_YAW);

	public NavX.Sensor accelForward = new NavX.Sensor(NavXReading.ACCEL_FORWARD);
	public NavX.Sensor accelRight = new NavX.Sensor(NavXReading.ACCEL_RIGHT);
	public NavX.Sensor accelUp = new NavX.Sensor(NavXReading.ACCEL_UP);

	public NavX.Sensor speedForward = new NavX.Sensor(NavXReading.SPEED_FORWARD);
	public NavX.Sensor speedRight = new NavX.Sensor(NavXReading.SPEED_RIGHT);
	public NavX.Sensor speedUp = new NavX.Sensor(NavXReading.SPEED_UP);

}
