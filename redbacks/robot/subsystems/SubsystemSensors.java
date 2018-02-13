package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.navx.sensors.NavX;
import redbacks.arachne.ext.navx.sensors.NavXReading;
import redbacks.robot.actions.AcMonitor;

public class SubsystemSensors extends SubsystemBase
{
	public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotDriveL1);
	public SenCANEncoder.Displacement driveCentreEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	public SenCANEncoder.Displacement driveLeftEncoder = new SenCANEncoder.Displacement(idMotDriveR1);
	public SenCANEncoder.Displacement driveRightEncoder = new SenCANEncoder.Displacement(idMotDriveL2);

	public SubsystemSensors() {
		super();
		resetSensors();
	}

	public void resetSensors() {
		armEncoder.set(0);
		driveCentreEncoder.set(0);
		driveLeftEncoder.set(0);
		driveRightEncoder.set(0);
		yaw.set(0);

		AcMonitor.oldAng = 0;
		AcMonitor.oldDis = 0;
		AcMonitor.posX = 0;
		AcMonitor.posY = 0;
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
