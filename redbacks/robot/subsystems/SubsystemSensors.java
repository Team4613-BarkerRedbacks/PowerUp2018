package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.I2C.Port;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.navx.sensors.NavX;
import redbacks.arachne.ext.navx.sensors.NavXReading;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.sensors.ModernRoboticsColorSensor;
import redbacks.arachne.lib.sensors.ModernRoboticsColorSensor.Frequency;
import redbacks.robot.RobotMap;

public class SubsystemSensors extends SubsystemBase
{
	public SenCANEncoder.Displacement armEncoder = new SenCANEncoder.Displacement(idMotDriveL1);
	public SenCANEncoder.Displacement driveLeftEncoder = new SenCANEncoder.Displacement(idMotDriveL2);
	public SenCANEncoder.Displacement driveRightEncoder = new SenCANEncoder.Displacement(idMotDriveR1);
	
	private SenCANEncoder.Displacement driveCentreEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
	public SenCANEncoder.Rate driveSpeed = new SenCANEncoder.Rate(idMotDriveL3);

	public SenCANEncoder.Displacement driveMonitorEncoderL = new SenCANEncoder.Displacement(idMotDriveL2);
	public SenCANEncoder.Displacement driveMonitorEncoderR = new SenCANEncoder.Displacement(idMotDriveR1);
	
	public ModernRoboticsColorSensor colorSensor = new ModernRoboticsColorSensor(Port.kMXP, Frequency.HZ_50);
	
	public NumericSensor averageEncoder = new NumericSensor() {
		protected double getSenVal() {
			return (driveLeftEncoder.get() + driveRightEncoder.get()) / 2 / RobotMap.sideEncoderTicksPerMetre * MotionSettings2.encoderTicksPerMetre;
		}
	};

//	public NumericSensor distanceEncoder = new NumericSensor() {
//		protected double getSenVal() {
//			return driveRightEncoder.get() / RobotMap.sideEncoderTicksPerMetre * MotionSettings2.encoderTicksPerMetre;
//		}
//	};

	public NumericSensor distanceEncoder = driveCentreEncoder;

	public SubsystemSensors() {
		super();
		resetSensors();
		driveRightEncoder.setScaleFactor(-1);
		driveMonitorEncoderR.setScaleFactor(-1);
	}

	public void resetSensors() {
		driveCentreEncoder.set(0);
		driveLeftEncoder.set(0);
		driveRightEncoder.set(0);
		yaw.set(0);
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
