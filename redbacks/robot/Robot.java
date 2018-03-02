package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * 
 * 
 * @author Sean Zammit, Matthew Brian, Ben Schwarz, Lucas Parker, Mitchell Barker, Darin Huang
 */
public class Robot extends ArachneRobot
{
	public static SubsystemSensors sensors = new SubsystemSensors();
	public static SubsystemDriver driver = new SubsystemDriver();
	public static SubsystemIntake intake = new SubsystemIntake();
	public static SubsystemClimber climber = new SubsystemClimber();
	public static SubsystemArm arm = new SubsystemArm();
	public static SubsystemShooter shooter = new SubsystemShooter();

	public static OI oi = new OI();

	private boolean hasCameraStarted = false;

	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
		arm.setDefaultCommand(armLoop.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		//TODO enable for secondary programmer
		//return autoID < 100 ? Auto.getAutonomous(autoID) : Auto2.getAutonomous(autoID);
		return Auto.getAutonomous(autoID);
	}

	public void initialiseRobot() {
		MotionSettings2.encoderTicksPerMetre = 26713;
		MotionSettings2.trajectoryMaxNegSpeed = -0.9;
		MotionSettings2.trajectoryMaxPosSpeed = 0.9;

		if(!hasCameraStarted) {
			new Thread(() -> {
				CameraServer.getInstance().startAutomaticCapture();

				CvSink cvSink = CameraServer.getInstance().getVideo();
				CvSource outputStream = CameraServer.getInstance().putVideo("Camera", 320, 240);

				Mat source = new Mat();

				while(!Thread.interrupted()) {
					cvSink.grabFrame(source);
					outputStream.putFrame(source);
				}
			}).start();
		}
	}

	public void initialiseAuto() {
		driver.centreEncoderSol.set(false);
	}

	public void initialiseTeleop() {
		driver.centreEncoderSol.set(true);
	}
}
