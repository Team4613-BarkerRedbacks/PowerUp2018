package redbacks.robot;

import static redbacks.arachne.lib.input.ButtonGettableWrapper.wrap;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import redbacks.arachne.core.OIBase;
import redbacks.arachne.lib.checks.ChColorSensor;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.input.BtnAxis;
import redbacks.arachne.lib.input.BtnCheck;
import redbacks.arachne.lib.input.BtnMulti;
import redbacks.arachne.lib.input.BtnPOV;
import redbacks.arachne.lib.input.ButtonGettableWrapper;
import redbacks.arachne.lib.input.JoystickAxis;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.robot.subsystems.SubsystemSensors;

/**
 * Maps controls to commands during teleop.
 *
 * @author Ben Schwarz, Darin Huang, Lucas Parker, Matthew Brian, Sean Zammit
 */
public class OI extends OIBase {
	
	public static boolean climberReleased = false;
	
	public void mapOperations() {
		//Driver Control
		whenHeld(d_A, cubeFollow.c());
		whenReleased(d_A, stopIntake.c());
		whenPressed(d_Start, stopAll.c(), resetSensors.c());
		
		//Climber Control
		whenPressed(c_LB, climberRelease.c(), climbManual.c());
		whenPressed(c_RB, climberRelease.c(), climbManual.c());
		whenPressed(c_Y, armToTop.c());
		
		//Operator Control
		whenHeld(o_LT, intakeCubeAnalog.c());
		whenHeld(o_RT, outtakeCubeAnalog.c());
		whenPressedReleased(o_LB, sideKickL.c(), solRetractIntakeL.c());
		whenPressedReleased(o_RB, sideKickR.c(), solRetractIntakeR.c());

		//Arm Positions
		whenPressed(o_B, armToBaseBack.c());
		whenPressed(o_POV_R, armToLowBack.c());
		whenPressed(o_POV_D, armToScaleBack.c());
		whenPressed(o_Y, armToTop.c());
		whenPressed(o_POV_U, armToScaleFront.c());
		whenPressed(o_POV_L, armToLowFront.c());
		whenPressed(o_X, armToBaseFront.c());

		whenPressed(o_RX_L, armToSlightFront.c());
		whenPressed(o_RX_R, armToSlightBack.c());

		whenPressedReleased(o_A, highFirePrime.c(), highFireRelease.c());
//		whenPressedReleased(o_Start, superFirePrime.c(), superFireRelease.c());
		whenPressed(o_Start, highFirePrime.c());
		whenPressed(new BtnMulti(LogicOperators.AND, o_Start, new BtnCheck(new ChColorSensor(Robot.sensors.colorSensor) {
			public boolean reachedColorThreshold(double r, double g, double b) {
				return SubsystemSensors.isWhite(r, g, b);
			}
		}, false)), highFireRelease.c());
	}
	
	public static final Joystick stickDriver = new Joystick(0);
	
	public static final JoystickAxis
		axis_d_LX = new JoystickAxis(stickDriver, 0),
		axis_d_LY = new JoystickAxis(stickDriver, 1),
		axis_d_LT = new JoystickAxis(stickDriver, 2),
		axis_d_RT = new JoystickAxis(stickDriver, 3),
		axis_d_RX = new JoystickAxis(stickDriver, 4),
		axis_d_RY = new JoystickAxis(stickDriver, 5);
	
	public static final ButtonGettableWrapper
		d_A = wrap(new JoystickButton(stickDriver, 1)),
		d_B = wrap(new JoystickButton(stickDriver, 2)),
		d_X = wrap(new JoystickButton(stickDriver, 3)),
		d_Y = wrap(new JoystickButton(stickDriver, 4)),
		d_LB = wrap(new JoystickButton(stickDriver, 5)),
		d_RB = wrap(new JoystickButton(stickDriver, 6)),
		d_Back = wrap(new JoystickButton(stickDriver, 7)),
		d_Start = wrap(new JoystickButton(stickDriver, 8)),
		d_LStick = wrap(new JoystickButton(stickDriver, 9)),
		d_RStick = wrap(new JoystickButton(stickDriver, 10)),
	
		d_POV_U = wrap(new BtnPOV(stickDriver, 0)),
		d_POV_R = wrap(new BtnPOV(stickDriver, 90)),
		d_POV_D = wrap(new BtnPOV(stickDriver, 180)),
		d_POV_L = wrap(new BtnPOV(stickDriver, 270)),
	
		d_LT = wrap(new BtnAxis(axis_d_LT, false, 0.5D)),
		d_RT = wrap(new BtnAxis(axis_d_RT, false, 0.5D));
	
	public static final Joystick stickOperator = new Joystick(1);
	
	public static final JoystickAxis
		axis_o_LX = new JoystickAxis(stickOperator, 0),
		axis_o_LY = new JoystickAxis(stickOperator, 1),
		axis_o_LT = new JoystickAxis(stickOperator, 2),
		axis_o_RT = new JoystickAxis(stickOperator, 3),
		axis_o_RX = new JoystickAxis(stickOperator, 4),
		axis_o_RY = new JoystickAxis(stickOperator, 5);
	
	public static final ButtonGettableWrapper
		o_A = wrap(new JoystickButton(stickOperator, 1)),
		o_B = wrap(new JoystickButton(stickOperator, 2)),
		o_X = wrap(new JoystickButton(stickOperator, 3)),
		o_Y = wrap(new JoystickButton(stickOperator, 4)),
		o_LB = wrap(new JoystickButton(stickOperator, 5)),
		o_RB = wrap(new JoystickButton(stickOperator, 6)),
		o_Back = wrap(new JoystickButton(stickOperator, 7)),
		o_Start = wrap(new JoystickButton(stickOperator, 8)),
		o_LStick = wrap(new JoystickButton(stickOperator, 9)),
		o_RStick = wrap(new JoystickButton(stickOperator, 10)),
	
		o_POV_U = wrap(new BtnPOV(stickOperator, 0)),
		o_POV_R = wrap(new BtnPOV(stickOperator, 90)),
		o_POV_D = wrap(new BtnPOV(stickOperator, 180)),
		o_POV_L = wrap(new BtnPOV(stickOperator, 270)),
	
		o_LT = wrap(new BtnAxis(axis_o_LT, false, 0.2D)),
		o_RT = wrap(new BtnAxis(axis_o_RT, false, 0.2D)),
		
		o_RX_L = wrap(new BtnAxis(axis_o_RX, true, 0.5)),
		o_RX_R = wrap(new BtnAxis(axis_o_RX, false, 0.5));
	
	public static final Joystick stickClimber = new Joystick(2);
	
	public static final JoystickAxis
		axis_c_LX = new JoystickAxis(stickClimber, 0),
		axis_c_LY = new JoystickAxis(stickClimber, 1),
		axis_c_LT = new JoystickAxis(stickClimber, 2),
		axis_c_RT = new JoystickAxis(stickClimber, 3),
		axis_c_RX = new JoystickAxis(stickClimber, 4),
		axis_c_RY = new JoystickAxis(stickClimber, 5);
	
	public static final ButtonGettableWrapper
		c_A = wrap(new JoystickButton(stickClimber, 1)),
		c_B = wrap(new JoystickButton(stickClimber, 2)),
		c_X = wrap(new JoystickButton(stickClimber, 3)),
		c_Y = wrap(new JoystickButton(stickClimber, 4)),
		c_LB = wrap(new JoystickButton(stickClimber, 5)),
		c_RB = wrap(new JoystickButton(stickClimber, 6)),
		c_Back = wrap(new JoystickButton(stickClimber, 7)),
		c_Start = wrap(new JoystickButton(stickClimber, 8)),
		c_LStick = wrap(new JoystickButton(stickClimber, 9)),
		c_RStick = wrap(new JoystickButton(stickClimber, 10)),
	
		c_POV_U = wrap(new BtnPOV(stickClimber, 0)),
		c_POV_R = wrap(new BtnPOV(stickClimber, 90)),
		c_POV_D = wrap(new BtnPOV(stickClimber, 180)),
		c_POV_L = wrap(new BtnPOV(stickClimber, 270)),
	
		c_LT = wrap(new BtnAxis(axis_c_LT, false, 0.2D)),
		c_RT = wrap(new BtnAxis(axis_c_RT, false, 0.2D)),
		
		c_RX_L = wrap(new BtnAxis(axis_c_RX, true, 0.5)),
		c_RX_R = wrap(new BtnAxis(axis_c_RX, false, 0.5));
	
	public void whenPressedReleased(Button button, CommandBase onPressed, CommandBase onReleased) {
		button.whenPressed(onPressed);
		button.whenReleased(onReleased);
	}
	
}
