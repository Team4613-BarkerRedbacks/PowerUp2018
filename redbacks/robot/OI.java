package redbacks.robot;

import static redbacks.arachne.lib.input.ButtonGettableWrapper.wrap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import redbacks.arachne.core.OIBase;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.input.BtnAxis;
import redbacks.arachne.lib.input.BtnPOV;
import redbacks.arachne.lib.input.ButtonGettableWrapper;
import redbacks.arachne.lib.input.JoystickAxis;

public class OI extends OIBase {
	
	@Override
	public void mapOperations() {
		whenHeld(d_A, CommandList.setArmFlatR.c());
		whenHeld(d_B, CommandList.setArmFlatL.c());
		whenHeld(d_X, CommandList.moveArm.c());
		whenHeld(d_Y, CommandList.reverseArm.c());
		
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
	
		o_LT = wrap(new BtnAxis(axis_o_LT, false, 0.5D)),
		o_RT = wrap(new BtnAxis(axis_o_RT, false, 0.5D));
	
	public void whenPressedReleased(Button button, CommandBase onPressed, CommandBase onReleased) {
		button.whenPressed(onPressed);
		button.whenReleased(onReleased);
	}

	public static final ButtonGettableWrapper
		d_1 = wrap(new JoystickButton(stickDriver, 1)),
		d_2 = wrap(new JoystickButton(stickDriver, 2));
		
		public static final ButtonGettableWrapper
			pov_d_45 = wrap(new BtnPOV(stickDriver, 45)),
			pov_d_0 = wrap(new BtnPOV(stickDriver, 0));

}
