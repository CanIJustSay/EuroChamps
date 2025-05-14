package Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Slides extends Subsystem {
    // BOILERPLATE
    public static final Slides INSTANCE = new Slides();
    private Slides() { }

    // USER CODE

    public MotorGroup arm_motors;
    private MotorEx leftArm;
    private MotorEx rightArm;

    public PIDFController controller =
            new PIDFController
                    (0.02, 0.0, 0.001, new StaticFeedforward(0.00 ),40);

    public String left_name = "leftArm";
    public String right_name = "rightArm";

    @Override
    public void initialize() {

        leftArm = new MotorEx(left_name);
        rightArm = new MotorEx(right_name);

        arm_motors = new MotorGroup(
                rightArm.reverse(),
                leftArm
        );
        leftArm.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.resetEncoder();

    }
    public Command toResting() {
        return new RunToPosition(arm_motors, // MOTOR TO MOVE
                0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toSample() {
        return new RunToPosition(arm_motors, // MOTOR TO MOVE
                3100, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toChamberPreload() {
        return new RunToPosition(arm_motors, // MOTOR TO MOVE
                1350.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toChamber() {
        return new RunToPosition(arm_motors, // MOTOR TO MOVE
                780.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public Command getDefaultCommand() {
            return new HoldPosition(arm_motors,controller,this);

    }
}