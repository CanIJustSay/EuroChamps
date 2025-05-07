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
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Intake extends Subsystem {
    // BOILERPLATE
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    // USER CODE
    public MotorEx intake;

    public PIDFController controller =
            new PIDFController
                    (0.027, 0.0, 0.001,
                            new StaticFeedforward(0.025),25);

    public String name = "linkage";


    @Override
    public void initialize() {
        intake = new MotorEx(name);
        intake.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.resetEncoder();

    }

    public Command goTo(double position) {
        return new RunToPosition(
                intake,
                position,
                controller,
                this
        );
    }

    public Command extend() {
        return new RunToPosition(intake,
                (270*0.7150827599),
                controller,
                this);
    }

    public Command retract() {
        return new RunToPosition(intake,
                0.0,
                controller,
                this);
    }


    @Override
    public void periodic(){


    }
}