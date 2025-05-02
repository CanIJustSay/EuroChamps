package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class ClawDeposit extends Subsystem {
    // BOILERPLATE
    public static final ClawDeposit INSTANCE = new ClawDeposit();
    private ClawDeposit() { }

    // USER CODE
    public Servo clawDeposit;

    public Servo rotateDeposit;

    public String name = "depositClaw";

    public String rotate_name = "rotateDeposit";

    public Command open() {
        return new ServoToPosition(clawDeposit, // SERVO TO MOVE
                0.25, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(clawDeposit, // SERVO TO MOVE
                0, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }


    public Command flipUp() {
        return new ServoToPosition(rotateDeposit,
                0.5,
                this);
    }
    public Command flipDown() {
        return new ServoToPosition(rotateDeposit,
                0,
                this);
    }

    @Override
    public void initialize() {
        clawDeposit = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        rotateDeposit = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, rotate_name);

    }
}