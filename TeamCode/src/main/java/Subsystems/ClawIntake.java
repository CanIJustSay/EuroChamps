package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class ClawIntake extends Subsystem {
    // BOILERPLATE
    public static final ClawIntake INSTANCE = new ClawIntake();
    private ClawIntake() { }

    // USER CODE
    public Servo clawIntake;
    public Servo rotateIntake;

    public String name = "intakeClaw";
    public String rotate_n = "rotateIntake";

    public Command open() {
        return new ServoToPosition(clawIntake, // SERVO TO MOVE
                0.25, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command rotateVertical(){
        return new ServoToPosition(rotateIntake,
                0,
                this);
    }

    public Command rotateHorizontal() {
        return new ServoToPosition(rotateIntake, // SERVO TO MOVE
                0.5, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(clawIntake, // SERVO TO MOVE
                0, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        clawIntake = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        rotateIntake = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,rotate_n);


    }
}