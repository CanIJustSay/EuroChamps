package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToSeperatePositions;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import java.util.Map;


public class RotationIntake extends Subsystem {
    // BOILERPLATE
    public static final RotationIntake INSTANCE = new RotationIntake();
    private RotationIntake() { }

    // USER CODE
    public Servo intakeLeft;
    public Servo intakeRight;

    public String name_left = "intakeLeft";
    public String name_right = "intakeRight";

    public Command down() {
        return new MultipleServosToSeperatePositions(Map.of(
                intakeLeft, 0.0,
                intakeRight,1.0
        )); // IMPLEMENTED SUBSYSTEM

    }

    public Command up() {
        return new MultipleServosToSeperatePositions(Map.of(
                intakeLeft, 0.65,
                intakeRight,0.35
        )); // IMPLEMENTED SUBSYSTEM

    }

    @Override
    public void initialize() {
        intakeLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name_left);
        intakeRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name_right);

    }
}