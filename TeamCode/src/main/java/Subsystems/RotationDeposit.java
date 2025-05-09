package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToSeperatePositions;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import java.util.Map;


public class RotationDeposit extends Subsystem {
    // BOILERPLATE
    public static final RotationDeposit INSTANCE = new RotationDeposit();
    private RotationDeposit() { }

    // USER CODE
    public Servo depoLeft;
    public Servo depoRight;

    public String name_left = "depoLeft";
    public String name_right = "depoRight";

    public Command down() {
        return new MultipleServosToSeperatePositions(Map.of(
                depoLeft, 0.0,
                depoRight,1.0
        ));
    }

    public Command up() {
        return new MultipleServosToSeperatePositions(Map.of(
                depoLeft, 0.45,
                depoRight,0.55
        ));
    }
    public Command mid() {
        return new MultipleServosToSeperatePositions(Map.of(
                depoLeft, 0.25,
                depoRight,0.75
        ));
    }



    @Override
    public void initialize() {
        depoLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name_left);
        depoRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name_right);

    }
}