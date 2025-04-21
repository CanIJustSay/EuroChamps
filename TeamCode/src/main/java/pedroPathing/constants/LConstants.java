package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00295;
        ThreeWheelConstants.strafeTicksToInches = 0.0029;
        ThreeWheelConstants.turnTicksToInches = 0.0029;
        ThreeWheelConstants.leftY = 5.518863;
        ThreeWheelConstants.rightY = -5.518863;
        ThreeWheelConstants.strafeX = -4.18;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftArm";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




