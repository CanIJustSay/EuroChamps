import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import Subsystems.ClawDeposit;
import Subsystems.ClawIntake;
import Subsystems.Deposit;
import Subsystems.Intake;
import Subsystems.RotationDeposit;
import Subsystems.RotationIntake;
import Subsystems.Slides;
@Config
@TeleOp(name = "Camera Test")
public class Camera_Test extends NextFTCOpMode {

    public Camera_Test() {
        super(
                ClawDeposit.INSTANCE,
                Deposit.INSTANCE,
                RotationDeposit.INSTANCE,
                RotationIntake.INSTANCE,
                ClawIntake.INSTANCE,
                Intake.INSTANCE,
                Slides.INSTANCE

        );
    }
    OpenCvWebcam webcam;
    public static double multValue = -0.856445;
    public static double intercept = 100;
    poopline freakyLine = new poopline();


    public Command driverControlled;

    @Override
    public void onInit() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "logi"), cameraMonitorViewId);


        webcam.setPipeline(freakyLine);




        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        ClawIntake.INSTANCE.open().invoke();
        RotationIntake.INSTANCE.up().invoke();
        RotationDeposit.INSTANCE.down().invoke();
        ClawDeposit.INSTANCE.open().invoke();
        Intake.INSTANCE.retract().invoke();
        Deposit.INSTANCE.getReady().invoke();
        ClawDeposit.INSTANCE.flipUp().invoke();



        // Change your motor directions to suit your robot.


    }
    @Override
    public void onStartButtonPressed() {





        telemetry.update();


        ClawIntake.INSTANCE.open().invoke();

        RotationIntake.INSTANCE.down().invoke();








    }
    @Override
    public void onUpdate(){
        double convertvalue = ((-freakyLine.dy) + 100) * multValue + intercept;
        telemetry.addData("Displacement x", freakyLine.dx);
        telemetry.addData("Displacement y", (-freakyLine.dy) + 100);
        telemetry.addData("orientation(sexual)", freakyLine.orient);
        telemetry.addData("linkage pos", Intake.INSTANCE.intake.getCurrentPosition());
        telemetry.addData("converted", convertvalue);
        telemetry.update();

        if (freakyLine.orient == "horizontal") {
            ClawIntake.INSTANCE.rotateVertical().invoke();
        }
        else if (freakyLine.orient == "diagonal right") {
            ClawIntake.INSTANCE.rotateDiagRight().invoke();
        }
        else if (freakyLine.orient == "vertical") {
            ClawIntake.INSTANCE.rotateHorizontal().invoke();
        }
        else if (freakyLine.orient == "diagonal left") {
            ClawIntake.INSTANCE.rotateDiagLeft().invoke();
        }
        else {
            ClawIntake.INSTANCE.rotateHorizontal().invoke();
        }

        Intake.INSTANCE.goTo(convertvalue).invoke();


    }
}