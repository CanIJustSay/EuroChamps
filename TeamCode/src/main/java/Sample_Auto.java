import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroData;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Sample Auto")
public class Sample_Auto extends NextFTCOpMode {
    public Sample_Auto() {
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
    poopline freakyLine = new poopline();


    double extension;
    double lateralDisplacement;



    private final Pose startPose = new Pose(8.2, 60.0, Math.toRadians(0.0));
    private final Pose basketPose = new Pose(17, 130.2, Math.toRadians(-45));
    private final Pose grabFirst = new Pose(18, 121, Math.toRadians(0));
    private final Pose grabSecond = new Pose(18, 131.5, Math.toRadians(2));
    private final Pose grabThird = new Pose(33.25, 117.5, Math.toRadians(63.5));
    private final Pose subPath = new Pose(70,125,Math.toRadians(-90));
    private final Pose  sub = new Pose(70,95,Math.toRadians(-90));

    private Point detected = new Point(lateralDisplacement,95);




    private Follower follower;



    private PathChain scorePreload;
    private PathChain moveToFirst;
    private PathChain moveToSecond;
    private PathChain moveToThird;
    private PathChain scoreFirst;
    private PathChain scoreSecond;
    private PathChain scoreThird;
    private PathChain subPickUp;
    private PathChain subScore;
    private PathChain subPathing;
    private PathChain cameraMovement;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(basketPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), basketPose.getHeading())
                .build();

        moveToFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose),new Point(grabFirst)))
                .setLinearHeadingInterpolation(basketPose.getHeading(),grabFirst.getHeading())
                .build();

        scoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabFirst),new Point(basketPose)))
                .setLinearHeadingInterpolation(grabFirst.getHeading(),basketPose.getHeading())
                .build();

        moveToSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose),new Point(grabSecond)))
                .setLinearHeadingInterpolation(basketPose.getHeading(),grabSecond.getHeading())
                .build();

        scoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSecond),new Point(basketPose)))
                .setLinearHeadingInterpolation(grabSecond.getHeading(),basketPose.getHeading())
                .build();

        moveToThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose),new Point(grabThird)))
                .setLinearHeadingInterpolation(basketPose.getHeading(),grabThird.getHeading())
                .build();

        scoreThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabThird),new Point(basketPose)))
                .setLinearHeadingInterpolation(grabThird.getHeading(),basketPose.getHeading())
                .build();
        subPathing = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose),new Point(subPath)))
                .setLinearHeadingInterpolation(basketPose.getHeading(),subPath.getHeading())
                .build();
        subPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPath),new Point(sub)))
                .setLinearHeadingInterpolation(subPath.getHeading(),sub.getHeading())
                .build();



    }

    public Command runAuto(){
        return new SequentialGroup(

                //preload score
                new ParallelGroup(
                        ClawIntake.INSTANCE.rotateHorizontal(),
                        ClawDeposit.INSTANCE.flipUp(),
                        RotationDeposit.INSTANCE.down(),
                        Intake.INSTANCE.retract(),
                        Deposit.INSTANCE.toScore(),
                        ClawIntake.INSTANCE.open(),
                        new FollowPath(scorePreload),
                        ClawDeposit.INSTANCE.close(),
                        Slides.INSTANCE.toSample()
                ),

                //score
                new SequentialGroup(
                    RotationDeposit.INSTANCE.up().thenWait(0.35),
                    ClawDeposit.INSTANCE.open().thenWait(0.1),
                    new ParallelGroup(
                            RotationDeposit.INSTANCE.down(),
                            Deposit.INSTANCE.getReady()
                    )
                ),

                //move first
                new SequentialGroup(
                        new ParallelGroup(
                                new FollowPath(moveToFirst,true),
                                Slides.INSTANCE.toResting(),
                                Intake.INSTANCE.extend(),
                                ClawIntake.INSTANCE.open(),
                                RotationIntake.INSTANCE.down()
                        ).thenWait(0.15),
                        ClawIntake.INSTANCE.close().thenWait(0.35)
                ),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.2),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.1),
                        ClawIntake.INSTANCE.open()
                ),

                new ParallelGroup(
                        new FollowPath(scoreFirst,true),
                        Deposit.INSTANCE.toScore(),
                        Slides.INSTANCE.toSample()
                ),

                //score
                new SequentialGroup(
                        RotationDeposit.INSTANCE.up().thenWait(0.35),
                        ClawDeposit.INSTANCE.open().thenWait(0.1),
                        new ParallelGroup(
                            RotationDeposit.INSTANCE.down(),
                            Deposit.INSTANCE.getReady()
                        )
                ),



                //move second
                new SequentialGroup(
                        new ParallelGroup(
                                new FollowPath(moveToSecond,true),
                                Slides.INSTANCE.toResting(),
                                Intake.INSTANCE.extend(),
                                ClawIntake.INSTANCE.open(),
                                RotationIntake.INSTANCE.down()
                        ).thenWait(0.15),
                        ClawIntake.INSTANCE.close().thenWait(0.35)
                ),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.2),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.1),
                        ClawIntake.INSTANCE.open()
                ),

                new ParallelGroup(
                        new FollowPath(scoreSecond,true),
                        Deposit.INSTANCE.toScore(),
                        Slides.INSTANCE.toSample()
                ),

                //move to third
                new SequentialGroup(
                        RotationDeposit.INSTANCE.up().thenWait(0.35),
                        ClawDeposit.INSTANCE.open().thenWait(0.1)

                ),
                new ParallelGroup(
                        RotationDeposit.INSTANCE.down(),
                        Deposit.INSTANCE.getReady(),
                        Slides.INSTANCE.toResting(),
                        new FollowPath(moveToThird),
                        ClawIntake.INSTANCE.open(),
                        ClawIntake.INSTANCE.rotateDiagRight(),
                        RotationIntake.INSTANCE.down()
                ).thenWait(0.35),
                Intake.INSTANCE.extend(),
                ClawIntake.INSTANCE.close().thenWait(0.2),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                ClawIntake.INSTANCE.rotateHorizontal(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.2),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.1),
                        ClawIntake.INSTANCE.open()
                ),

                new ParallelGroup(
                        new FollowPath(scoreThird,true),
                        Deposit.INSTANCE.toScore(),
                        Slides.INSTANCE.toSample()
                ),

                //score
                new SequentialGroup(
                        RotationDeposit.INSTANCE.up().thenWait(0.5),
                        ClawDeposit.INSTANCE.open().thenWait(0.3),
                        new ParallelGroup(
                                RotationDeposit.INSTANCE.down(),
                                Deposit.INSTANCE.getReady(),
                                Slides.INSTANCE.toResting(),
                                ClawIntake.INSTANCE.rotateHorizontal(),
                                new FollowPath(subPathing,true)
                        )

                ),

                new FollowPath(subPickUp,true).thenWait(1),
                new SequentialGroup(
                        new ParallelGroup(

                                //translate extension into a pid target

                                //change extension amount based off camera

                                //translate x displacement into field coordinates and put that as "lateralDisplacement

                                //Intake.INSTANCE.goTo(extension),

                                //followPath or movement while extending probably

                                //lateralMovement(detected),

                                ClawIntake.INSTANCE.open(),
                                RotationIntake.INSTANCE.down()
                        ).thenWait(0.35),
                        ClawIntake.INSTANCE.close().thenWait(0.35)
                ),
                new SequentialGroup(
                //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.2),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.1),
                        ClawIntake.INSTANCE.open()

                )
        );
    };

    @Override
    public void onInit() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        PedroData.INSTANCE.setFollower(follower);

        follower.setStartingPose(new Pose(8.2, 111.8, 0));
        buildPaths();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "logi"), cameraMonitorViewId);
        webcam.setPipeline(freakyLine);

        webcam.setMillisecondsPermissionTimeout(5000);
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


    }

    @Override
    public void onStartButtonPressed() {

        runAuto().invoke();

    }
    @Override
    public void onUpdate(){

        telemetry.addData("slides pos", Slides.INSTANCE.arm_motors.getCurrentPosition());
        telemetry.addData("deposit pos", Deposit.INSTANCE.deposit.getCurrentPosition());
        telemetry.addData("intake pos", Intake.INSTANCE.intake.getCurrentPosition());


        telemetry.addData("Displacement x", freakyLine.dx);

        telemetry.addData("Displacement y", freakyLine.dy);

        telemetry.addData("orientation(sexual)", freakyLine.orient);



        telemetry.update();


        follower.update();
    }
    public Command lateralMovement(Point point) {
        return new LambdaCommand()
                .setStart(() -> follower.holdPoint(point,Math.toRadians(-90)))
                .setIsDone(() -> follower.atPoint(point, 0.25, 0.25));
    }

}