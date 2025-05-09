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

@Autonomous(name = "Specimen Auto")
public class Specimen_Auto extends NextFTCOpMode {
    public Specimen_Auto() {
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



    private final Pose startPose = new Pose(8.2, 55, Math.toRadians(0.0));
    private final Pose chamberPose = new Pose(39.5, 68.5, Math.toRadians(0.0));
    private final Pose pickHumanPlayer = new Pose(8.2, 34.3, Math.toRadians(90));






    private Follower follower;



    private PathChain scorePreload;
    private PathChain scoreSpecimen;
    private PathChain grabSpecimen;


    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())
                .build();

        grabSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(pickHumanPlayer)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), pickHumanPlayer.getHeading())
                .build();
        scoreSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickHumanPlayer), new Point(chamberPose)))
                .setLinearHeadingInterpolation(pickHumanPlayer.getHeading(), chamberPose.getHeading())
                .build();



    }

    public Command runAuto(){
        return new SequentialGroup(
                ClawDeposit.INSTANCE.close().thenWait(0.2),
                new ParallelGroup(
                    Intake.INSTANCE.retract(),
                    ClawIntake.INSTANCE.open(),
                    ClawIntake.INSTANCE.rotateHorizontal(),
                    Deposit.INSTANCE.getReady(),
                    RotationDeposit.INSTANCE.mid(),
                    Slides.INSTANCE.toChamber(),
                    new FollowPath(scorePreload,true)

                ),
                new ParallelGroup(
                        Slides.INSTANCE.toResting().afterTime(0.2),
                        ClawDeposit.INSTANCE.open()
                )

        );
    };

    @Override
    public void onInit() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        PedroData.INSTANCE.setFollower(follower);

        follower.setStartingPose(startPose);
        buildPaths();



    }

    @Override
    public void onStartButtonPressed() {

        runAuto().invoke();

    }
    @Override
    public void onUpdate(){


        follower.update();
    }

}