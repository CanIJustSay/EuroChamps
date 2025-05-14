import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
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
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.core.units.Angle;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroData;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
import com.rowanmcalpin.nextftc.pedro.Turn;
import com.rowanmcalpin.nextftc.pedro.TurnTo;

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
    pipeline freakyLine = new pipeline();



    private final Pose startPose = new Pose(8.2, 55, Math.toRadians(0.0));
    private final Pose chamberPose = new Pose(38.8, 70.5, Math.toRadians(0.0));

    private final Pose chamber1 = new Pose(46, 69.5, Math.toRadians(0.0));
    private final Pose chamber2 = new Pose(46, 68.5, Math.toRadians(0.0));

    private final Pose chamber3 = new Pose(46, 67.5, Math.toRadians(0.0));

    private final Pose chamber4 = new Pose(46, 66.5, Math.toRadians(0.0));


    private final Pose pickHumanPlayer = new Pose(12.6, 35.4, Math.toRadians(180));
    private final Pose firstPose = new Pose(27,43,Math.toRadians(-45));
    private final Pose secondPose = new Pose(26,32.4,Math.toRadians(-45));
    private final Pose thirdPose = new Pose(34.5,28.5,Math.toRadians(-63));

    private final Pose dropFirstPose = new Pose(27,37.5,Math.toRadians(-120));
    private final Pose dropSecondPose = new Pose(28.6,34,Math.toRadians(-130));
    private final Pose dropThirdPose = new Pose(28.6,30,Math.toRadians(-135));










    private Follower follower;



    private PathChain scorePreload;
    private PathChain scoreSpecimen1;
    private PathChain scoreSpecimen2;
    private PathChain scoreSpecimen3;
    private PathChain scoreSpecimen4;
    private PathChain grabSpecimen;
    private PathChain grabSpecimenFromPickUp;

    private PathChain grabFirst;
    private PathChain grabSecond;
    private PathChain grabThird;
    private PathChain dropFirst;
    private PathChain dropSecond;
    private PathChain dropThird;



    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())
                .build();

        grabSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(pickHumanPlayer)))
                .setConstantHeadingInterpolation(pickHumanPlayer.getHeading())
                .build();





        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickHumanPlayer),new Point(31.013, 31.348, Point.CARTESIAN),
                        new Point(13.746, 75.604, Point.CARTESIAN), new Point(chamber1)))
                .setConstantHeadingInterpolation(pickHumanPlayer.getHeading())
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickHumanPlayer),new Point(31.013, 31.348, Point.CARTESIAN),
                        new Point(13.746, 75.604, Point.CARTESIAN), new Point(chamber2)))
                .setConstantHeadingInterpolation(pickHumanPlayer.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickHumanPlayer),new Point(31.013, 31.348, Point.CARTESIAN),
                        new Point(13.746, 75.604, Point.CARTESIAN), new Point(chamber3)))
                .setConstantHeadingInterpolation(pickHumanPlayer.getHeading())
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickHumanPlayer),new Point(31.013, 31.348, Point.CARTESIAN),
                        new Point(13.746, 75.604, Point.CARTESIAN), new Point(chamber4)))
                .setConstantHeadingInterpolation(pickHumanPlayer.getHeading())
                .build();







        grabFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(firstPose)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), firstPose.getHeading())
                .build();
        dropFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPose),new Point(dropFirstPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), dropFirstPose.getHeading())
                .build();


        grabSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropFirstPose), new Point(secondPose)))
                .setLinearHeadingInterpolation(dropFirstPose.getHeading(), secondPose.getHeading())
                .build();
        dropSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPose),new Point(dropSecondPose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), dropSecondPose.getHeading())
                .build();


        grabThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropSecondPose), new Point(thirdPose)))
                .setLinearHeadingInterpolation(dropSecondPose.getHeading(), thirdPose.getHeading())
                .build();
        dropThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPose),new Point(dropFirstPose)))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), dropFirstPose.getHeading())
                .build();


        grabSpecimenFromPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropFirstPose),new Point(pickHumanPlayer)))
                .setLinearHeadingInterpolation(dropThirdPose.getHeading(), pickHumanPlayer.getHeading())
                .build();







    }

    public Command runAuto(){
        return new SequentialGroup(
                ClawDeposit.INSTANCE.close().thenWait(0.2),
                ClawDeposit.INSTANCE.flipUp(),
                RotationDeposit.INSTANCE.mid().thenWait(0.1),
                new ParallelGroup(
                    Intake.INSTANCE.retract(),
                    ClawIntake.INSTANCE.open(),
                    ClawIntake.INSTANCE.rotateHorizontal(),
                    Deposit.INSTANCE.getReady(),
                    Slides.INSTANCE.toChamberPreload(),
                    new FollowPath(scorePreload,true)

                ),
                new ParallelGroup(
                        Slides.INSTANCE.toResting(),
                        ClawDeposit.INSTANCE.open().afterTime(0.3)
                ),

                new FollowPath(grabFirst,true),
                new ParallelGroup(
                    Intake.INSTANCE.extend(),
                        RotationIntake.INSTANCE.down(),
                        ClawIntake.INSTANCE.rotateDiagLeft()

                ),
                new SequentialGroup(

                    ClawIntake.INSTANCE.close().thenWait(0.1),

                    new FollowPath(dropFirst,true).thenWait(0.1),

                    ClawIntake.INSTANCE.open(),

                    new FollowPath(grabSecond,true).thenWait(0.3),

                    ClawIntake.INSTANCE.close().thenWait(0.2),

                    new FollowPath(dropSecond,true).thenWait(0.1),

                    ClawIntake.INSTANCE.open(),
                    Intake.INSTANCE.retract(),

                    new FollowPath(grabThird,true),
                    new ParallelGroup(
                        Intake.INSTANCE.extend(),
                        ClawIntake.INSTANCE.rotateVertical()
                    ).thenWait(0.2),

                    ClawIntake.INSTANCE.close().thenWait(0.1),

                    new FollowPath(dropThird,true)

                ),

                ClawIntake.INSTANCE.open(),

                new ParallelGroup(
                        Intake.INSTANCE.retract(),
                        ClawIntake.INSTANCE.rotateHorizontal(),
                        RotationIntake.INSTANCE.up()
                ),

                RotationDeposit.INSTANCE.up().thenWait(0.1),
                ClawDeposit.INSTANCE.flipUp(),
                ClawDeposit.INSTANCE.open(),
                Deposit.INSTANCE.toPickUp(),

                new FollowPath(grabSpecimenFromPickUp),
                ClawDeposit.INSTANCE.close().thenWait(0.1),
                new ParallelGroup(
                    new FollowPath(scoreSpecimen1),
                    Deposit.INSTANCE.toChamber(),
                    Slides.INSTANCE.toChamber(),
                    RotationDeposit.INSTANCE.scoreSpeci().thenWait(0.1),
                    ClawDeposit.INSTANCE.flipDown()

                ),


                ClawDeposit.INSTANCE.open().thenWait(0.1),

                new ParallelGroup(
                        Slides.INSTANCE.toResting(),
                        new FollowPath(grabSpecimenFromPickUp),
                        RotationDeposit.INSTANCE.up(),
                        new ParallelGroup(
                            ClawDeposit.INSTANCE.flipUp(),
                            ClawDeposit.INSTANCE.open(),
                            Deposit.INSTANCE.toPickUp()

                    )
                ),



                ClawDeposit.INSTANCE.close().thenWait(0.1),
                new ParallelGroup(
                    Deposit.INSTANCE.toChamber(),
                    Slides.INSTANCE.toChamber(),
                    RotationDeposit.INSTANCE.scoreSpeci().thenWait(0.1),
                    ClawDeposit.INSTANCE.flipDown(),
                    new FollowPath(scoreSpecimen2)
                ),

                ClawDeposit.INSTANCE.open().thenWait(0.1),

                new ParallelGroup(
                        Slides.INSTANCE.toResting(),
                        new FollowPath(grabSpecimenFromPickUp),
                        RotationDeposit.INSTANCE.up(),
                        new ParallelGroup(
                                ClawDeposit.INSTANCE.flipUp(),
                                ClawDeposit.INSTANCE.open(),
                                Deposit.INSTANCE.toPickUp()

                        )
                ),



                ClawDeposit.INSTANCE.close().thenWait(0.1),
                new ParallelGroup(
                    Deposit.INSTANCE.toChamber(),
                    Slides.INSTANCE.toChamber(),
                    RotationDeposit.INSTANCE.scoreSpeci().thenWait(0.1),
                    ClawDeposit.INSTANCE.flipDown(),
                    new FollowPath(scoreSpecimen3)
                ),

                ClawDeposit.INSTANCE.open().thenWait(0.1),

                new ParallelGroup(
                        Slides.INSTANCE.toResting(),
                        new FollowPath(grabSpecimenFromPickUp),
                        RotationDeposit.INSTANCE.up(),
                        new ParallelGroup(
                                ClawDeposit.INSTANCE.flipUp(),
                                ClawDeposit.INSTANCE.open(),
                                Deposit.INSTANCE.toPickUp()

                        )
                ),


                ClawDeposit.INSTANCE.close().thenWait(0.1),
                Deposit.INSTANCE.toChamber(),
                Slides.INSTANCE.toChamber(),
                RotationDeposit.INSTANCE.scoreSpeci().thenWait(0.1),
                ClawDeposit.INSTANCE.flipDown(),

                new FollowPath(scoreSpecimen4)



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
    private static double normalize(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }


}