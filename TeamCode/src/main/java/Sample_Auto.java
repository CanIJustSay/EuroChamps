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
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroData;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

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

    private final Pose startPose = new Pose(8.2, 60.0, Math.toRadians(0.0));
    private final Pose basketPose = new Pose(17, 130.2, Math.toRadians(-45));
    private final Pose grabFirst = new Pose(18, 121, Math.toRadians(0));
    private final Pose grabSecond = new Pose(18, 132, Math.toRadians(2));
    private final Pose grabThird = new Pose(25, 125, Math.toRadians(45));

    private ElapsedTime elapsedtime;

    private Follower follower;



    private PathChain scorePreload;
    private PathChain moveToFirst;
    private PathChain moveToSecond;
    private PathChain moveToThird;
    private PathChain scoreFirst;
    private PathChain scoreSecond;
    private PathChain scoreThird;

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


    }

    public Command runAuto(){
        return new SequentialGroup(

                //preload score
                new ParallelGroup(
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
                    ClawDeposit.INSTANCE.open().thenWait(0.3),
                    new ParallelGroup(
                            RotationDeposit.INSTANCE.down(),
                            Deposit.INSTANCE.getReady()
                    )
                ),

                //grab first
                new SequentialGroup(
                        new ParallelGroup(
                                Slides.INSTANCE.toResting(),
                                new FollowPath(moveToFirst,true),
                                Intake.INSTANCE.extend(),
                                ClawIntake.INSTANCE.open(),
                                RotationIntake.INSTANCE.down()
                        ).thenWait(0.5),
                        ClawIntake.INSTANCE.close().thenWait(0.35)
                ),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.5),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.2),
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
                        ClawDeposit.INSTANCE.open().thenWait(0.3),
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
                        ).thenWait(0.5),
                        ClawIntake.INSTANCE.close().thenWait(0.35)
                ),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        ).thenWait(0.5),
                        Deposit.INSTANCE.toTransfer(),
                        ClawDeposit.INSTANCE.close().thenWait(0.2),
                        ClawIntake.INSTANCE.open()
                ),

                new ParallelGroup(
                        new FollowPath(scoreSecond,true),
                        Deposit.INSTANCE.toScore(),
                        Slides.INSTANCE.toSample()
                ),

                //score
                new SequentialGroup(
                        RotationDeposit.INSTANCE.up().thenWait(0.35),
                        ClawDeposit.INSTANCE.open().thenWait(0.3),
                        RotationDeposit.INSTANCE.down(),
                        Deposit.INSTANCE.getReady(),
                        Slides.INSTANCE.toResting()
                ),

                //grab third
                new SequentialGroup(
                        new ParallelGroup(
                                new FollowPath(moveToThird),
                                ClawIntake.INSTANCE.open(),
                                RotationIntake.INSTANCE.down()
                        ).thenWait(0.5),
                        Intake.INSTANCE.extend(),
                        ClawIntake.INSTANCE.close().thenWait(0.5)
                ),

                //transfer
                new SequentialGroup(
                        //score the preload, bring back deposit and slides
                        new ParallelGroup(
                                RotationIntake.INSTANCE.up(),
                                Intake.INSTANCE.retract()

                        )
                )

//                new ParallelGroup(
//                        new FollowPath(scoreThird,true),
//                        Deposit.INSTANCE.toScore(),
//                        Slides.INSTANCE.toSample()
//                ),
//
//                //score
//                new SequentialGroup(
//                        RotationDeposit.INSTANCE.up().thenWait(0.5),
//                        ClawDeposit.INSTANCE.open().thenWait(0.3),
//                        RotationDeposit.INSTANCE.down(),
//                        Deposit.INSTANCE.getReady(),
//                        Slides.INSTANCE.toResting()
//                )
        );
    };

    @Override
    public void onInit() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        PedroData.INSTANCE.setFollower(follower);

        follower.setStartingPose(new Pose(8.2, 111.8, 0));
        buildPaths();

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void onStartButtonPressed() {
        runAuto().invoke();


        //Deposit.INSTANCE.toScore().invoke();
    }
    @Override
    public void onUpdate(){
        telemetry.addData("Loop Times", elapsedtime.milliseconds());

        telemetry.addData("slides pos", Slides.INSTANCE.arm_motors.getCurrentPosition());

        telemetry.update();
        elapsedtime.reset();

        follower.update();
    }
}