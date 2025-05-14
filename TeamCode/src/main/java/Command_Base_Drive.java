import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import Subsystems.ClawDeposit;
import Subsystems.ClawIntake;
import Subsystems.Deposit;
import Subsystems.Intake;
import Subsystems.RotationDeposit;
import Subsystems.RotationIntake;
import Subsystems.Slides;

@TeleOp(name = "Command Base Drive")
public class Command_Base_Drive extends NextFTCOpMode {

    public Command_Base_Drive() {
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

    public String frontLeftName = "leftFront";
    public String frontRightName = "rightFront";
    public String backLeftName = "leftRear";
    public String backRightName = "rightRear";

    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;

    public MotorEx[] motors;

    public MecanumDriverControlled driverControlled;

    @Override
    public void onInit() {
        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);





        ClawIntake.INSTANCE.open().invoke();
        RotationIntake.INSTANCE.up().invoke();
        RotationDeposit.INSTANCE.down().invoke();
        ClawDeposit.INSTANCE.close().invoke();
        Intake.INSTANCE.retract().invoke();
        Deposit.INSTANCE.getReady().invoke();
        ClawDeposit.INSTANCE.flipDown().invoke();
        ClawIntake.INSTANCE.rotateHorizontal().invoke();



        // Change your motor directions to suit your robot.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        motors = new MotorEx[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};


    }
    @Override
    public void onStartButtonPressed() {

        // TODO:
        //       change starting pos of deposit(add a hard stop to intake rotation)
        //       figure out how to slow drive down with bumper
        //       do arm scoring - one command for high basket and another for high spec.
        //       possibly swap out torque servos for speed if you can find...




        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());

        driverControlled.invoke();

        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(
                ()-> new InstantCommand(
                        () -> {
                            driverControlled.setScalar(0.5);
                        })
        );

        gamepadManager.getGamepad1().getRightBumper().setReleasedCommand(
                ()-> new InstantCommand(
                        () -> {
                            driverControlled.setScalar(1);
                        })
        );

        ClawIntake.INSTANCE.open().invoke();
        RotationIntake.INSTANCE.up().invoke();
        RotationDeposit.INSTANCE.down().invoke();
        ClawDeposit.INSTANCE.open().invoke();
        Intake.INSTANCE.retract().invoke();
        Deposit.INSTANCE.getReady().invoke();
        ClawDeposit.INSTANCE.flipUp().invoke();
        ClawIntake.INSTANCE.rotateHorizontal().invoke();


        //a button
        //handles the transfer
        {
            gamepadManager.getGamepad2().getA().setPressedCommand(
                    () -> new SequentialGroup(
                            RotationDeposit.INSTANCE.down(),
                            Deposit.INSTANCE.getReady(),
                            ClawDeposit.INSTANCE.flipUp(),
                            ClawDeposit.INSTANCE.open(),
                            ClawIntake.INSTANCE.close().thenWait(0.2),
                            RotationIntake.INSTANCE.up().thenWait(0.2),
                            Intake.INSTANCE.retract().thenWait(0.5),
                            Deposit.INSTANCE.toTransfer(),
                            ClawDeposit.INSTANCE.close().thenWait(0.1),
                            ClawIntake.INSTANCE.open().thenWait(0.1),
                            new ParallelGroup(
                                    Deposit.INSTANCE.toScore(),
                                    Slides.INSTANCE.toSample()
                            )
                    )
            );
        }

        //left trigger
        //extend intake, open claw, rotate intake down
        {
            gamepadManager.getGamepad2().getLeftTrigger().setPressedCommand(
                    value -> new SequentialGroup(
                            //extend linage
                            ClawIntake.INSTANCE.open(),
                            RotationIntake.INSTANCE.down(),
                            Intake.INSTANCE.extend()


                    )
            );
        }

        //right trigger
        //close claw, rotate intake up, retract intake
        {
            gamepadManager.getGamepad2().getRightTrigger().setPressedCommand(
                    value -> new SequentialGroup(
                            ClawIntake.INSTANCE.close().thenWait(0.1),
                            RotationIntake.INSTANCE.up(),
                            Intake.INSTANCE.retract()

                    )
            );
        }


        //y button
        //extend intake, rotate intake down, open claw
        //used for depositing samples in hp
        {
            gamepadManager.getGamepad2().getY().setPressedCommand(
                    () -> new SequentialGroup(
                           RotationDeposit.INSTANCE.up().thenWait(0.5),
                            ClawDeposit.INSTANCE.open().thenWait(0.3),
                            RotationDeposit.INSTANCE.down(),
                            Deposit.INSTANCE.getReady(),
                            Slides.INSTANCE.toResting()


                    )
            );
        }

        //b button
        //pickup samples
        {
            gamepadManager.getGamepad2().getB().setPressedCommand(
                    () -> new SequentialGroup(
                            RotationDeposit.INSTANCE.up().thenWait(0.1),
                            ClawDeposit.INSTANCE.flipUp(),
                            ClawDeposit.INSTANCE.open(),
                            Deposit.INSTANCE.toPickUp()
                    )
            );

            gamepadManager.getGamepad2().getX().setPressedCommand(
                    () -> new SequentialGroup(
                            ClawDeposit.INSTANCE.close().thenWait(0.1),
                            Deposit.INSTANCE.toScore(),
                            Slides.INSTANCE.toChamber()
                    )
            );
        }

        {
            gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                    () -> new SequentialGroup(
                            ClawIntake.INSTANCE.rotateVertical()
                    )
            );
            gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(
                    () -> new SequentialGroup(
                            ClawIntake.INSTANCE.rotateHorizontal()
                    )
            );
        }


    }
    @Override
    public void onUpdate(){


    }
}