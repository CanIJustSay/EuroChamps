package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

@Config
public class Deposit extends Subsystem {
    // BOILERPLATE
    public static final Deposit INSTANCE = new Deposit();
    private Deposit() { }


    // USER CODE
    public MotorEx deposit;

    public static double p = 0.009;
    public static double i = 0;

    public static double d = 0.001;

    public static double target = 0;






    private double calculateFeedforward() {
        return Math.cos(Math.toRadians(controller.getTarget() / (537.6 / 360) )) * 0.005;
    }
    public PIDFController controller =
            new PIDFController
                    (0.009, 0, 0.001,
                            pos -> calculateFeedforward(),20);

    public String name = "deposit";

    @Override
    public void initialize() {
        deposit = new MotorEx(name);
        deposit.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deposit.resetEncoder();
    }

    public Command getReady() {
        return new RunToPosition(deposit, // MOTOR TO MOVE
                (1200*0.37730685565), // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toTransfer() {
        return new RunToPosition(deposit, // MOTOR TO MOVE
                (1420*0.37730685565), // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toScore() {
        return new RunToPosition(deposit, // MOTOR TO MOVE
                (300.0*0.37730685565), // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toPickUp() {
        return new RunToPosition(deposit, // MOTOR TO MOVE
                (1600.0*0.37730685565), // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(deposit, controller, this);
    }

    @Override
    public void periodic(){
//        controller.setKP(p);
//        controller.setKI(i);
//        controller.setKD(d);
//
//
//        deposit.setPower(controller.calculate(deposit.getCurrentPosition(),target));
//
//        OpModeData.telemetry.addData("target",target);
//        OpModeData.telemetry.addData("pos",deposit.getCurrentPosition());
//

    }

}