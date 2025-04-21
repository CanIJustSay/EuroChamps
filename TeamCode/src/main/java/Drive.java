import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Drive")
public class Drive extends OpMode {

    public static double tick_in_degrees = 537.6 / 360;

    public int transferTimer = 0;
    public int linkageTimer = 0;


    private PIDController depositController;
    public static double depositP = 0.01, depositI = 0, depositD = 0.001;
    public static double depositF = 0.005;
    public static double depositTarget = 0;

    int depositPos;
    double depositPID;
    double depositFF;
    double depositPower;

    private PIDController linkageController;
    public static double linkageP = 0.0, linkageI = 0, linkageD = 0.0;
    public static double linkageF = 0.0;
    public static double linkageTarget = 0;

    int linkagePos;
    double linkagePID;
    double linkageFF;
    double linkagePower;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx deposit;
    private DcMotorEx linkage;


    private Servo intakeLeft;
    private Servo intakeRight;

    private Servo depoLeft;
    private Servo depoRight;

    private Servo intakeClaw;
    private Servo depositClaw;
    private Servo rotateIntake;
    private Servo rotateDeposit;

    private double intakeLeftPos = 0;
    private double intakeRightPos = 1;

    private double depoRightPos = 1;
    private double depoLeftPos = 0;

    private double close = 0;
    private double open = 0.25;

    private double horizontal = 0.5;

    private boolean canScore;

    private DcMotorEx rightArm;

    enum IntakeState {
        RESTING,
        SEARCHING,
        PICKUP,
        TRANSFER,
        TRANSFERRING
    }

    private IntakeState currentIntakeState;
    private boolean lastXButtonState = false;

    // Variables for the three-button toggles in RESTING
    private boolean lastAButtonState = false;
    private boolean lastBButtonState = false;
    private boolean lastYButtonState = false;
    private boolean isAToggled = false; // Toggled by A button
    private boolean isBToggled = false; // Toggled by B button
    private boolean isYToggled = false; // Toggled by Y button (also resets others)

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        intakeRight = hardwareMap.get(Servo.class,"intakeRight");

        depoLeft = hardwareMap.get(Servo.class,"depoLeft");
        depoRight = hardwareMap.get(Servo.class,"depoRight");

        intakeClaw = hardwareMap.get(Servo.class,"intakeClaw");
        depositClaw = hardwareMap.get(Servo.class,"depositClaw");

        rotateIntake = hardwareMap.get(Servo.class,"rotateIntake");
        rotateDeposit = hardwareMap.get(Servo.class,"rotateDeposit");

        deposit = hardwareMap.get(DcMotorEx.class,"deposit");

        deposit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deposit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linkage = hardwareMap.get(DcMotorEx.class,"linkage");

        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentIntakeState = IntakeState.RESTING;

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        depositController = new PIDController(depositP, depositI, depositD);
        linkageController = new PIDController(linkageP, linkageI, linkageD);
        rightArm = hardwareMap.get(DcMotorEx.class,"rightArm");

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {
        // Driver 1: Drivetrain control


        // Handle X button for IntakeState cycling


        // Handle the current IntakeState (button logic inside)

        depositController.setPID(depositP,depositI,depositD);

        depositPos = deposit.getCurrentPosition();
        depositPID = depositController.calculate(depositPos, depositTarget);
        depositFF = Math.cos(Math.toRadians(depositTarget / tick_in_degrees)) * depositF;
        depositPower = depositPID + depositFF;

        deposit.setPower(depositPower);






        // Set servo positions





        // Telemetry
        telemetry.addData("pos", deposit.getCurrentPosition());
        telemetry.addData("target",depositTarget);


        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "OpMode Stopped");
    }



}