package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Movement.ActionManaging_main.blockpos;
import static org.firstinspires.ftc.teamcode.Movement.ActionManaging_main.unblockpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging_main;
import org.firstinspires.ftc.teamcode.Movement.IMU_Driving;
import org.firstinspires.ftc.teamcode.Movement.Mecanum_Driving;
import org.firstinspires.ftc.teamcode.utils.PanelHelper;

@Configurable
@TeleOp
public class main3 extends LinearOpMode {
    DcMotor intake, fl, fr, rl, rr;
    DcMotorEx outtake, outtake2;
    IMU imu;
    Servo blocker, push;
    PanelHelper p;
    private ActionManaging_main action;
    Mecanum_Driving drive;

    private boolean Outtake_wasPressed = false;
    private boolean Intake_wasPressed = false;
    private boolean IntakeR_wasPressed = false;
    private boolean isOuttakeOn = false;
    private boolean lastY = false;

    public static double intakepower = 1.0;

    public static double pushpos = 0.547; //0.57
    public static double unpushpos = 0.51; //0.57


    public static double SHOOTING_VELOCITY = 1200;
    public static double PREHEAT_VELOCITY  = 700;
    public static double SHOOTING_VELOCITY2 = 1450;
    public static double PREHEAT_VELOCITY2  = 700;

    public static double curTargetVelocity = PREHEAT_VELOCITY;
    public static double curTargetVelocity2 = PREHEAT_VELOCITY2;

    // 0.4 /  0.75  !

    // 0.4 / 0.8
    @Override
    public void runOpMode() {
        waitForStart();

        initialize();

        drive = new Mecanum_Driving(hardwareMap);

        action = new ActionManaging_main(outtake,outtake2,intake,blocker,push,this.hardwareMap);

        while (opModeIsActive()) {
            double rt = gamepad1.right_trigger;

            double minSpeed = 0.3;
            double speed = 1.0 - (rt * (1.0 - minSpeed));

            drive.setSpeed(speed);

            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            outtakeM();
            intake();
            intakeR();
            blocker();
            push();

            telemetry.addData("current", curTargetVelocity);
            telemetry.addData("current2", curTargetVelocity2);

            telemetry.addData("velocity", outtake.getVelocity());
            telemetry.addData("error", curTargetVelocity-outtake.getVelocity());
            telemetry.addData("velocity2", outtake2.getVelocity());
            telemetry.addData("error2", curTargetVelocity2-outtake2.getVelocity());

            if (gamepad2.dpad_up){
                curTargetVelocity = SHOOTING_VELOCITY;
                curTargetVelocity2 = SHOOTING_VELOCITY2;
            }
            if (gamepad2.dpad_down){
                curTargetVelocity = PREHEAT_VELOCITY;
                curTargetVelocity2 = PREHEAT_VELOCITY2;
            }
            telemetry.update();

        }

    }
    void outtakeM() {
        if (gamepad2.y && !lastY) {
            isOuttakeOn = !isOuttakeOn;
        }
        lastY = gamepad2.y;

        if (isOuttakeOn) {
            action.outtake(curTargetVelocity,curTargetVelocity2);
        } else {
            action.outtake_stop();
        }
    }
    void intake(){
        if (gamepad2.a) {
            action.intake(intakepower);
            if (!Intake_wasPressed) Intake_wasPressed = true;
        }

        if (!gamepad2.a && Intake_wasPressed) {
            action.intake_stop();
            Intake_wasPressed = false;
        }
    }


    void intakeR(){
        if (gamepad2.x) {
            action.intake(intakepower*-0.5);
            if (!IntakeR_wasPressed) IntakeR_wasPressed = true;
        }
        if (!gamepad2.x && IntakeR_wasPressed) {
            action.intake_stop();
            IntakeR_wasPressed = false;
        }
    }

    void blocker(){
        if (gamepad2.right_trigger > 0.4){
            blocker.setPosition(blockpos);
        }else{
            blocker.setPosition(unblockpos);
        }
    }

    void push(){
        if (gamepad2.left_trigger > 0.4){
            push.setPosition(pushpos);
        }else{
            push.setPosition(unpushpos);
        }
    }

    void initialize(){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        rl = hardwareMap.dcMotor.get("rl");
        rr = hardwareMap.dcMotor.get("rr");

        imu = hardwareMap.get(IMU.class,"imu");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = hardwareMap.get(DcMotorEx.class,"outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class,"outtake2");
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        blocker = hardwareMap.get(Servo.class,"blocker");
        blocker.setDirection(Servo.Direction.REVERSE);

        push = hardwareMap.get(Servo.class,"push");



    }
}
