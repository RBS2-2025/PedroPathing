package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Movement.ActionManaging_main.blockpos;
import static org.firstinspires.ftc.teamcode.Movement.ActionManaging_main.unblockpos;
import static org.firstinspires.ftc.teamcode.TEST.pidf.SHOOTING_VELOCITY;
import static org.firstinspires.ftc.teamcode.TEST.pidf.SHOOTING_VELOCITY2;

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
import org.firstinspires.ftc.teamcode.utils.PanelHelper;

@Configurable
@TeleOp
public class main2 extends LinearOpMode {
    DcMotor intake, fl, fr, rl, rr;
    DcMotorEx outtake, outtake2;
    IMU imu;
    Servo blocker, push;
    private ActionManaging_main action;
    PanelHelper p;
    private boolean Outtake_wasPressed = false;
    private boolean Intake_wasPressed = false;
    private boolean IntakeR_wasPressed = false;
    private boolean isOuttakeOn = false;
    private boolean lastY = false;

    public static double power = 0.4;
    public static double power2 = 0.75;
    public static double intakepower = 1.0;
    public static double pos = 0.62;
    public static double pushpos = 0.545; //0.57
    public static double unpushpos = 0.51; //0.57


    public static double SHOOTING_VELOCITY = 2100;
    public static double PREHEAT_VELOCITY  = 1000;
    public static double SHOOTING_VELOCITY2 = 2100;
    public static double PREHEAT_VELOCITY2  = 1000;
    public static double curTargetVelocity = PREHEAT_VELOCITY;
    public static double curTargetVelocity2 = PREHEAT_VELOCITY2;

    // 0.4 /  0.75  !

    // 0.4 / 0.8
    @Override
    public void runOpMode() {
        waitForStart();

        initialize();

        IMU_Driving imuDriving = new IMU_Driving(fl,fr,rl,rr,imu,telemetry,gamepad1);

        action = new ActionManaging_main(outtake,outtake2,intake,blocker,push,this.hardwareMap);

        imuDriving.init();
        imuDriving.getYaw();

        while (opModeIsActive()) {

            imuDriving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);
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

            if (gamepad1.right_stick_button){
                curTargetVelocity = SHOOTING_VELOCITY;
                curTargetVelocity2 = SHOOTING_VELOCITY2;
            }
            if (gamepad1.left_bumper){
                curTargetVelocity = PREHEAT_VELOCITY;
                curTargetVelocity2 = PREHEAT_VELOCITY2;
            }
            telemetry.update();

        }

    }
    void outtakeM() {
        if (gamepad1.y && !lastY) {
            isOuttakeOn = !isOuttakeOn;
        }
        lastY = gamepad1.y;

        if (isOuttakeOn) {
            action.outtake(curTargetVelocity,curTargetVelocity2);
        } else {
            action.outtake_stop();
        }
    }
// 16 20
    void intake(){
        if (gamepad1.a) {
            action.intake(intakepower);
            if (!Intake_wasPressed) Intake_wasPressed = true;
        }

        if (!gamepad1.a && Intake_wasPressed) {
            action.intake_stop();
            Intake_wasPressed = false;
        }
    }


    void intakeR(){
        if (gamepad1.x) {
            action.intake(intakepower*-1);
            if (!IntakeR_wasPressed) IntakeR_wasPressed = true;
        }
        if (!gamepad1.x && IntakeR_wasPressed) {
            action.intake_stop();
            IntakeR_wasPressed = false;
        }
    }

    void blocker(){
        if (gamepad1.right_trigger > 0.5){
            blocker.setPosition(blockpos);
        }else{
            blocker.setPosition(unblockpos);
        }
    }

    void push(){
        if (gamepad1.left_trigger > 0.5){
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
