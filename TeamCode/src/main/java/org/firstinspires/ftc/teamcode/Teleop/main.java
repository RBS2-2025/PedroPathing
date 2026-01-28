package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.*;
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
public class main extends LinearOpMode {
    DcMotor outtake, outtake2, intake, fl, fr, rl, rr;
    IMU imu;
    Servo blocker, push;
    private ActionManaging_main action;
    PanelHelper p;
    private boolean Outtake_wasPressed = false;
    private boolean Intake_wasPressed = false;
    private boolean IntakeR_wasPressed = false;
    private boolean Blocking_wasPressed = false;
    private boolean UnBlocking_wasPressed = false;

    public static double power = 0.4;
    public static double power2 = 0.75;
    public static double intakepower = 1.0;
    public static double pos = 0.62;
    public static double pushpos = 0; //0.57


    // 0.4 /  0.75  !

    // 0.4 / 0.8
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        p = new PanelHelper(this);

        action = new ActionManaging_main(outtake,outtake2,intake,blocker,push);

        IMU_Driving imuDriving = new IMU_Driving(fl,fr,rl,rr,imu,telemetry,gamepad1);

        blocker.setPosition(pos);
        push.setPosition(pushpos);

        while (opModeIsActive()) {
            p.updateGamepads();

            imuDriving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);

            outtake();
            intake();
            intakeR();

            if (p.gamepad1.right_trigger > 0.5){
                blocker.setPosition(pos);
            }

            if (p.gamepad1.left_trigger > 0.5){
                push.setPosition(pushpos);
            }else{
                push.setPosition(0.5);
            }

        }

    }
    void outtake(){
        if (p.gamepad1.b) {
            action.outtake(power,power2);
            if (!Outtake_wasPressed) Outtake_wasPressed = true;
        }

        if (!p.gamepad1.b && Outtake_wasPressed) {
            action.outtake_stop();
            Outtake_wasPressed = false;
        }
    }

    void intake(){
        if (p.gamepad1.a) {
            action.intake(intakepower);
            if (!Intake_wasPressed) Intake_wasPressed = true;
        }

        if (!p.gamepad1.a && Intake_wasPressed) {
            action.intake_stop();
            Intake_wasPressed = false;
        }
    }

    void intakeR(){
        if (p.gamepad1.x) {
            action.intake(intakepower*-1);
            if (!IntakeR_wasPressed) IntakeR_wasPressed = true;
        }
        if (!p.gamepad1.x && IntakeR_wasPressed) {
            action.intake_stop();
            IntakeR_wasPressed = false;
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
