package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging_main;

@Configurable
@Autonomous
public class auto extends LinearOpMode {

    private DcMotor fl, rl, fr, rr,intake;
    private DcMotorEx outtake,outtake2;
    private Servo blocker,push;
    private ElapsedTime runtime = new ElapsedTime();
    ActionManaging_main action;

    private enum State {
        START_DELAY,

        WARMUP,         // 1. 예열
        SHOOT_RPM,      // 2. 발사 RPM
        INTAKE_ONLY,    // 3. 인테이크 2초
        FEED,           // 4. blocker + intake
        PUSH,           // 5. pusher
        MOVE_FORWARD,   // 6. 전진

        STOP,
        DONE
    }


    private State currentState = State.START_DELAY;

    public static double START_WAIT_SECONDS = 0;
    public static double MOVE_FORWARD_SECONDS = 2.3;
    public static double FORWARD_POWER = 0.3;

    public static double PREPARE_TIME = 4;
    public static double SHOOTING_TIME = 4;

    public static double INTAKE_SHOOT = 4;
    public static double BLOCKING_INTAKE = 2;
    public static double PUSHER_TIME = 1;


    public static double pushpos = 0.56; //0.57
    public static double unpushpos = 0.51; //0.57
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        rl  = hardwareMap.get(DcMotor.class, "rl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        telemetry.addData("Status", "Ready to run (Time Based)");
        telemetry.addData(">", String.format("Will move for %.1f seconds", MOVE_FORWARD_SECONDS));
        telemetry.update();


        action = new ActionManaging_main(
                outtake,
                outtake2,
                intake,
                blocker,
                push,
                hardwareMap
        );

        waitForStart();

        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive() && currentState != State.DONE) {

            switch (currentState) {

                case START_DELAY:
                    if (runtime.seconds() >= START_WAIT_SECONDS) {
                        runtime.reset();
                        currentState = State.WARMUP;
                    }
                    break;

                case WARMUP: // 1. 예열
                    action.outtake(700, 700);
                    if (runtime.seconds() >= PREPARE_TIME) {
                        runtime.reset();
                        currentState = State.SHOOT_RPM;
                    }
                    break;

                case SHOOT_RPM: // 2. 발사 RPM 3초 대기
                    action.outtake(1200, 1450);
                    if (runtime.seconds() >= SHOOTING_TIME) {
                        runtime.reset();
                        currentState = State.INTAKE_ONLY;
                    }
                    break;

                case INTAKE_ONLY: // 3. 인테이크 2초
                    action.intake(1);
                    if (runtime.seconds() >= INTAKE_SHOOT) {
                        runtime.reset();
                        currentState = State.FEED;
                    }
                    break;

                case FEED: // 4. blocker + intake
                    action.intake(1);
                    if (runtime.seconds() >= BLOCKING_INTAKE) {
                        runtime.reset();
                        currentState = State.PUSH;
                    }
                    break;

                case PUSH: // 5. pusher
                    action.push.setPosition(pushpos);
                    if (runtime.seconds() >= PUSHER_TIME) {
                        action.push.setPosition(unpushpos);
                        runtime.reset();
                        currentState = State.MOVE_FORWARD;
                    }
                    break;

                case MOVE_FORWARD: // 6. 전진
                    setAllPower(FORWARD_POWER*-1);
                    if (runtime.seconds() >= MOVE_FORWARD_SECONDS) {
                        setAllPower(0);
                        runtime.reset();
                        currentState = State.STOP;
                    }
                    break;

                case STOP:
                    action.intake_stop();
                    action.outtake_stop();
                    currentState = State.DONE;
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();
        }


        setAllPower(0);

        while (opModeIsActive()) {
            telemetry.addData("State", "4. Done");
            telemetry.update();
        }
    }

    private void setAllPower(double power) {
        fl.setPower(power);
        rl.setPower(power);
        fr.setPower(power);
        rr.setPower(power);
    }
}
