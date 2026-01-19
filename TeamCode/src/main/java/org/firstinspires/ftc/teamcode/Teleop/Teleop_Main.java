package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging_new;
import org.firstinspires.ftc.teamcode.Movement.IMU_Driving;
import org.firstinspires.ftc.teamcode.utils.Sorting;

@Configurable
@TeleOp
public class Teleop_Main extends LinearOpMode {
    DcMotor fl, fr, rl, rr;
    IMU imu;
    IMU_Driving imu_driving;
    public NormalizedColorSensor c1, c2, c3;
    public Servo rotateWheel, LiftServo;

    private ActionManaging_new ActionManager;
    private Sorting sorting;

    // 나중에 사용
    private boolean In_wasPressed = false;

    @Override
    public void runOpMode() {
        initialize();

        imu_driving = new IMU_Driving(fl,fr,rl, rr, imu, telemetry,gamepad1);

        ActionManager = new ActionManaging_new(LiftServo, this, rotateWheel);

        sorting = new Sorting(this, ActionManager, c1, c2, c3);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //imu_driving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);

            sort();
            test();

            sorting.updateSensorTelemetry();

            telemetry.update();
        }
    }
    void sort(){
        if (gamepad2.a) {
            sorting.sortForColor("Green");
            ActionManager.sleepFor(0.5);
        }
        if (gamepad2.b) {
            sorting.sortForColor("Purple");
            ActionManager.sleepFor(0.5);
        }
    }

    void test(){
        // 수동 제어
        if (gamepad2.dpad_left) {
            ActionManager.rotateToArtifactPosition(Sorting.test1);
        }
        if (gamepad2.dpad_down) {
            ActionManager.rotateToArtifactPosition(Sorting.test2);
        }
        if (gamepad2.dpad_right) {
            ActionManager.rotateToArtifactPosition(Sorting.test3);
        }
    }

    public void initialize() {
//        fl = hardwareMap.dcMotor.get("fl");
//        fr = hardwareMap.dcMotor.get("fr");
//        rl = hardwareMap.dcMotor.get("rl");
//        rr = hardwareMap.dcMotor.get("rr");
//        imu = hardwareMap.get(IMU.class,"imu");
//
//
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        rl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateWheel = hardwareMap.get(Servo.class, "RotateWheel");
        LiftServo = hardwareMap.get(Servo.class, "LiftServo");

        c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
        c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
        c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");

        c1.setGain(10);
        c2.setGain(10);
        c3.setGain(10);

        rotateWheel.setDirection(Servo.Direction.FORWARD);
        LiftServo.setDirection(Servo.Direction.FORWARD);

        rotateWheel.setPosition(0.5);
        LiftServo.setPosition(0.0);

    }
}