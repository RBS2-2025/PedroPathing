package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging_main;
import org.firstinspires.ftc.teamcode.Movement.IMUDriving;

@Configurable
@TeleOp
public class main extends LinearOpMode {
    DcMotor outtake, outtake2, intake;
    Servo blocker;
    private ActionManaging_main action;
    private boolean Outtake_wasPressed = false;
    private boolean Intake_wasPressed = false;
    private boolean IntakeR_wasPressed = false;
    private boolean Blocking_wasPressed = false;

    public static double power = 0.45;
    public static double power2 = 0.85;
    public static double intakepower = 1.0;

    public static double servopos = 0;
    @Override
    public void runOpMode() {
        waitForStart();

        initialize();

        action = new ActionManaging_main(outtake,outtake2,intake);

        IMUDriving imuDriving = new IMUDriving(hardwareMap,telemetry,gamepad1);

        while (opModeIsActive()) {
            imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);

            outtake();
            intake();
            intakeR();
            blocking();
        }

    }
    void outtake(){
        if (gamepad2.a) {
            action.outtake(power,power2);
            if (!Outtake_wasPressed) Outtake_wasPressed = true;
        }

        if (!gamepad2.a && Outtake_wasPressed) {
            action.outtake_stop();
            Outtake_wasPressed = false;
        }
    }

    void intake(){
        if (gamepad2.b) {
            action.intake(intakepower);
            if (!Intake_wasPressed) Intake_wasPressed = true;
        }

        if (!gamepad2.b && Intake_wasPressed) {
            action.intake_stop();
            Intake_wasPressed = false;
        }
    }

    void intakeR(){
        if (gamepad2.x) {
            action.intake(intakepower*-1);
            if (!IntakeR_wasPressed) IntakeR_wasPressed = true;
        }
        if (!gamepad2.x && IntakeR_wasPressed) {
            action.intake_stop();
            IntakeR_wasPressed = false;
        }
    }

    void blocking(){
        if (gamepad2.y){
            blocker.setPosition(servopos);
            if (!Blocking_wasPressed) Blocking_wasPressed = true;
        }
        if (!gamepad2.y && Blocking_wasPressed){
            blocker.setPosition(0);
            Blocking_wasPressed = false;
        }
    }

    void initialize(){
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
    }
}
