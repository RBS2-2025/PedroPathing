package org.firstinspires.ftc.teamcode.movement;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class ActionManaging {


    //    Servo lifting;
    public DcMotor Turret_R , IntakeDc;
    public DcMotorEx Turret_S;
    private static final double PREHEAT_VELOCITY = 1000;
    private static final double SHOOT_VELOCITY   = 1500;
    private double currentTargetVelocity = 0;

    private static final double P = 450;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double BASE_F = 15;

    private static final double NOMINAL_VOLTAGE = 12.0;

    public ActionManaging(DcMotorEx Turret_S, DcMotor Turret_R, DcMotor IntakeDc) {
        this.Turret_S = Turret_S;
        this.Turret_R = Turret_R;
        this.IntakeDc = IntakeDc;
    }

    public void intake(double power_in) {
        IntakeDc.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeDc.setPower(power_in);
    }

    public void intake_stop() {
        IntakeDc.setPower(0);
    }
    public void intake_r() {
        IntakeDc.setPower(-1);
    }

    public void updateFlywheelPIDF(double batteryVoltage) {
        double compensatedF = BASE_F * (NOMINAL_VOLTAGE / batteryVoltage);

        PIDFCoefficients pidf = new PIDFCoefficients(
                P, I, D, compensatedF
        );



        Turret_S.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
    }
    public void preheat() {
        currentTargetVelocity = PREHEAT_VELOCITY;
        Turret_S.setVelocity(currentTargetVelocity);
    }
    public void outtake() {
        currentTargetVelocity = SHOOT_VELOCITY;
        Turret_S.setVelocity(SHOOT_VELOCITY);
    }

    public void outtake_stop(){
        Turret_S.setPower(0);
        IntakeDc.setPower(0);
    }


}