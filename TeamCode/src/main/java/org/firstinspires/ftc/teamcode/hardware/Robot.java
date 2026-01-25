package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public Limelight3A limelight;
    public IMU imu;
    public DcMotor lf,rf,lr,rr;
    public DcMotorEx intaker, shooter, tracker;
    public Servo blocker;
    public Robot(HardwareMap hardwareMap,boolean useBevel){
        this.lf = hardwareMap.get(DcMotor.class, "lf");
        this.rf = hardwareMap.get(DcMotor.class, "rf");
        this.lr = hardwareMap.get(DcMotor.class, "lr");
        this.rr = hardwareMap.get(DcMotor.class, "rr");
        if(useBevel){
            this.lf.setDirection(DcMotorSimple.Direction.FORWARD);
            this.rf.setDirection(DcMotorSimple.Direction.REVERSE);
            this.lr.setDirection(DcMotorSimple.Direction.FORWARD);
            this.rr.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            this.lf.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rf.setDirection(DcMotorSimple.Direction.FORWARD);
            this.lr.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rr.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        this.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intaker = hardwareMap.get(DcMotorEx.class, "IntakeDc");
        this.shooter = hardwareMap.get(DcMotorEx.class, "Turret_S");
        this.tracker = hardwareMap.get(DcMotorEx.class, "Turret_R");

        this.blocker = hardwareMap.get(Servo.class, "blocker");

        this.limelight = hardwareMap.get(Limelight3A.class,"limelight");
        this.imu = hardwareMap.get(IMU.class,"imu");
    }
}
