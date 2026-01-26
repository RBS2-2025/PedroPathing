package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Vision.TurretControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Configurable
@TeleOp(name = "tracker Test", group = "TEST")
public class TrackingTest extends OpMode {

    public static double speed = 0.03;
    public static double max_speed = 0.1;
    Robot robot;
    DcMotorEx tracker;
    TurretControl control;
    @Override
    public void init() {
        robot = new Robot(hardwareMap,true);
        this.tracker = this.robot.tracker;
        this.control = new TurretControl(robot,telemetry,true,robot.imu);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            control.align(max_speed,false,speed);
        }
    }
}
