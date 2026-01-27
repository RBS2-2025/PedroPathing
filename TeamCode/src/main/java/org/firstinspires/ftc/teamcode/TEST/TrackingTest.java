package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Vision.TurretControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

@Configurable
@TeleOp(name = "tracker Test", group = "TEST")
public class TrackingTest extends OpMode {

    public static double speed = 0.03;
    public static double max_speed = 0.1;
    public static boolean reversed = false;
    Robot robot;
    DcMotorEx tracker;
    TurretControl control;
    PanelsHelper panel;
    @Override
    public void init() {
        robot = new Robot(hardwareMap,true);
        this.tracker = this.robot.tracker;
        this.control = new TurretControl(robot,true,robot.imu);
        panel = new PanelsHelper(this);
        if(reversed){
            tracker.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            tracker.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            control.align(max_speed,false,speed);
        }
        if(gamepad1.yWasPressed()){
            tracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(gamepad1.bWasPressed() && !tracker.isBusy()){
            tracker.setTargetPosition(0);
            tracker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tracker.setPower(max_speed);
        }
        if(gamepad1.dpad_left){
            tracker.setPower(-max_speed);
        }
        else if (gamepad1.dpad_right){
            tracker.setPower(max_speed);
        }
        if(!gamepad1.dpad_right && !gamepad1.dpad_left){
            tracker.setPower(0);
        }
        panel.addData("position: ", tracker.getCurrentPosition());
        panel.update();

    }
}
