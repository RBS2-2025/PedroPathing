package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogics;
import org.firstinspires.ftc.teamcode.movement.IMUDriving;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

@TeleOp(name = "TeleOp Red",group = "TeleOp")
public class TeleOpMainRed extends OpMode {
    Robot robot;
    IMUDriving imuDriving;
    PanelsHelper panel;
    TaskLogics task;

    Follower follower;

    boolean rightBumperWasPressed = false;
    boolean leftBumperWasPressed = false;
    boolean rightDpadWasPressed = false;
    boolean leftDpadWasPressed = false;
    boolean leftPressed = false;


    @Override
    public void init() {
        this.follower = Constants.createFollower(hardwareMap);
        /*
        !!!! WARNING START: need tuning !!!!
        !!!! 오토에서 끝난 위치를 넣어야 됨    !!!!
         */
        this.follower.setStartingPose(new Pose(48, 96, 180));
        // !!!! WARNING END !!!!
        this.robot = new Robot(hardwareMap, true);
        this.task = new TaskLogics(this.robot, this.follower, false);

        this.panel = new PanelsHelper(this);
        this.imuDriving = new IMUDriving(hardwareMap,panel,gamepad1);
        this.task.panel = this.panel;
        this.imuDriving.speed=0.7;
    }

    @Override
    public void start(){
        this.task.start();
    }

    @Override
    public void loop() {
        follower.update();

        imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
        this.task.loop();
        this.inputManage();

        // 텔레메트리에 값 찍어보려고 꺼내둠
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    void inputManage(){
        //region a - intake
        if(gamepad2.aWasPressed()){
            task.setIntakeState(INTAKESTATE.INTAKE);
        }
        if(gamepad2.aWasReleased()){
            task.setIntakeState(INTAKESTATE.STOP);
        }
        //endregion a - intake
        //region b - shoot
        if(gamepad2.bWasPressed()){
            task.setBlockState(BLOCKSTATE.OPEN);
            task.setOuttakeState(OUTTAKESTATE.SHOOT);
//            task.setTrackingState(TRACKINGSTATE.TRACK);
        }
        if(gamepad2.bWasReleased()){
            task.setOuttakeState(OUTTAKESTATE.REST);
//            task.setTrackingState(TRACKINGSTATE.RESET);
//            task.setBlockState(BLOCKSTATE.BLOCK);
        }
        //endregion b - shoot
        //region x - discharge
        if(gamepad2.xWasPressed()){
            task.setIntakeState(INTAKESTATE.DISCHARGE);
        }
        if(gamepad2.xWasReleased()){
            task.setIntakeState(INTAKESTATE.STOP);
        }
        //endregion x - discharge
        //region y - change PIDF and velocity
        if(gamepad2.yWasPressed()){
            task.setBlockState(BLOCKSTATE.BLOCK);
        }
        //endregion y - change PIDF and velocity
        //region dpad_l - rotate turret L
        if(gamepad2.dpad_left && !leftDpadWasPressed){
            leftDpadWasPressed = true;
            task.setTrackingState(TRACKINGSTATE.MANUAL_L);
        }
        if(!gamepad2.dpad_left && leftDpadWasPressed){
            leftDpadWasPressed = false;
            task.setTrackingState(TRACKINGSTATE.STOP);
        }
        //endregion dpad_l - rotate turret L
        //region dpad_r - rotate turret R
        if(gamepad2.dpad_right && !rightDpadWasPressed){
            rightDpadWasPressed = true;
            task.setTrackingState(TRACKINGSTATE.MANUAL_R);
        }
        if(!gamepad2.dpad_right && rightDpadWasPressed){
            rightDpadWasPressed = false;
            task.setTrackingState(TRACKINGSTATE.STOP);
        }
        //endregion dpad_r - rotate turret R

        if(gamepad2.left_bumper && !leftPressed){
            leftPressed = true;
            task.setTrackingState(TRACKINGSTATE.TRACK);
        }
        if(!gamepad2.left_bumper && leftPressed){
            leftPressed = false;
            task.setTrackingState(TRACKINGSTATE.STOP);
        }

    }



}
