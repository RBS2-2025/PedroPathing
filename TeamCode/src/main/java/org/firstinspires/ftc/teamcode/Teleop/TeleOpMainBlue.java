package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogics;
import org.firstinspires.ftc.teamcode.movement.IMUDriving;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

@TeleOp(name = "TeleOp Blue",group = "TeleOp")
public class TeleOpMainBlue extends OpMode {
    Robot robot;
    IMUDriving imuDriving;
    PanelsHelper panel;
    TaskLogics task;

    boolean rightBumperWasPressed = false;
    boolean leftBumperWasPressed = false;
    boolean rightDpadWasPressed = false;
    boolean leftDpadWasPressed = false;



    @Override
    public void init() {
        this.robot = new Robot(hardwareMap,true);
        this.panel = new PanelsHelper(this);
        this.imuDriving = new IMUDriving(hardwareMap,panel,gamepad1);
        this.task = new TaskLogics(this.robot,true);
        this.task.panel = this.panel;
        this.imuDriving.speed=0.7;
    }

    @Override
    public void start(){
        this.task.start();
    }

    @Override
    public void loop() {
        imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
        this.task.loop();
        this.inputManage();
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
            task.setTrackingState(TRACKINGSTATE.TRACK);
        }
        if(gamepad2.bWasReleased()){
            task.setOuttakeState(OUTTAKESTATE.REST);
            task.setTrackingState(TRACKINGSTATE.RESET);
            task.setBlockState(BLOCKSTATE.BLOCK);
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
//            task.setOuttakeState(OUTTAKESTATE.TOGGLE_POSITION);
            task.setTrackingState(TRACKINGSTATE.TRACK);
        }
        if (gamepad2.yWasReleased()){
            task.setTrackingState(TRACKINGSTATE.RESET);
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

    }



}
