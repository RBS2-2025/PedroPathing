package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision.TurretControl;
import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKEPOSITION;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.STATES;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

//TODO OUTTAKE -  PIDF, VELOCITY 넣기
//TODO BLOCK - open, block position 넣기
//TODO TRACKING - 로직 짜기
public class TaskLogics {
//vars
//region hardware
    DcMotorEx intaker, shooter, tracker;
    Servo blocker;
//endregion hardware

//region state
    INTAKESTATE intakeState = INTAKESTATE.IDLE;
    OUTTAKESTATE outtakeState = OUTTAKESTATE.PREHEAT;
    TRACKINGSTATE trackingState = TRACKINGSTATE.IDLE;
    BLOCKSTATE blockState = BLOCKSTATE.IDLE;
//endregion state

//region outtake
    double SHOOTING_VELOCITY_NEAR = 1500;
    double SHOOTING_VELOCITY_FAR = 2000;
    double shooting_target_velocity;
    double PREHEAT_VELOCITY = 500;
    PIDFCoefficients outtakePIDF_near = new PIDFCoefficients(450,0,0,15);
    PIDFCoefficients outakePIDF_far = new PIDFCoefficients(450,0,0,15);
//endregion outtake

//region tracker
    TurretControl turretControl;
//endregion tracker

//region block
    double OPEN_POSITION = 0.56;
    double BLOCK_POSITION = 0.44;
//endregion block

//region panels
    public PanelsHelper panel;
//endregion panels

    Map<STATES,ElapsedTime> timers = new EnumMap<>(STATES.class);


    public TaskLogics(Robot robot, boolean isBlue){
        this.intaker = robot.intaker;
        this.shooter = robot.shooter;
        this.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,this.outtakePIDF_near);
        this.blocker = robot.blocker;
        this.tracker = robot.tracker;
        this.turretControl = new TurretControl(robot, isBlue,robot.imu);
        for (STATES state : STATES.values()) {
            timers.put(state, new ElapsedTime());
        }
    }
    public void start(){
        this.setIntakeState(INTAKESTATE.STOP);
        this.setOuttakeState(OUTTAKESTATE.PREHEAT);
        this.setTrackingState(TRACKINGSTATE.RESET);
        this.setBlockState(BLOCKSTATE.BLOCK);
        changeOuttakePosition(OUTTAKEPOSITION.NEAR);

    } // 기본 상태 (시작 시)

    public void loop(){
        switch (intakeState){
            case IDLE:
                //IDLE
                break;
            case INTAKE:
                intake();
                break;
            case DISCHARGE:
                discharge();
                break;
            case STOP:
                intake_stop();
                break;
        }
        switch (outtakeState){
            case PREHEAT:
                preheat();
                break;
            case SHOOT:
                shoot();
                break;
            case REST:
                outtake_rest();
                break;
            case TOFAR:
                if(this.shooting_target_velocity != this.SHOOTING_VELOCITY_FAR){
                    changeOuttakePosition(OUTTAKEPOSITION.FAR);
                }
                break;
            case TONEAR:
                if(this.shooting_target_velocity != this.SHOOTING_VELOCITY_NEAR){
                    changeOuttakePosition(OUTTAKEPOSITION.NEAR);
                }
                break;
            case TOGGLE_POSITION:
                if(this.shooting_target_velocity != this.SHOOTING_VELOCITY_NEAR){
                    changeOuttakePosition(OUTTAKEPOSITION.NEAR);
                    break;
                }
                if(this.shooting_target_velocity != this.SHOOTING_VELOCITY_FAR){
                    changeOuttakePosition(OUTTAKEPOSITION.FAR);
                    break;
                }
                break;
        }
        switch (trackingState){
            case IDLE:
                //IDLE
                break;
            case TRACK:
                track();
                break;
            case RESET:
                track_reset();
                break;
            case STOP:
                track_stop();
                break;
            case MANUAL_L:
                track_manual(false);
                break;
            case MANUAL_R:
                track_manual(true);
                break;
        }
        switch (blockState){
            case IDLE:
                //IDLE
                break;
            case BLOCK:
                close_block();
                break;
            case OPEN:
                open_block();
                break;
        }
        if(panel != null){
            panel.addData("intake: ", intakeState);
            panel.addData("outtake: ", outtakeState);
            panel.addData("tracking: ",trackingState);
            panel.addData("block: ",blockState);
            panel.addData("shoot velocity: ",shooting_target_velocity);
            panel.update();
        }
    }

//methods
//region manager
    public void setIntakeState(INTAKESTATE state){
        this.intakeState = state;
        this.timers.get(STATES.INTAKE).reset();
    }
    public void setOuttakeState(OUTTAKESTATE state){
        this.outtakeState = state;
        this.timers.get(STATES.OUTTAKE).reset();
    }
    public void setTrackingState(TRACKINGSTATE state){
        this.trackingState = state;
        this.timers.get(STATES.TRACKING).reset();
    }
    public void setBlockState(BLOCKSTATE state){
        this.blockState = state;
        this.timers.get(STATES.BLOCK).reset();
    }

    public double getTime(STATES state){
        return this.timers.get(state).time(TimeUnit.SECONDS);
    }
    public double getTime(STATES state,TimeUnit unit){
        return this.timers.get(state).time(unit);
    }

//endregion manager

//region intake
    void intake(){
        this.intaker.setPower(0.6);
        this.setIntakeState(INTAKESTATE.IDLE);
    }
    void intake_stop(){
        this.intaker.setPower(0);
        this.setIntakeState(INTAKESTATE.IDLE);
    }
    void discharge(){
        this.intaker.setPower(-0.6);
        this.setIntakeState(INTAKESTATE.IDLE);
    }
//endregion intake

//region outtake
    void preheat(){
        this.shooter.setVelocity(PREHEAT_VELOCITY);
    }
    void shoot(){
        this.shooter.setVelocity(this.shooting_target_velocity);
        if(this.timers.get(STATES.OUTTAKE).time(TimeUnit.SECONDS) > 0.5){
            this.setIntakeState(INTAKESTATE.INTAKE);
        }
    }
    void outtake_rest(){
        this.shooter.setPower(0);
        this.setIntakeState(INTAKESTATE.STOP);
        if(this.timers.get(STATES.OUTTAKE).time(TimeUnit.SECONDS) > 2){
            this.setOuttakeState(OUTTAKESTATE.PREHEAT);
        }
    }
    public void changeOuttakePosition(OUTTAKEPOSITION targetPosition){
        switch (targetPosition){
            case NEAR:
                this.shooting_target_velocity = this.SHOOTING_VELOCITY_NEAR;
                this.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,this.outtakePIDF_near);
                break;
            case FAR:
                this.shooting_target_velocity = this.SHOOTING_VELOCITY_FAR;
                this.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,this.outakePIDF_far);
                break;
        }
        this.setOuttakeState(OUTTAKESTATE.PREHEAT);
    }
//endregion outtake

//region track
    void track(){
        turretControl.align(0.4,false,0.1);
    }
    void track_reset(){
        tracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tracker.setTargetPosition(0);
        tracker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tracker.setPower(0.4);//max_power
        if(Math.abs(tracker.getCurrentPosition()) < 10){
            setTrackingState(TRACKINGSTATE.IDLE);
        }
    }
    void track_stop(){
        tracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tracker.setPower(0);
        this.setTrackingState(TRACKINGSTATE.IDLE);
    }
    void track_manual(boolean toRight){
        tracker.setPower((toRight? 0.4: -0.4));
    }
//endregion track

//region block
    void close_block(){
        this.blocker.setPosition(BLOCK_POSITION);
        this.setBlockState(BLOCKSTATE.IDLE);
    }
    void open_block() {
        this.blocker.setPosition(OPEN_POSITION);
        this.setBlockState(BLOCKSTATE.IDLE);
    }
//endregion block
}