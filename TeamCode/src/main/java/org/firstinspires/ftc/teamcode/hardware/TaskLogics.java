package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKEPOSITION;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.STATES;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;

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
    double SHOOTING_VELOCITY_NEAR = 0;
    double SHOOTING_VELOCITY_FAR = 0;
    double shooting_target_velocity;
    double PREHEAT_VELOCITY = 0;
    PIDFCoefficients outtakePIDF_near = new PIDFCoefficients(0,0,0,0);
    PIDFCoefficients outakePIDF_far = new PIDFCoefficients(0,0,0,0);
//endregion outtake

//region block
    double OPEN_POSITION = 0;
    double BLOCK_POSITION = 1;
//endregion block

    Map<STATES,ElapsedTime> timers = new EnumMap<>(STATES.class);


    public TaskLogics(Robot robot){
        this.intaker = robot.intaker;
        this.shooter = robot.shooter;
        this.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,this.outtakePIDF_near);
        this.blocker = robot.blocker;
        this.tracker = robot.tracker;
        for (STATES state : STATES.values()) {
            timers.put(state, new ElapsedTime());
        }
    }
    public void start(){
        this.setIntakeState(INTAKESTATE.STOP);
        this.setOuttakeState(OUTTAKESTATE.PREHEAT);
        this.setTrackingState(TRACKINGSTATE.RESET);
        this.setBlockState(BLOCKSTATE.BLOCK);


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
        }
        switch (trackingState){
            case IDLE:
                //IDLE
                break;
            case TRACE:
                track();
                break;
            case RESET:
                track_reset();
                break;
            case STOP:
                track_stop();
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
    }
    void outtake_rest(){
        this.shooter.setPower(0);
        if(this.timers.get(STATES.OUTTAKE).time(TimeUnit.SECONDS) > 2){
            this.setOuttakeState(OUTTAKESTATE.PREHEAT);
        }
    }
    public void changeOuttakePosition(OUTTAKEPOSITION targetPosition){
        switch (targetPosition){
            case NEAR:
                this.shooting_target_velocity = this.SHOOTING_VELOCITY_NEAR;
                break;
            case FAR:
                this.shooting_target_velocity = this.SHOOTING_VELOCITY_FAR;
                break;
        }
    }
//endregion outtake

//region track
    void track(){
        this.setTrackingState(TRACKINGSTATE.IDLE);
    }
    void track_reset(){
        this.setTrackingState(TRACKINGSTATE.IDLE);
    }
    void track_stop(){
        this.setTrackingState(TRACKINGSTATE.IDLE);
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