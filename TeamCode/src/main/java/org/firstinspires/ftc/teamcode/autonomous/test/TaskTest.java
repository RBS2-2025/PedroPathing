package org.firstinspires.ftc.teamcode.autonomous.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogicsOld;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TaskTEST", group = "Autonomous")
@Configurable // telemetry
public class TaskTest extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    public static int useTask = 1;
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Robot robot;
    TaskLogicsOld task;

    //골대 시작 각도: -36도(블루)

    @Override
    public void init() {
//        telemetry = new telemetryHelper(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        robot = new Robot(hardwareMap,true);
        task = new TaskLogicsOld(robot,true);

//        telemetry.deb("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        task.loop(); // do task

        // Log values to telemetry and Driver Stationtelemetry
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//        telemetry.update();
    }



    public static class Paths {
        public PathChain Home;
        public PathChain Shoot;
        public PathChain Right;
        public PathChain Left;
        public PathChain Back;
        public PathChain Straight;
        public PathChain LeftLong;
        public PathChain RightLong;
        public Paths(Follower follower) {
            Home = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Right = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(96.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Left = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 72.000),

                                    new Pose(48.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Back = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 72.000),

                                    new Pose(72.000, 96.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Straight = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 96.000),

                                    new Pose(72.000, 48.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();
            LeftLong = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(130.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                    .build();
            RightLong = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(20, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                    .build();
        }
    }




    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                setPathState(useTask);
                break;
            case 1:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    setPathState(-1);
                }
                break;
            case 2:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.DISCHARGE);
                    setPathState(-1);
                }
                break;
            case 3:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    setPathState(-3);
                }
                break;
            case -3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    task.setOuttakeState(OUTTAKESTATE.REST);
                    setPathState(-1);
                }
                break;
            case 4:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setBlockState(BLOCKSTATE.BLOCK);
                    setPathState(-4);
                }
                break;
            case -4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    task.setBlockState(BLOCKSTATE.OPEN);
                    setPathState(-1);
                }
                break;
            case 5:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setTrackingState(TRACKINGSTATE.TRACK);
                    setPathState(-5);
                }
                break;
            case -5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    task.setTrackingState(TRACKINGSTATE.STOP);
                    setPathState(-1);
                }
                break;
            case 6:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setTrackingState(TRACKINGSTATE.RESET);
                    setPathState(-1);
                }
                break;
            case 7:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Straight,1,true);
                    setPathState(-7);
                }
                break;
            case 8:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Left,1,true);
                    setPathState(-7);
                }
                break;
            case 9:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Right,1,true);
                    setPathState(-7);
                }
                break;
            case 10:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Back,1,true);
                    setPathState(-7);
                }
                break;
            case 11:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Shoot,1,true);
                    setPathState(-7);
                }
                break;
            case -7:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(-1);
                }
                break;
            case 12:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.LeftLong,1,true);
                    setPathState(-12);
                }
                break;
            case -12:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(12);
                }
                break;
            case 13:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.LeftLong,1,true);
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    setPathState(-13);
                }
                break;
            case -13:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(13);
                }
                break;
            case 14:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.LeftLong,1,true);
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    setPathState(-14);
                }
                break;
            case -14:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(14);
                }
                break;
            case 15:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.LeftLong,1,true);
                    task.setTrackingState(TRACKINGSTATE.TRACK);
                    setPathState(-15);
                }
                break;
            case -15:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(15);
                }
                break;
            case 16:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.LeftLong,1,true);
                    task.setOuttakeState(OUTTAKESTATE.PREHEAT);
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    setPathState(-16);
                }
                break;
            case -16:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    task.setIntakeState(INTAKESTATE.STOP);
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(16);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.RightLong,1,true);
                    setPathState(-17);
                }
                break;
            case -17:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(17);
                }
                break;
            case -1:
                //IDLE
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.STOP);
                    task.setBlockState(BLOCKSTATE.OPEN);
                    task.setOuttakeState(OUTTAKESTATE.REST);
                    task.setTrackingState(TRACKINGSTATE.STOP);
                    telemetry.addData("status: ","complete");
                }
                break;

        }


    }
    void setPathState(int state){
        pathState = state;
    }
    void followPath(PathChain path,double maxPower,boolean holdEnd){
        pathTimer.resetTimer();
        follower.followPath(path,maxPower,holdEnd);
    }
}