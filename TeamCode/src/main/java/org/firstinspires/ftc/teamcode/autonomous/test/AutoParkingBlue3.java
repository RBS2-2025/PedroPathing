
package org.firstinspires.ftc.teamcode.autonomous.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.STATES;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogicsOld;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Parking 6", group = "Autonomous")
@Configurable // Panels
public class AutoParkingBlue3 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Robot robot;
    TaskLogicsOld task;

    //골대 시작 각도: -36도(블루)

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25, 129, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        this.robot = new Robot(hardwareMap,true);
        this.task = new TaskLogicsOld(this.robot,true);
        this.task.start();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        task.loop();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




//#region paths








    public static class Paths {
        public PathChain StartOuttake;
        public PathChain Intake;
        public PathChain IntakeEnd;
        public PathChain Open;
        public PathChain Outtake2;
        public PathChain Parking;

        public Paths(Follower follower) {
            StartOuttake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(25.000, 129.000),
                                    new Pose(37.500, 111.500),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(60.730, 79.791),
                                    new Pose(48.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 60.000),

                                    new Pose(18.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Open = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 60.000),

                                    new Pose(14.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(165))

                    .build();

            Outtake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.000, 61.000),
                                    new Pose(50.443, 70.857),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(135))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();
        }
    }








//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                delay(0.5);
                followPath(paths.StartOuttake,1,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(task.getTime(STATES.OUTTAKE) > 2){
                        task.setBlockState(BLOCKSTATE.OPEN);
                        delay(0.1);
                    }
                    if(task.getTime(STATES.OUTTAKE) > 5){
                        task.setOuttakeState(OUTTAKESTATE.REST);
                        task.setBlockState(BLOCKSTATE.BLOCK);
                        delay(0.1);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    followPath(paths.Intake,1,true);
                    task.setIntakeState(INTAKESTATE.FEED);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    followPath(paths.IntakeEnd,0.4,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.STOP);
                    followPath(paths.Outtake2,1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(task.getTime(STATES.OUTTAKE) > 2){
                        task.setBlockState(BLOCKSTATE.OPEN);
                        delay(0.1);
                    }
                    if(task.getTime(STATES.OUTTAKE) > 5){
                        task.setOuttakeState(OUTTAKESTATE.REST);
                        task.setBlockState(BLOCKSTATE.BLOCK);
                        delay(0.1);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    followPath(paths.Parking,1,true);
                    setPathState(-1);
                }
                break;

            case -1:
                //IDLE
                panelsTelemetry.addData("status: ","complete");
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
    void delay(double t){
        Timer waitTimer = new Timer();
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTimeSeconds() <= t){
            panelsTelemetry.addData("status: ", "waiting");
            panelsTelemetry.update();
        }
        return;
    }
}