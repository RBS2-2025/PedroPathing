
package org.firstinspires.ftc.teamcode.autonomous.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogicsOld;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Parking Only", group = "Autonomous")
@Configurable // Panels
public class AutoOnlyParkingRed extends OpMode {
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
        follower.setStartingPose(new Pose(118, 130, Math.toRadians(37)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        this.robot = new Robot(hardwareMap,true);
        this.task = new TaskLogicsOld(this.robot,false);
        this.task.start();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




//#region paths






    public static class Paths {
        public PathChain Parking;

        public Paths(Follower follower) {
            Parking = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(118.000, 130.000),
                                    new Pose(94.557, 104.226),
                                    new Pose(95.583, 71.687)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();
        }
    }






//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                followPath(paths.Parking,1,true);
                setPathState(-1);
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