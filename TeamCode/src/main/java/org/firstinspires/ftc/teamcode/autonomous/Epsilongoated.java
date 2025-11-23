package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Epsilongoated extends OpMode {
    private Timer pathtimer;
    private Timer OpmodeTimer;
    public enum Pathstate {
        //start and end POS
        //Drive- movement
        //shoot
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        Stafretopickspike1

    }
    Pathstate pathstate;
    private final Pose startPose = new Pose(21.75700934579439,125.83177570093459,Math.toRadians(144));
    private final Pose shootPose = new Pose(44.41121495327103,104.5233644859813,Math.toRadians(144));
    private final Pose strafePose1 =new Pose(44.41121495327103,84.33644859813086,Math.toRadians(180));
    private PathChain drivestartpostoshootpos,Stafretopickspike1;


    public void buildPaths(){
        //put in coords for start pose > ending pose
        drivestartpostoshootpos =follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        Stafretopickspike1 =follower.pathBuilder()
                .addPath(new BezierLine(shootPose,strafePose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafePose1.getHeading())
                .build();

    }
    public void statePathupdate(){
        switch(pathstate){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(drivestartpostoshootpos,true);
                setPathstate(Pathstate.SHOOT_PRELOAD);// reste the timer and make new state
                break;
            case SHOOT_PRELOAD:
                //cheak is follower done its path ?
                if (!follower.isBusy()&& pathtimer.getElapsedTimeSeconds() >5 ){
                    //TODO add logic to flywheel shooter
                    follower.followPath(Stafretopickspike1, true);
                    setPathstate(Pathstate.Stafretopickspike1);
                    telemetry.addLine("done path 1");
                }
                break;
            case Stafretopickspike1:
                if (!follower.isBusy()){
                    telemetry.addLine("going to pick up spick mark");
                }
            default:
                telemetry.addLine("no state commanded");
                break;
        }
    }
    public void setPathstate(Pathstate newstate ){
        pathstate= newstate;
        pathtimer.resetTimer();
    }



    @Override
    public void init(){
        pathstate= Pathstate.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        OpmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechaniams
        buildPaths();
        follower.setPose(startPose);

    }
    public void start(){
        OpmodeTimer.resetTimer();
        setPathstate(pathstate);

    }
    @Override
    public void loop(){
        follower.update();
        statePathupdate();
        telemetry.addData("path state", pathstate.toString());
        telemetry.addData("x", follower.getPose().getPose());
        telemetry.addData("y", follower.getPose().getPose());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path time", pathtimer.getElapsedTimeSeconds());
    }
}