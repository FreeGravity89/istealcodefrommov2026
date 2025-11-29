package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Epsilongoated extends OpMode {
    private Timer pathtimer;
    private Timer OpmodeTimer;

    public enum servos{
        intake,
    }
    public enum Pathstate {
        //start and end POS
        //Drive- movement
        //shoot
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        Stafretopickspike1,
        CArrytoscore,
        spike2,
        StrafetoPICK2,
        SCORE2,
        PARK,



    }
    private PathChain drivestartpostoshootpos,
            Stafreto1,
            thruoghSpike1,
            fromPickuptoscore,
            STRAFE2,
            Though2Spike,
            FromSpike2TOscore,
            PARRKRRKR;





    Pathstate pathstate;
    private final Pose startPose = new Pose(21.75700934579439,125.83177570093459,Math.toRadians(144));
    private final Pose shootPose = new Pose(44.41121495327103,104.5233644859813,Math.toRadians(144));
    private final Pose strafePose1 =new Pose(44.41121495327103,84.33644859813086,Math.toRadians(180));
    private final Pose intkespike = new Pose(12.560747663551403,83.88785046728971,Math.toRadians(180));
    private final Pose shootSpike1 = new Pose(44.41121495327103,104.5233644859813,Math.toRadians(144));
    private final Pose strafePose2 =new Pose(44.18691588785046,59.88785046728972,Math.toRadians(180));
    private final Pose inatkePike2 =new Pose(12.785046728971961,59.663551401869164,Math.toRadians(180));

    private final Pose shoot2Spike =new Pose(46.205607476635514,104.29906542056075,Math.toRadians(144));
    private final Pose score2ANDPARK =new Pose(20.859813084112147,69.30841121495328, Math.toRadians(270));
    //private final Pose
    //private final Pose



    public void buildPaths(){
        //put in coords for start pose > ending pose
        drivestartpostoshootpos =follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        Stafreto1 =follower.pathBuilder()
                .addPath(new BezierLine(shootPose,strafePose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafePose1.getHeading())
                .build();
        thruoghSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(strafePose1, intkespike))
                .setLinearHeadingInterpolation(strafePose1.getHeading(), intkespike.getHeading())
                .build();
        fromPickuptoscore =follower.pathBuilder()
                .addPath(new BezierLine(intkespike,shootSpike1))
                .setLinearHeadingInterpolation(intkespike.getHeading(),shootSpike1.getHeading())
                .build();
        STRAFE2 =follower.pathBuilder()
                .addPath(new BezierLine(shootSpike1,strafePose2))
                .setLinearHeadingInterpolation(shootSpike1.getHeading(),strafePose2.getHeading())
                .build();
        Though2Spike =follower.pathBuilder()
                .addPath(new BezierLine(strafePose2,inatkePike2))
                .setLinearHeadingInterpolation(strafePose2.getHeading(),inatkePike2.getHeading())
                .build();
        FromSpike2TOscore =follower.pathBuilder()
                .addPath(new BezierLine(inatkePike2,shoot2Spike))
                .setLinearHeadingInterpolation(inatkePike2.getHeading(),shoot2Spike.getHeading())
                .build();
        PARRKRRKR =follower.pathBuilder()
                .addPath(new BezierLine(shoot2Spike,score2ANDPARK))
                .setLinearHeadingInterpolation(shoot2Spike.getHeading(), score2ANDPARK.getHeading())
                .build();


    }
    public void statePathupdate(){
        switch(pathstate){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(drivestartpostoshootpos,true);
                setPathstate(Pathstate.SHOOT_PRELOAD);// reset the timer and make new state
                break;
            case SHOOT_PRELOAD:
                //cheak is follower done its path ?
                if (!follower.isBusy()&& pathtimer.getElapsedTimeSeconds() >3 ){
                    //TODO add logic to flywheel shooter
                    follower.followPath(Stafreto1, true);
                    setPathstate(Pathstate.Stafretopickspike1);
                    telemetry.addLine("lines up with the spike mark");
                }
                break;

            case  Stafretopickspike1:

                if (!follower.isBusy()&& pathtimer.getElapsedTimeSeconds()>1){
                    follower.followPath(thruoghSpike1,false );
                    setPathstate(Pathstate.CArrytoscore);
                    telemetry.addLine("intakeing da balls");

                }
                break;
            case CArrytoscore:
                if(!follower.isBusy()&&pathtimer.getElapsedTimeSeconds()>2){
                    follower.followPath(fromPickuptoscore,true);
                    setPathstate(Pathstate.spike2);
                    telemetry.addLine("shoots the 1st spike marks atleast should!");
                }
                break;
            case spike2:
                if(!follower.isBusy()&&pathtimer.getElapsedTimeSeconds()>2){
                    follower.followPath(STRAFE2,true);
                    setPathstate(Pathstate.StrafetoPICK2);
                }
                break;
            case StrafetoPICK2:
                if (!follower.isBusy()&&pathtimer.getElapsedTimeSeconds()>1){
                    follower.followPath(Though2Spike,false);
                setPathstate(Pathstate.SCORE2);}
                break;
            case SCORE2:
                if(!follower.isBusy()&&pathtimer.getElapsedTimeSeconds()>4) {
                    follower.followPath(FromSpike2TOscore,false);
                    setPathstate(Pathstate.PARK);
                }
                    break;
            case PARK:
                if(!follower.isBusy()&&pathtimer.getElapsedTimeSeconds()>1){
                    follower.followPath(PARRKRRKR,false);
                setPathstate(Pathstate.PARK);}




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
        //ServoEx outakeflipL = new ServoEx(hardwareMap, "outakeflipL");
        //ServoEx outakeflipR = new ServoEx(hardwareMap, "outakeflipR");

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
