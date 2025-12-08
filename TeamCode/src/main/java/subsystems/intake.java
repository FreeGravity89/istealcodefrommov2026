package subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


public class intake {
    private DcMotor intake;
    private Servo pin;
    private ElapsedTime Time = new ElapsedTime();
    private enum stateInake{
        ON,
        OFF

    }
    private stateInake balls;
    private double BallsON;
    private double BallsOFF;
    private double pinPurp;
    private double pinGREEN;
    public int BallsLEFT;
    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        pin = hardwareMap.get(Servo.class, "pin");
        pin.setDirection(Servo.Direction.REVERSE);
        balls=stateInake.ON;;

    }
    public void update(){
        switch (balls){
            case ON:
                if(Time.seconds()> 1){
                    intake.setPower(BallsON);
                    Time.reset();
                    balls=stateInake.OFF;
                }
                break;
            case OFF:
                if (Time.seconds() > BallsLEFT){
                    intake.setPower(BallsOFF);
                    Time.reset();
                    balls = stateInake.OFF;
                }
                break;
        }
    }
    public void takeIN(int Balls){
        if (balls == stateInake.ON || balls==stateInake.OFF){

        }
        BallsLEFT=Balls;
    }
    public boolean isBusy(){
        return balls != subsystems.intake.stateInake.ON && balls != subsystems.intake.stateInake.OFF;
    }


}

