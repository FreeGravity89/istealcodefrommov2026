package subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class intakeMYBALLS {
    public DcMotor intake;
    public Servo pin;
    private enum IntakeState {
        PPG,
        PGP,
        GPP,
    }
    private IntakeState intakeState;
    private double intakeON = 1;
    private double intakeoFF =0;

}