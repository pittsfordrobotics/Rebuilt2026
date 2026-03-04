package frc.robot.Logging;

import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class ServoLogger extends ClassSpecificLogger <Servo>{
    public ServoLogger(){
        super(Servo.class);
    }

    @Override
    public void update(EpilogueBackend backend, Servo servo){
        //Log motor IDs
        backend.log("Position", servo.getPosition());
    }
}

