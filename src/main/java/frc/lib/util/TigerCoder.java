package frc.lib.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder; 

public class TigerCoder extends DutyCycleEncoder{

    private int m_Case;
    private double m_CutoffAngle;
    private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
    private double angleOffset = 0;


    public TigerCoder(int channel, int myCase, double cutoffAngle) {
        super(channel);
        this.m_Case = myCase;
        this.m_CutoffAngle = cutoffAngle;
    }


    public double getRealPosition () {

        if(m_Case == 1) {
            return getRealPositionCase1 ();
        }else if (m_Case == 2) {
            return getRealPositionCase2 ();
        }
        else if (m_Case == 3) {
            return getRealPositionCase3 ();
        }
        return angleOffset;
    }
    

public double getRealPositionCase1 () {

        double myPos = getAbsolutePosition()*360;
        return myPos - angleOffset;
} 
public double getRealPositionCase2 () {

    if (getAbsolutePosition() >= m_CutoffAngle)  {
        double myPos = getAbsolutePosition()*360;
        return myPos - angleOffset;
    }else{
        double eNew = 360 - angleOffset;
        double myPos = getAbsolutePosition()*360;
        return eNew + myPos;
    }    
} 
public double getRealPositionCase3 () {

    if (getAbsolutePosition() >= m_CutoffAngle)  {
        double myPos = getAbsolutePosition()*360;
        return myPos - angleOffset;
    }
    else{
        double myPos = getAbsolutePosition()*360;

            return (360 - myPos) + angleOffset;
    }
    
} 
}
