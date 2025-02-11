
public class PID {
	// Current PID parameters
	private PIDParameters p;

    private double I = 0; // Integral part of PID
    private double D = 0; // Derivative part of PID
    private double v = 0; // Computed control signal
    private double e = 0; // Error signal
    // private double y = 0; // Measurement signal
    private double yOld = 0; // Old measurement signal
    private double ad; // Help variable for Derivative calculation
    private double bd; // Help variable for Derivative calculation
	
	// Constructor
	public PID() {
        PIDParameters p = new PIDParameters();
        p.Beta = 1.0;
        p.H = 0.1;
        p.K = -0.2;
        p.N = 5.0;
        p.Td = 0.5;
        p.Ti = 0.0;
        p.Tr = 1.0;
        p.integratorOn = false;
        setParameters(p);       
    }
	
	// Calculates the control signal v.
	public synchronized double calculateOutput(double y, double yref) {
        this.e = yref-y;
        this.D = ad*D - bd*(y-yOld);
        this.v = p.K*(p.Beta*yref-y)+I+D;
        yOld = y;
        return this.v;
    }
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	public synchronized void updateState(double u) {
        if (p.integratorOn) {
			I = I + (p.K*p.H/p.Ti)*e + (p.H/p.Tr)*(u-v);
		} else {
			I = 0.0;
		}		
    }
	
	// Returns the sampling interval expressed as a long.
	// Note: Explicit type casting needed
	public synchronized long getHMillis() {
        return (long)(p.H*1000.0);
    }
	
	// Sets the PIDParameters.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters) {
        p = (PIDParameters)newParameters.clone();
        if (!p.integratorOn) {
			I = 0.0;
		}	
		ad = p.Td/(p.Td+p.N*p.H);
        bd = (p.K*p.Td*p.N)/(p.Td+p.N*p.H);
    }

    public synchronized PIDParameters getParameters() {
        return p;
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.
    public synchronized void reset() {
        I = 0.0;
    }
}

