import se.lth.control.realtime.AnalogIn;
import se.lth.control.realtime.AnalogOut;
import se.lth.control.realtime.IOChannelException;

public class Regul extends Thread {

    private PI inner = new PI();
    private PID outer = new PID();

    private ReferenceGenerator refGen;
    private OpCom opCom;

    private AnalogIn analogInAngle;
    private AnalogIn analogInPosition;
    private AnalogOut analogOut;

    private int priority;
    private boolean shouldRun = true;
    private long startTime;

    private ModeMonitor modeMon;

    public Regul(int pri, ModeMonitor modeMon) {
        priority = pri;
        setPriority(priority);
        this.modeMon = modeMon;

        try {
            /** Written by you */
            analogInAngle = new AnalogIn(0);
            analogInPosition = new AnalogIn(1);
            analogOut = new AnalogOut(0);
        } catch (Exception e) {
            System.out.print("Error: IOChannelException: ");
            System.out.println(e.getMessage());
        }
    }

    /** Sets OpCom (called from main) */
    public void setOpCom(OpCom opCom) {
        /** Written by you */
        this.opCom = opCom;
    }

    /** Sets ReferenceGenerator (called from main) */
    public void setRefGen(ReferenceGenerator refGen) {
        /** Written by you */
        this.refGen = refGen;
    }

    // Called in every sample in order to send plot data to OpCom
    private void sendDataToOpCom(double yRef, double y, double u) {
        double x = (double) (System.currentTimeMillis() - startTime) / 1000.0;
        opCom.putControlData(x, u);
        opCom.putMeasurementData(x, yRef, y);
    }

    // Sets the inner controller's parameters
    public void setInnerParameters(PIParameters p) {
        /** Written by you */
        this.inner.setParameters(p);
    }

    // Gets the inner controller's parameters
    public PIParameters getInnerParameters() {
        /** Written by you */
        return this.inner.getParameters();
    }

    // Sets the outer controller's parameters
    public void setOuterParameters(PIDParameters p) {
        this.outer.setParameters(p);
    }

    // Gets the outer controller's parameters
    public PIDParameters getOuterParameters(){
        /** Written by you */
        return this.outer.getParameters();
    }

    // Called from OpCom when shutting down
    public void shutDown() {
        shouldRun = false;
    }

    // Saturation function
    private double limit(double v) {
        return limit(v, -10, 10);
    }

    // Saturation function
    private double limit(double v, double min, double max) {
        if (v < min) v = min;
        else if (v > max) v = max;
        return v;
    }

    public void run() {

        long duration;
        long t = System.currentTimeMillis();
        startTime = t;

        // double ang, y, yRef, angRef, u;
        double ang = 0;
        double y = 0;
        double yRef = 0;
        double angRef = 0;
        double u = 0;

        while (shouldRun) {
            /** Written by you */

            switch (modeMon.getMode()) {
                case OFF: {
                    /** Written by you */
                    y = 0;
                    yRef = 0;
                    u = 0;
                    break;
                }
                case BEAM: {
                    y = readInput(analogInPosition);
                    yRef = refGen.getRef();

                    synchronized (inner) {
                        u = limit(inner.calculateOutput(y, yRef));
                        writeOutput(u);
                        inner.updateState(u);
                    }
                    break;
                }
                case BALL: {
                    ang = readInput(analogInAngle);
                    y = readInput(analogInPosition);
                    yRef = refGen.getRef();

                    synchronized (outer) {
                        angRef = outer.calculateOutput(y, yRef);
                        outer.updateState(angRef);
                    }

                    synchronized (inner) {
                        u = limit(inner.calculateOutput(ang, angRef));
                        writeOutput(u);
                        inner.updateState(u);
                    }
                    break;
                }
                default: {
                    System.out.println("Error: Illegal mode.");
                    break;
                }
            }

            sendDataToOpCom(yRef, y, u);

            // sleep
            t = t + inner.getHMillis();
            duration = t - System.currentTimeMillis();
            if (duration > 0) {
                try {
                    sleep(duration);
                } catch (InterruptedException x) {}
            } else {
                System.out.println("Lagging behind...");
            }
        }
        /** Written by you: Set control signal to zero before exiting run loop */
    }

    // Writes the control signal u to the output channel: analogOut
    // @throws: IOChannelException
    private void writeOutput(double u) {
        try {
            analogOut.set(u);
        } catch (IOChannelException e) {
            e.printStackTrace();
        }
    }

    // Reads the measurement value from the input channel: in
    // @throws: IOChannelException
    private double readInput(AnalogIn in) {
        try {
            return in.get();
        } catch (IOChannelException e) {
            e.printStackTrace();
            return 0.0;
        }
    }
}
