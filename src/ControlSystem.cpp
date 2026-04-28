#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), limTiltX(-0.2, 0.2),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("IMU quaternion");
    g.setName("gain");
    limTiltX.setName("Tilt limit in x-direction");

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");
    g.getOut().getSignal().setName("alpha");

    // Connect signals
    g.getIn().connect(q1.getOut());
    limTiltX.getIn().connect(g.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(q1);
    timedomain.addBlock(g);
    timedomain.addBlock(limTiltX);
    
    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}