Python PLC Project:
This project aims to develop a framework which combines physical servo hardware with virtual control systems.

---GENERAL OUTLINE---

    A "Servo" will be treated as any object which can implement a control loop. Here, a control loop consists of a
physical system over which control is exerted, a virtual set point, and an internal model on how to correspond the two.

This gives rise to three communication protocols:

1. A protocol that exists around building the servo pipeline. This should include the following methods:
    Open   ->
    Close  ->
    Reset  ->
    Verify -> Identify a downstream servo. Important for physical servos, where connection isn't guaranteed.
    and some framework for error handling, such as keeping track of when a servo is physically disconnected.

    ---NETWORK SANITY---
    building the servo network will require that the control pipeline is sensible. Each servo will have its own input
    and output space, and it will need to be ensured that all variables that are feeding through have the correct
    dimensionality, units etc.

2. The control loop protocol.
    The servo at the top of the ladder manages all communication requests. All communications in the control loop should
    be synchronous when possible.
    This gives each servo two vector spaces, which are not necessarily linear. One for the control vector, and one for
    the sense vector.

3. A protocol for calibrating each servo.
    This sets the feed_forward and feed_back functions.

---SERIAL INTERFACE---
    If Physical servos are to be used we need a system to communicate with them, and some sort of serial interface is an
obvious choice. We want all serial communications to be synchronous, i.e. have a definite length of the communication.
As such, Serial communications should have
