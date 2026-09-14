# Security Policy

## Scope

AuNa is research and simulation infrastructure for autonomous navigation. It is
intended for use in simulation and on scaled research vehicles in a controlled
environment. It is **not** hardened for safety-critical or public-road
deployment, and it carries no safety certification.

The default deployment makes assumptions appropriate to a lab network and not to
an untrusted one:

- Docker Compose services run with `network_mode: host`, and several run
  privileged in order to reach display and hardware devices.
- ROS 2 communication is unauthenticated and unencrypted; SROS 2 is not
  configured. Anyone able to reach the ROS domain can publish commands to a
  robot.
- V2X / CAM messages are neither signed nor validated against the ETSI security
  profile.

Run AuNa on a trusted, isolated network. Treat any host running it as capable of
commanding the connected vehicles.

## Supported versions

Security fixes are applied to the `main` branch. There are no long-term support
branches; older commits and version-specific branches are not patched.

## Reporting a vulnerability

Report suspected vulnerabilities privately to **harun.teper@tu-dortmund.de**
rather than opening a public issue. Please include a description of the issue,
the affected component, and steps to reproduce it.

As a small research project, AuNa cannot commit to a fixed response deadline.
You can expect an acknowledgement, an assessment of whether the report falls
inside the scope above, and credit in the fix unless you prefer otherwise.

Issues that follow from the documented design — for example, that an
unauthenticated ROS 2 topic can be published to by anyone on the same domain —
are known properties of the default configuration rather than vulnerabilities.
Reports that such a property is inadequately documented are welcome.
