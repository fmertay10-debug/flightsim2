# Vehicles are mass properties plus a list of ForceComponents with named channels

A Vehicle is defined as mass properties + a flat list of ForceComponents, each
implementing one contract — (state, environment, channel inputs) → wrench about its
own reference station, with the CG transfer applied uniformly by the Entity — plus
the named actuator Channels those components declare. Aerodynamics, propulsion,
gimbaled nozzles, RCS, and future rotors are peers behind this one interface;
"rocket" and "aircraft" are no longer code-level types that factories dispatch on,
and configs name every implementation explicitly via registry keys.

We chose this over the previous split (one special AeroModel + propulsive Effectors
+ a fixed ControlInput union) because the goal is generality across vehicle
classes: a helicopter rotor is simultaneously aerodynamics and control effector, so
the aero-vs-effector split cannot express it, and a fixed channel struct cannot
express "a quadrotor has four motor channels" without editing a core header per
vehicle class. Named, declared channels also let the loader validate the
write/read graph at load time, eliminating the silent-open-loop failure mode where
a controller writes channels no component consumes.

Consequences: channel names resolve to indices once at load (no string lookups in
the sim loop); vehicle configs are fully explicit component lists (no key-sniffing,
no type-based defaults); the Python generators (datcom_export.py,
make_missiles.py) emit this schema — the old schema is not supported.
