# Angular blockage sleep

Enable **Sleep On Angular Blockage** under **Mechanism > Experiment** on an MA using Angular Position mode. It is
off by default. Existing endpoint sensing and the ordinary stall timeout remain
available.

The fast path requires all of the following:

- A valid independent simulated Child and Parent, an active angular position
  drive, and substantial error from both the final and currently applied target.
- Earlier measurable, directed relative rotation and reduction of target error
  during the same command. The
  configured speed cap alone never establishes a motion baseline.
- A current loaded, non-probe collision whose normal opposes continued rotation
  about the joint pivot. Mere broadphase overlap, a non-blocking support normal,
  or the Parent/Child joint itself is insufficient.
- At least four distinct physics-time samples of continuous resisting contact,
  covering **Angular Blockage Confirmation** (default 0.15 seconds).
- Relative angular motion confined within **Angular Blockage Jitter** (default
  0.05 degrees), with no meaningful directed progress or advancing motion envelope.

Actual relative angles are sampled on the physics thread, so moving the entire
machine does not count as jaw progress. Small back-and-forth oscillations are
measured by their envelope, not instantaneous peak angular velocity. Monotonic
creep vetoes fast sleep even if it fits inside the jitter amplitude. Very low
speeds with insufficient measurable travel, motion blocked from rest, missing
contact reactions, and sampling gaps fall back to the original policy. The
detector is deliberately conservative; finite-resolution motion cannot provide
an absolute guarantee for every noisy, arbitrarily slow trajectory.

Once confirmed, the next game-thread update releases the position drive, calls
native Chaos sleep, and emits the existing `RotateToEnd` and `RotationBlocked`
events. It does not report `RotateToTarget` for a blocked grip. Existing **Freeze
On Rotation Stopped** still controls subsequent freezing. This removes the
additional one-second stall wait after confirmation, rather than promising a
zero-duration physical obstruction decision.

New commands, target/axis/body changes, constraint rebuilds, freeze, or teardown
invalidate old evidence. Physics work carries copied metadata and proxy lifetime
identities, never UObjects. There is at most one queued sample per MA. Current
pose and request age are checked before applying a completed physics result.
Existing operation logging can show the single `[AngularBlockage]` confirmation;
no continuous diagnostic capture is added.

The window-policy test source covers directed slow motion/creep, oscillation in
either direction, absent/intermittent contacts, startup, reset and time gaps.
Runtime integration and contact geometry still need an explicit build and scene
test; no checks are automatically executed by this feature.
