# forward_command_controller_traced

This is a separately named traced controller plugin. It intentionally does not
install the upstream `forward_command_controller` package or
`libforward_command_controller.so`: Humble's `position_controllers` links to that
standard library, and shadowing it with a controller from another release line is
ABI-unsafe.

## Diagnostics

`tracing_mode` is `off`, `metrics` (default), or `capture`.
`trace_capture_seconds` defaults to 30 and is limited to 300 seconds. Environment
defaults are `MARVIN_TRACING_MODE` and `MARVIN_TRACE_CAPTURE_SECONDS`.
Capture expires into metrics using steady time; reconfigure to start another
capture. These parameters are startup/configuration settings, not live toggles.

Commands always use `commands_traced` with values, timestamp and ID. Legacy
`commands` subscription requires `accept_legacy_commands=true`. Turning diagnostics
off never disables command application or safety monitoring.

Metrics publish at 1 Hz on `/trace/metrics`. Detailed applied events pass through
a bounded 256-slot queue; a non-realtime timer drains it. Full queues drop events
and count the drops without blocking command application. Applied timestamps
describe writing command interfaces, not motor execution or CAN delivery.

Coordinated bringup examples (also available in `isaac_sim.launch.py`):

```bash
ros2 launch bringup robot.launch.py tracing_mode:=metrics
ros2 launch bringup robot.launch.py tracing_mode:=off
ros2 launch bringup robot.launch.py tracing_mode:=capture trace_capture_seconds:=20
```

Each producer starts its own capture deadline during initialization/configuration;
capture sessions are not yet synchronized or controlled from the UI.

## Offline interface-switch test (Jazzy)

Source Jazzy and an isolated build/install of this package first. Then run:

```bash
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest test/test_manager_switch.py -q
```

The test launches its own manager in a process-specific namespace. Its inline
URDF uses only `mock_components/GenericSystem`. It never loads physical hardware.
Two instances of this forward controller request the same position interface.
The test verifies conflicting activation is rejected and strict switching transfers
the claim in both directions. It terminates its manager in a cleanup block.

This is not a joint-test controller or proof of command continuity. It does not
validate CAN watchdog behavior, queued command drainage, or hardware support.

The manager test also observes GenericSystem positions through a joint-state
broadcaster. It checks target retention across both switches, accepts commands
from the active owner, and rejects the effect of traffic to the inactive owner.
Each hold window requires at least ten matching feedback samples. Samples collected
during switch service calls are included. This is sampled mock-state evidence,
not a lossless record of every hardware write or a physical continuity guarantee.
The old sender is stopped before handback. A short settling wait in this test is
not an adequate production source fence.

`test_tracing_modes.cpp` separately verifies local buffer reset and demonstrates
that commands arriving after activation are still accepted, even with old trace IDs.

## Experimental hold-only controller

`forward_command_controller_traced/ExperimentalJointHoldController` is a test
foundation, not a production motion controller. No production launch selects it.
It claims position command/state interfaces for an explicit `joints` list. The
matching `max_handover_error` array is mandatory, in radians, with no physical
default. Activation checks names/order and finite command values.
It captures the existing command, not the measured position.
Updates repeat that command. It has no command subscription or waveform endpoint.
Missing/nonfinite measurement or excessive tracking error latches a separate fault
at activation and during updates. The latch does not release the held interfaces.
Valid feedback alone cannot clear it. The accessor is diagnostic, not motion
permission; no motion-admission endpoint exists yet.

Four focused C++ tests and the extended GenericSystem manager test passed on
Jazzy. The manager test switches into/out of this plugin and checks observed
target retention while an inactive source publishes. Only an isolated install
was built. Humble and physical behavior are unverified.

Do not deploy this plugin on the robot yet. It does not validate measurement age,
operator readiness or hardware watchdog behavior. Failed
activation during a switch can leave the previous controller inactive; this needs
an explicit tested recovery policy. There is no automatic return to IK/policy.

The manager test injects a 0.01 rad GenericSystem following offset. A second hold
instance uses a 0.001 rad tracking bound. It now retains ownership and target
instead of rejecting hold acquisition. The lifecycle test verifies the tracking
latch and non-clearing feedback recovery. Invalid command memory still rejects
activation. Four C++ tests and two manager cases passed. An earlier prototype
rejected tracking mismatch during activation and left interfaces unclaimed after
switch failure; that observation remains a warning about general switch failures.

Read-only status is published at 5 Hz on `~/status`, as a
`std_msgs/String` JSON object. Schema 1 includes `phase`, `revision`,
`tracking_fault_latched`, `motion_available=false`, and
`measurement_freshness="unverified"`. Phases are `inactive`, `pending_first_write`,
`holding`, and `write_failed`. Holding means the last update wrote all command
interfaces successfully, not that physical motors reached their targets.
A non-realtime timer serializes one atomic flags/revision snapshot. It does not
read hardware or allocate JSON in `update()`. Repeated status publication alone
does not prove update progress; consumers must check revision advancement and
receipt age. Revision resets on plugin reconstruction and is not a session token.
The manager test checks fresh inactive/holding reports and tracking-fault reports.

Optional receipt-age checks use `feedback_receipt_interface` and an explicit
positive `max_feedback_age_s`. With no interface selected, freshness remains
unverified and the feedback fault stays set. With an interface selected, the
controller requires an additional state interface per joint, after all position
interfaces, containing a same-host/same-boot steady-clock receipt timestamp in
seconds. Missing, nonfinite, nonpositive, future or expired timestamps latch a
feedback fault without releasing held targets. Valid receipts alone do not clear
that latch. Status adds `feedback_fault_latched` and reports `fresh_receipt` or
`stale_or_invalid_receipt` when configured. These describe cached update checks;
status consumers still need revision/age checks. Motion remains unavailable.

Marvin's ODrive driver already has the opt-in `encoder_receipt_monotonic_s`
interface, enabled through `export_encoder_receipt_timing=true` with tracing
enabled. It estimates kernel CAN receipt time, not ODrive internal sample time.
No driver or launch changes were made here. Simulation still needs a compatible
receipt interface. Five C++ tests, two GenericSystem manager cases and the existing
ODrive encoder timestamp test passed. Live driver-to-controller integration and
Humble behavior remain unverified.

`test/test_receipt_pipeline.py` connects an actual TopicBasedSystem adapter,
controller manager and hold plugin to a synthetic local feedback publisher.
Source isolated builds of both packages and use the same localhost/domain test
environment as above. The test covers fresh feedback, stopped feedback, continued
fixed-target output with retained interface ownership, and resumed feedback that
does not clear the fault latch. Its explicit 0.15 second threshold is a test value,
not an approved robot limit. The fixture is `test/fixtures/receipt_pipeline.yaml`.
No Isaac or physical hardware is started. Live Isaac and Humble remain separate
validation gates; kernel CAN receipt and DDS receipt are different boundaries.

`GetJointTestState` preserves controller identity and the actual activation
generation while inactive or without a valid sample. Such a response still has
`motion_available=false` and `hold_applied=false`. Inactive does not mean generation
zero: a preconnected recorder must distinguish reuse from the first activation.
The chirp completion integration case checks deactivation, reactivation with the
next generation, and a second inactive identity response.

The pure C++ `joint_test_trajectory.hpp` now mirrors the Python prototype's
quintic transition and Hann-windowed sine. Construction validates per-joint
position, handover, speed, acceleration and total-duration bounds. Sampling writes
into caller-owned storage and does not allocate. Invalid time/storage returns an
invalid phase without modifying targets. This helper is not wired to the plugin's
update loop or a start endpoint; it grants no motion permission.

Two native tests passed. `test/joint_trajectory_probe.cpp` and
`test/compare_joint_trajectory.py` provide a repeatable cross-language comparison.
Compile the probe with C++17 and this package's include directory. Set PYTHONPATH
to the marvin_controller checkout, then pass the executable to the Python script.
The 5,001-point test differed by at most 2.78e-17 rad. Numerical peak speed was
0.031416 rad/s and acceleration 0.067503 rad/s², below synthetic limits 0.1 and 0.2.
These figures validate one numerical test plan, not a physical profile.

`joint_test_playback.hpp` adds a single-threaded native session core. Activation
and session IDs fence cancellation/renewal; a renewable lease never extends the
fixed trajectory deadline. Cancellation, expiry, readiness loss and invalid/gapped
time latch a terminal reason without changing the caller's last output. Completion
also retains the last output rather than issuing a jump to baseline. Four native
tests passed. Output is still a proposed command, not evidence of a hardware write.

The core has no ROS transport and is not connected to plugin playback. Its numeric
IDs are not authentication and are not sufficient across controller recreation.
The future adapter must enforce a unique boot identity, non-reused session IDs,
fresh authenticated permits, serialized access, and confirmed hold/handback.
It must hold the actual last successfully written targets on fault. Abrupt target
hold is not a proven safe physical stop or a bounded-acceleration cancellation.

`joint_test_admission.hpp` validates preparation semantics outside the update loop.
It checks nonnil lowercase UUID-shaped identifiers, finite positive values,
controller instance/generation, profile ID/hash, known joint, requested envelope
and server profile structure. Profile limits/baselines cannot come from the request.
Four initial tests used the generated ROS request type. `runtime_interfaces` is now
a normal dependency for the opt-in typed state service described below.

`LoadedJointTestProfile` validates and owns an immutable server-profile copy.
It ignores any supplied hash and computes SHA-256 from the validated contents.
OpenSSL Crypto supplies the digest implementation. Construct this object outside
the update loop. Parameter/file loading and binding the snapshot to a prepared
session remain pending. The raw validator still accepts a plain profile object.
Hash equality identifies content, not approval or authentication. This check grants no lease,
ownership or motion. Construct the mathematical plan outside the update loop from
a coherent snapshot; recheck fences, freshness and snapshot validity before apply.
No physical profile is supplied or selected in this increment.

The canonical encoding starts with ASCII `joint-test-profile.v1` plus one NUL.
It then contains the profile ID, total-duration limit, cycle limit and joint count.
Each joint contributes its name, lower/upper position, speed, acceleration,
handover-error and amplitude limits, in that order. Joint order is significant.
Strings use a four-byte big-endian byte-length prefix. Counts use unsigned
four-byte big-endian integers. Reals use IEEE-754 binary64, big-endian, with
negative zero normalized to positive zero. Nonfinite values are rejected.

The independent Python recipe for the test fixture is:

```python
import hashlib, struct
encode = lambda text: struct.pack('>I', len(text.encode())) + text.encode()
data = b'joint-test-profile.v1\0' + encode('probe') + struct.pack('>dII', 5., 2, 2)
for joint, amplitude in [('hip', .02), ('calf', .03)]:
    data += encode(joint) + struct.pack('>6d', -1., 1., .1, .2, .05, amplitude)
print(hashlib.sha256(data).hexdigest())
```

`PreparedJointTestPlan` binds that immutable profile to a controller snapshot and
the validated trajectory. Preparation rejects an unconfirmed hold, latched faults,
zero revision, mismatched joint order, missing data and stale/future receipt times.
The trajectory checks finite positions, handover error and position/derivative
limits against the captured baseline. All preparation runs outside the update loop.

The prepared direct-joint test uses existing **command targets** as its baseline,
not measured positions. Measurements still gate tracking error, joint limits and
freshness. The `transition_duration_s` interval is a pre-test hold for this path.
Only the selected joint gets the bounded sine offset; all other targets stay
unchanged. This avoids changing the whole pose to compensate for static tracking
error before measuring one actuator. The lower-level trajectory utility can still
represent a transition, but preparation does not request one implicitly.

The controller adapter must capture the snapshot coherently from one update.
Snapshot/receipt times and the age limit are server-owned, not request fields.
Receipt time is not sensor sample time. This helper does not implement snapshot
transport, acquire ownership, issue a lease or start motion. The adapter must
recheck current fences, freshness, faults and command continuity before application.
Three new plan tests passed, bringing admission/profile/plan coverage to ten tests.

The experimental hold plugin now exposes an in-process `read_sample` cache API.
After a complete command write, it captures commands, positions, receipts, activation
and status revision in fixed storage for at most 256 joints. The update loop uses
`try_lock` and skips the capture if a reader holds the mutex. Reads never query
hardware or renew timestamps. A stale cache revision, pending activation, failed
write or inactive state returns false. Consumers must still check sample age and
repeat readiness checks before applying a plan.

This is one controller-update sample, not proof that all sensors sampled together.
It assumes serialized lifecycle/update calls and hardware state stable during update.
The mutex avoids waiting for readers during acquisition, not a hard realtime timing
guarantee. Boot identity, configuration mapping, service integration and playback
application remain pending. No production launch selects this experimental plugin.

`PreparedJointTestPlan::check_apply` adds an allocation-free final state check.
It requires the same controller instance/activation, a nondecreasing revision,
unchanged held targets and fresh valid feedback. It checks measured positions
against position limits, held targets and the captured baseline. The plan expires
after its server-owned feedback-age interval from preparation. This conservative
expiry is not a selected physical limit. It prevents indefinite prepared plans.

The adapter must call this check in the serialized update that applies playback.
An earlier service-side check is insufficient. This helper does not consume a
start permit, enforce one-use execution or validate current profile configuration.
Those checks remain adapter responsibilities. Eleven admission/profile/plan tests
pass, including state changes between preparation and application.

`read_plan_snapshot` now converts the hold cache into `JointTestSnapshot` outside
the update loop. The cached record contains bounded joint names and a random UUIDv4
controller-instance ID, generated at initialization through OpenSSL. Initialization
fails if randomness is unavailable. Each activation attempt increments its generation.
Names, numeric values, flags and identity come from the same cached record. Conversion
does not query hardware or renew feedback timestamps. Joint names are limited to
64 bytes and must not contain NUL. Instance IDs provide restart fencing, not authentication.

This joins controller sampling to the plan data format. Preparation service handlers,
profile loading, start permits and live playback are still not connected. The
converted snapshot can already be stale on return, so final update-time checks
remain mandatory. No production launch selects this controller.

`JointPlanController.CachedStatePreparesPlanAndReactivationInvalidatesIt` connects
the actual hold-controller object to synthetic loaned joint interfaces, then uses
its cache to construct a plan with the generated preparation request type. It checks
fresh admission, stale-receipt rejection, unavailable inactive snapshots and old
request/plan rejection after reactivation. All twelve admission/profile/plan tests
passed. Run this test with `ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1`; otherwise the
controller integration case skips. No physical hardware, manager process, service
handler or playback is involved in this particular test.

`JointTestPermitGate` adds serialized, non-realtime one-use admission. It binds
the controller instance, activation, session and expiring challenge. A matching
request consumes the challenge even when readiness fails or the challenge expires.
An exact retry returns its recorded outcome with `dispatch=false`. A changed
payload under the same operation ID conflicts. A new operation cannot reuse the
consumed gate. The gate is noncopyable and must live for the entire session.

The adapter must generate unique challenges, authenticate callers, serialize calls,
and check current profile/readiness before consumption. It must preserve the outcome
if dispatch fails; no automatic retry is authorized. `accepted` is historical
admission, not successful application or current motion state. Live update checks,
applied acknowledgement, renewal/cancellation and service integration remain pending.

Playback can now be constructed in a prepared state, with all trajectory storage
allocated before the update loop. `begin(fence, now)` starts the lease and trajectory
clock once, without allocation. Ticks and renewals cannot start prepared playback.
Cancellation before start and an invalid start clock latch terminal outcomes.
Duplicate begin calls cannot reset elapsed time or extend the lease. The adapter
must consume the start permit and check live state before begin in the same update.
Six playback tests passed. This remains a native helper, not live plugin playback.

The startup parameter `enable_joint_test_state_service` defaults to false. When
true, the experimental plugin exposes `~/joint_test/state` using
`runtime_interfaces/srv/GetJointTestState`. It reads cached state only. A newer
update revision alone does not invalidate the last complete snapshot. The reader
requires matching activation and all status flags, successful writes, and bounded
sample age. Existing encoder-receipt freshness checks remain in place. This avoids
false `sample_unavailable` responses between status publication and cache copying,
without making the update loop wait for readers. `test_joint_sample_validity`
checks that interleaving and rejects changed flags, activation and expired samples.
It returns
the controller identity and, when a current sample exists, activation/revision and
fault flags. Receipt and sample ages are checked at query time. Unverified/stale
samples cannot assert `hold_applied` or `feedback_receipt_verified` in this service.
`motion_available` is always false and no permit challenge is issued.

This introduces a normal dependency on the existing `runtime_interfaces` package.
Build its updated interfaces before this controller. No production launch enables
the service. Tests verify default absence, typed inactive-state responses and
unchanged repeated reads. The service test also covers active hold, cache expiry
without controller updates, fault latching on the next update, fresh feedback that
does not clear the latch, and unavailable state after deactivation. The 0.25 second
age limit is synthetic. All sixteen admission/plan/service tests passed.
The preparation endpoint is described below. Control handlers and live playback remain absent.

### Chirp profile support (not approved for physical deployment)

Optional `enable_joint_test_target_telemetry` defaults to false. When enabled,
`~/joint_test/applied_targets` publishes `sensor_msgs/msg/JointState` with joint
names and targets after a complete successful command-interface write.
Velocity and effort arrays remain empty. This is controller-output evidence,
not CAN delivery, motor acknowledgement or measured motion.

The publisher uses preallocated message storage and a nonblocking realtime
publisher. Busy publication skips a sample without delaying the controller.
The timestamp is the controller-manager update time, not feedback receipt time
or CAN send time. Confirm its clock domain before comparing with other streams.
Inactive controllers emit no new samples, but previously queued samples can arrive.
Best-effort transport can lose samples. A companion `~/joint_test/target_samples`
stream uses `runtime_interfaces/msg/JointTestTargetSample`. It includes controller
identity, activation, monotonic sample time and an update sequence. The sequence
increments after every complete successful update, including skipped publications.
The typed stream now uses a fixed 256-sample single-producer/single-consumer queue.
A separate worker publishes reliable ROS messages with history depth 64.
The update loop does not allocate or wait for this publisher. A full queue rejects
the new sample and increments `target_queue_overflow`. Publication exceptions
increment `target_publish_errors`. Both counters appear in status. Missing
sequences and stale timestamps still fail the recorder guard. Buffering does not
extend the 150 ms live-data limit. Worker shutdown can discard queued telemetry.
Use this stream to detect missing samples. The JointState stream remains available
for existing tools. Neither stream confirms CAN delivery or motor response.

The controller can prepare a linear-frequency chirp using two optional, immutable
profile parameters: `joint_test_profile.chirp_start_hz` and
`joint_test_profile.chirp_end_hz`. Both default to zero, preserving the existing
windowed sine and its v1 profile hash. A chirp requires positive start frequency,
end frequency at least as high, and `max_cycles=1`. The request's `cycles` must
also be 1; it does not specify the number of chirp oscillations.

Chirp profiles use the distinct `joint-test-profile.v2-chirp` hash domain and
include both frequencies. Clients cannot change the sweep through request fields.
The existing sim-only HTTP/Zenoh allowlist is unchanged and does not admit this
new profile. There is no reviewed physical profile or deployed robot chirp yet.

The target is `baseline + amplitude * sin(phase) * sin(pi*t/T)^2`, with
`phase = 2*pi*(start_hz*t + (end_hz-start_hz)*t*t/(2*T))`.
Admission checks conservative continuous speed and acceleration bounds, including
the phase acceleration caused by the sweep. Amplitude is bounded by the fixed
profile; this is not an adaptive amplitude or current controller.

The ordinary preparation fences, exclusive command interfaces, lease renewal,
feedback freshness checks and cancellation path apply unchanged. Cancel holds
the last output; it does not remove motor torque. Before physical testing, verify
the independent hardware-disable and power-disconnect paths. Never assume that a
passing synthetic cancellation test proves the physical operator stop works.

Tests include standalone numerical checks and a synthetic ROS manager pipeline
for chirp completion, lease expiry, feedback loss and cancellation. These are not
Isaac physics or real-hardware acceptance tests.

Server profile loading is opt-in through `joint_test_profile.enabled` (false by
default). Configuration requires verified receipt-interface setup and these
parameters under `joint_test_profile`:

- `id`: nonempty profile identifier.
- `lower_rad`, `upper_rad`, `speed_rad_s`, `acceleration_rad_s2`,
  `max_amplitude_rad`: arrays in the exact configured `joints` order.
- `max_total_seconds`: positive total-duration bound.
- `max_cycles`: positive integer within uint32 range.

Handover bounds come from the existing per-joint `max_handover_error`. No physical
defaults are supplied. The loader validates all contents and computes the hash.
`test_profile()` returns an immutable shared snapshot outside the update loop.
Parameter edits do not hot-reload it. Reconfiguration first clears the current
profile and publishes a replacement only after successful validation. Previously
held snapshots remain immutable but are not evidence of current configuration.
Start handling must compare against the current profile. Seventeen focused tests
passed, including valid loading, no implicit hot reload and invalid replacement.

`enable_joint_test_preparation_service` defaults to false. When enabled at startup,
`~/joint_test/prepare` uses `runtime_interfaces/srv/PrepareJointTest`. It loads the
current immutable profile and cached state, validates the trajectory, and retains
one prepared plan per activation. It does not write targets, acquire ownership,
issue a start challenge or extend a lease. Motion remains unavailable.

The default mutually-exclusive callback group serializes preparation requests.
The controller retains up to 128 operation outcomes for its instance lifetime.
Exact retries return historical responses, including failures. Changed payloads
under the same ID return `OPERATION_CONFLICT`. Full history returns
`OPERATION_CAPACITY` without eviction. A second plan in the same activation returns
`PREPARATION_BUSY` until the pending plan is cancelled. A later
activation can prepare a new plan with a new operation ID. A historical accepted
response does not prove that its plan is still current or unexpired.

Seventeen focused tests passed, including ROS preparation, unchanged command
targets, failed-result replay, exact accepted replay, conflicts and busy rejection.
Live start still requires current-profile and update-time state checks. Do not
enable this experimental workflow for physical tests yet.

Enabling preparation also exposes `~/joint_test/control`. Only `CANCEL` is
implemented. It matches instance, activation and session without requiring a lease
or challenge, then queues cancellation through a fixed mailbox. Held targets do not change.
A new preparation can replace it after acknowledgement. Old cancellation retries return historical
outcomes and cannot cancel the replacement. Cross-endpoint operation-ID reuse
conflicts. Normal control history has a 4096-entry bound. A first valid cancellation
for the current session bypasses that bound and retains its outcome. Each session
can use this allowance once. The preparation ledger admits at most 128 sessions,
so control history remains bounded by 4224 entries. Exact retries still replay
without side effects. This is pending-plan cancellation, not a physical stop path.
The update loop uses try-lock acquisition and acknowledges only after every hold
write succeeds. The cache carries the cancelled session and the first cancellation's
request sequence. `cancel_pending` remains visible before acknowledgement. Neither
a service response nor a complete interface write proves physical standstill.
Playback cancellation still needs to stop trajectory generation before holding.

The state service reports the session ID and `prepared`, `prepared_unready`, `cancel_pending` or
`cancelled` for a matching activation. Preparation readiness is checked against
the current profile and live cached state without extending expiry. All seventeen
focused tests passed, including cancellation, replacement, status and denied start.

The ROS service test fills normal control history, cancels the pending session,
checks replay/conflict behavior, prepares a replacement and cancels that session
through its separate allowance. No commands change during these operations.
It also checks pending versus applied cancellation, applied sequence and rejection
of replacement before acknowledgement. Three manager integration cases passed.

New cancellation requests also require a currently active controller generation.
A retained plan is not evidence that its activation is live. `STALE_ACTIVATION`
rejects new requests after deactivation or for an earlier generation. Exact retries
still return historical outcomes without dispatch. The mailbox checks activation
again in update. Tests deactivate with cancellation pending, reactivate, and confirm
that no old cancellation acknowledgement appears in the new generation.

`JointTestExecution` combines the immutable plan, deferred playback and acknowledged
target storage. Construction allocates outside update. Begin checks the current
profile hash and live state once. Update proposes targets, invokes a supplied
per-joint writer, and records only successful interface writes. A failed write
latches a fault and stops trajectory advancement. Later updates hold the last
acknowledged target for each joint, including a partial-write mixture. They do not
resume the trajectory. The adapter must provide a nonthrowing, nonallocating writer
and retain exclusive command-interface ownership.

This native component is not yet connected to the plugin's update loop. Tests
cover denied/repeated starts, cancellation, partial failure and latched hold. All
nineteen focused tests passed. A successful write is not motor feedback. An
unacknowledged write can leave physical state uncertain, and target hold is not a
validated safe stop or bounded-acceleration cancellation.

The plugin now connects native execution to its update loop behind startup option
`enable_experimental_joint_playback=false`. Only the internal management adapter
method `stage_prepared_execution` can stage it. The ROS START action still rejects
requests. The future adapter must consume an authenticated one-use permit and
supply server-owned lease/gap limits before calling this method.

Staging allocates plan/state/target storage outside update and verifies session,
activation and current profile. Update uses try-lock acquisition, checks live state,
begins once and writes through loaned command interfaces. Busy mailboxes retain
the last held targets. Pending cancellation stops playback before the next target
write and is acknowledged after a complete hold write. Cache state reports start
sequence and executing/execution_hold phases. Old activation objects are ignored
and replaced outside update, not freed in the update loop.

Nineteen focused tests passed, including actual controller-object playback with
synthetic interfaces, elapsed-zero continuity, changed targets, applied sequence,
in-motion cancellation, replacement and activation fencing. Three manager regression
cases also passed. Manager-driven playback, ROS start/renewal permission handling,
simulator testing and physical validation remain pending. No production launch
enables playback. The state field motion_available still means remote start is unavailable.

Playback state now reports specific terminal reasons: TEST_COMPLETED, CANCELLED,
READINESS_LOST, LEASE_EXPIRED, UPDATE_TIMING_FAULT or INVALID_OUTPUT_STORAGE.
The controller-object test expires a synthetic lease, verifies unchanged written
targets and the applied start sequence, rejects same-session restaging, and then
cancels the expired session. A separate execution test checks permission loss,
feedback-readiness loss and a missed update. Every terminal update writes the
last acknowledged targets. Restored readiness and renewal cannot resume the session.
Twenty focused tests passed after an isolated rebuild. These synthetic checks do
not establish physical standstill or a safe stopping response under ground load.

Renewal now has an issuance/application-time overload. The controller challenge
issuance time determines the new lease deadline; time spent queued consumes that
lease. Both timestamps must use the controller's steady clock. The adapter must
obtain issuance time from its own challenge record, not trust a request timestamp.
Repeated, regressed or future issuance times cannot extend permission. An expired
existing lease cannot be revived, and renewal at the fixed trajectory deadline
reports completion. The immediate-renewal overload remains for synchronous callers.
Twenty admission/execution tests and seven playback tests passed. Challenge
generation, transport deduplication and live renewal delivery remain unwired.

Staged starts now carry controller-clock permission issuance through execution to
the update loop. Queue delay consumes the initial lease. The default internal
staging overload timestamps staging; a future transport adapter must pass the
earlier trusted challenge issuance explicitly. An expired start retains inherited
targets, reports start_rejected / LEASE_EXPIRED and does not acknowledge a start
sequence. A still-valid start begins the waveform at elapsed zero, with only the
remaining permission lifetime. Same-session restaging cannot revive rejection.
Twenty admission/execution tests and eight playback tests passed after rebuild,
including the controller-object rejection path. Remote ROS START remains disabled.

### Current experimental ROS start integration

`enable_experimental_joint_start=false` is a separate startup gate. Enabling it
requires playback, preparation and state-service opt-ins, plus explicit positive
finite `joint_test_lease_s` and `joint_test_max_update_gap_s`. Both timing defaults
are zero, so enabling Start without reviewed limits fails initialization.
No production launch enables these options. Use only isolated synthetic testing
until gateway access control, renewal, handover and physical fault handling are verified.

Preparation generates a random UUIDv4 challenge and records its issuance time.
Exact preparation retries return the original response. State reads expose the
same unconsumed challenge and decreasing remaining lifetime only while ready;
they never generate or renew permission. START consumes that challenge once,
checks live cached readiness, and stages execution with server timing limits.
The response and cached state report start_pending until update acknowledges
application. Exact operation retries return their original response without staging
again. Changed requests conflict; a new operation cannot reuse consumed permission.
Update still checks the original issuance deadline and current state before motion.

This challenge is freshness evidence, not caller authentication. Any reachable ROS
caller may request it; network authorization must be enforced separately. RENEW and
RELEASE remain disabled. Without renewal, execution stops at the initial lease.
Twenty-two focused tests and three manager regressions passed, including actual ROS
prepare/start/state/cancel, duplicate/conflict handling, expiry, unchanged targets
before update and opt-in configuration rejection. All interfaces were synthetic.

Manager-driven playback is now exercised by the playback variant in
`test/test_receipt_pipeline.py`, using `test/fixtures/receipt_playback.yaml` as
synthetic-only overrides. It starts the actual Jazzy controller manager and
TopicBasedSystem, switches from a normal controller to the test controller, and
uses ROS preparation/start rather than the internal staging method. It verifies
an observed command excursion, applied sequence and exclusion of the normal
controller. Commands published to that inactive source cannot replace playback.
Feedback loss produces execution_hold / READINESS_LOST while interface ownership
remains active. Feedback recovery does not resume the trajectory; output retains
the final held target. Both receipt variants passed, then all four manager/receipt
cases passed in 14.90 seconds. No actual Isaac scene, CAN or robot was involved.

### Guarded renewal

The same experimental opt-in now enables RENEW. The shared one-use gate lives in
`joint_test_permit.hpp` and binds the action as well as identity/session/challenge.
Renewal sequence must exceed the last requested sequence. The controller creates
a new challenge after accepting Start or Renew but exposes it through state only
after the previous operation is applied and fresh, fault-free execution is observed.
State reads do not generate challenges. The ROS callback queues a bounded renewal
slot; only update can extend the lease or acknowledge the sequence. Queue delay
consumes the original challenge lifetime. Expiry, missed updates and readiness
faults prevent renewal. Cancellation is processed first and cannot be reversed by
a pending renewal. The fixed trajectory deadline is never extended.

Exact retries return recorded outcomes without another renewal. The existing
4096-entry control ledger still limits total admitted requests per instance;
reserved first-cancellation outcomes remain available beyond that capacity.
Exhaustion denies further renewal and the lease expires; it does not grant motion.
The challenge is not authentication. RELEASE, gateway authorization, production
profiles and physical validation remain unfinished. Production remains unchanged.
Twenty-three admission/execution tests and eight playback tests passed. All four
manager/receipt cases passed in 14.43 seconds before the final timing guard; both
receipt variants passed again in 5.04 seconds after it. Tests include repeated ROS
renewals beyond the initial lease, delayed acknowledgement, duplicate outcomes,
changing challenges, cancellation priority and manager-applied renewal sequence.

The manager test now includes full completion and lease-expiry scenarios. Both use
a 1.65-second synthetic trajectory, longer than the one-second initial lease.
Completion needs repeated ROS renewals and verifies both positive and negative
command excursions. Expiry receives no renewals and reports LEASE_EXPIRED while
feedback remains fresh. Both retain exclusive ownership and repeated held outputs,
reject late renewal, and accept cancellation. Service clients are reused during
monitoring instead of recreated for each request. All four receipt scenarios
passed in 11.54 seconds. This tests command generation and ownership, not a motor
response model or physical standstill.

### Opt-in simultaneous chirp

An immutable profile may provide `joint_test_profile.simultaneous_directions`,
with one +1 or -1 per joint. Only the explicit `__all__` selector is then accepted.
Every joint receives the same amplitude with its configured sign. All joint
position, speed and acceleration bounds are validated before preparation.
Single-joint profiles reject this selector. The v3 profile digest binds direction
values, order and existing limits. Empty directions preserve legacy hashes.
The controller keeps existing tracking, receipt, ownership, lease and stop checks.
This does not add automatic pose repositioning or authorize physical deployment.


### Physical activation receipt

`export_command_receipt` defaults to false. Marvin's physical configuration enables
it to claim a `command_receipt_monotonic_s` interface for each position joint.
The controller buffers callback receipt time with the command, then writes that
same time on every update. The ODrive driver uses it to reject retained commands
from before hardware activation. Interface loan order does not change the pairing.
This option requires the paired ODrive hardware interfaces and is off in simulation.

## Supervised native diagnostics (default)

`forward_command_controller_traced/SupervisedCommandController` replaces the normal
forward controller by default in the Marvin robot and simulation launches. It owns
the same command interfaces. Do not load both controllers as simultaneous writers.
`ROBOT_RUNTIME_NATIVE_EXECUTION=false` temporarily selects the rollback path. Retain
that path until powered stand validation passes; it is not the preferred executor.

The supervisor sends one bounded chirp plan through `~/execution`, then renews
permission through `~/execution_commands`. Streamed IK/policy targets use that same
fenced input. Legacy target topics are not command inputs in this mode.
`~/commands_traced` reports applied output for existing observers and recorders.
`~/execution_state` reports the controller epoch, generation, applied hold, terminal
fault, target sequence and dropped status records. Queue acceptance is not an
applied acknowledgement. These identities fence stale messages; they are not
caller authentication.

The update runs at the configured controller rate (200 Hz in Marvin). Trajectory
targets change at most once per 20 ms. Intermediate updates retain the target and
check permission, actual feedback receipt age, tracking and deadlines. Missed
samples are skipped, never sent as a burst. Cancel/fault holds the last accepted
target. A new generation is required to release a terminal execution. Hardware
watchdogs and explicit motor disable remain separate mechanisms.

Required parameters: twelve `joints`, `interface_name: position`, twelve-element
`diagnostic_lower`, `diagnostic_upper`, `diagnostic_amplitude`, and
`feedback_receipt_interface`. Launch derives bounds from the canonical joint-limit
YAML. Simulation uses `feedback_receipt_monotonic_s`; the robot uses
`encoder_receipt_monotonic_s`. A missing receipt interface rejects activation.
The robot also retains `export_command_receipt: true`.

Current diagnostic bounds: 0.2 s permission, feedback and update-gap limits;
0.2 rad tracking error; 0.3 rad target-step budget with 20 rad/s replenishment.
Existing per-group amplitude and travel limits remain active. These are configured
software limits, not validation of every possible 10 Hz physical sweep.

Verification: `test_diagnostic_execution` tests the pure state machine.
`test/test_native_execution_ros.py` uses an actual controller manager and a synthetic
TopicBasedSystem on localhost domain 229. It opens no CAN interface. Fixed-body
Isaac acceptance passed 40 checks on 2026-10-28, including a 30-second all-joint
0.1–10 Hz sweep, handovers, cancellation, recovery, feedback-loss hold and Rest.
The 30-second capture contained 1,501 contiguous sample records with zero recorder
or native-status drops. Targets averaged 20 ms apart. The test used small amplitudes
(hip/thigh/calf: 0.01/0.015/0.02 rad), not every admitted amplitude/frequency pair.

The owned Isaac verification uses a stage-scoped Python GC baseline before play.
An automatic full collection previously paused the simulator for 226 ms and
correctly triggered the unchanged 200 ms freshness limit. New-object collection
remains enabled, and stage close releases the frozen baseline. This is a verification
startup profile, not a shared GUI or physical robot deployment. Recording quality
still reports missing context samples and read errors separately from waveform integrity.
See `marvin-measurements/native-chirp-20261028-isaac-25` and the Marvin wiki.
No walking, balance, Humble build or physical readiness is established by this test.


Default rollout, 2026-10-28: the Pi/Humble build and no-hardware ROS integration
passed. The new overlay is selected for the next runtime start, with motion stopped.
The normal workstation default also passed 40 Isaac checks. Powered stand validation
remains required before removing the legacy execution override. Deployment evidence:
`marvin-measurements/native-default-20261028/report.md`.


## 2026-10-28 — report-only tracking during chirp

With operator authorization, active chirp tracking overruns are report-only.
The 0.2 rad threshold remains a warning threshold. Admission/start still requires
the measured joints to be within the existing tracking bound of the held targets.
Absolute-position diagnostics retain the stopping tracking check.

The native controller parameter chirp_tracking_report_only defaults to true.
Setting it false restores the active-sweep tracking abort. The C++ kernel keeps
the strict default unless its owning controller selects report-only explicitly.
It counts joint-sample overruns and records the largest error and joint index.
The non-realtime status worker emits throttled CHIRP_TRACKING_REPORT_ONLY warnings
with operation identity, joint name, peak radians, threshold and cumulative count.
No logging occurs in the controller update loop.

The Python executor also omits only the active-chirp tracking abort. It reports
overrun samples and peak error in diagnostic status and emits throttled
chirp_tracking_report_only events. Admission and absolute-position tests stay
strict. Target bounds, feedback validity/receipt age, tilt, continuity, execution
deadlines, permission leases, exclusive ownership and cancellation remain enforced.
Normal completion still returns to the captured start targets and holds.

Workstation: 122 focused Python tests and the native kernel test passed. A real
controller-manager test with synthetic feedback verified that a 0.35 rad error on
joint_3 is logged while the sweep completes, and frozen feedback still faults.
Pi build and verification passed: 122 focused Python tests, the native kernel test,
and the isolated ROS controller-manager integration test. The service now selects
`/home/mikael/Development/chirp-tracking-report-20261028/install/setup.bash`
for the next start. Runtime remains inactive. Physical sweep verification remains pending.
