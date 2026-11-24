---
id: doc-5
title: 'AA-57c LeRobot background chunk inference: design note (2026-11-24)'
type: other
created_date: '2026-11-24 06:36'
updated_date: '2026-11-24 06:36'
---
# AA-57c: does LeRobot compute the next action chunk in the background by default?

Read-only design note. No code, backlog or git changes were made.
`L = robot/lerobot_env/.venv/lib/python3.12/site-packages/lerobot` (the installed version).

## Short answer

**No. LeRobot's default path is synchronous.** `ACTPolicy.select_action` runs the network
inline, on the caller's thread, whenever its action queue is empty. `lerobot-rollout` defaults to
`SyncInferenceEngine`, which is "inline mode — no background thread". `lerobot-record` no longer
runs policies at all. LeRobot ships two opt-in ways to compute chunks in the background:

1. `async_inference`: a gRPC policy server plus a robot client with two threads.
2. `rollout --inference.type=rtc`: a background thread for Real-Time Chunking. ACT does not
   support RTC.

So adding background chunk computation would mean adopting an opt-in LeRobot feature or writing
our own. It is not something we would get for free by "using LeRobot normally".

## Our facts

From the brief, plus a check of the code:
- The checkpoint has `chunk_size=100`, `n_action_steps=20` and `temporal_ensemble_coeff=null`.
- `policy_handoff.py` `PolicyAttempt.run` (line 97) calls `self.policy.step(...)` once per
  control step (line 135). `policy.step` is `MappedPolicy.step` → `LeRobotBackend.step`
  (`src/inchworm/policy_runner.py:772`). At line 784 that calls
  `self._post(self._policy.select_action(self._pre(frame)))`, so we use the standard LeRobot
  synchronous path.
- So 1 step in every 20 runs the full network (80–96 ms) and the other 19 pop from the queue.
- Arithmetic check: 20 steps / (19 × 33.3 ms + ~90 ms) ≈ 20 / 0.72 s ≈ **27.9 Hz**. This matches the
  ~28 Hz we measured. The shortfall comes entirely from one stall of about 3 periods every 20 steps.
  The per-step budget is otherwise fine.

## (1) ACT `select_action`: synchronous queue refill

`$L/policies/act/modeling_act.py`:

- Line 93–98, `reset()`:
  `self._action_queue = deque([], maxlen=self.config.n_action_steps)` (line 98). The queue holds
  `n_action_steps` (20 for us), not `chunk_size` (100).
- Line 101–107, `select_action` docstring: *"It works by managing the actions in a queue and only
  calling `select_actions` when the queue is empty."*
- Line 110–113, temporal-ensemble branch: `if self.config.temporal_ensemble_coeff is not None:
  actions = self.predict_action_chunk(batch); action = self.temporal_ensembler.update(actions)`.
  This would run the **full network on every step**, which costs more in CPU time, not less. It is
  off for us (`null`).
- Line 117–122, refill:
  ```
  117  if len(self._action_queue) == 0:
  118      actions = self.predict_action_chunk(batch)[:, : self.config.n_action_steps]
  ...
  122      self._action_queue.extend(actions.transpose(0, 1))
  123  return self._action_queue.popleft()
  ```
  The refill runs on the caller's thread, inside the control step. It only happens when the queue is
  empty, and 80 of the 100 predicted actions are thrown away.
- Line 126–136, `predict_action_chunk`: a plain `self.model(batch)[0]`, with no threading.
- Line 168–211, `ACTTemporalEnsembler`: weights `exp(-temporal_ensemble_coeff * i)`. The docstring
  says the original ACT value is 0.01.
- ACT does not override `supports_rtc`, so the base `PreTrainedPolicy.supports_rtc` applies.
  It returns `False` (`$L/policies/pretrained.py:252–254`).

**Verdict: the default is synchronous.** No ACT code path computes in the background.

## (2) `async_inference`: background chunks over gRPC

**Does it compute chunks while the robot executes? Yes.**
- `robot_client.py:485–512` `async_client`: starts
  `action_receiver_thread = threading.Thread(target=client.receive_actions, daemon=True)` (498),
  then *"The main thread runs the control loop"* (`client.control_loop`, 506).
- `robot_client.py:458–482` `control_loop`: every tick it (1) pops and sends an action
  `if self.actions_available()`, and (2) sends an observation
  `if self._ready_to_send_observation()`. It then sleeps `environment_dt - elapsed`.
- Inference runs in a **separate server process**. `policy_server.py:322–328` `_get_action_chunk`
  calls `self.policy.predict_action_chunk(observation)` and returns
  `chunk[:, : self.actions_per_chunk, :]`. It bypasses `select_action` and its 20-step queue.

**What triggers the next request:** `robot_client.py:403–406`
```
def _ready_to_send_observation(self):
    with self.action_queue_lock:
        return self.action_queue.qsize() / self.action_chunk_size <= self._chunk_size_threshold
```
`chunk_size_threshold` defaults to 0.5 (`configs.py:137`) and must be in [0, 1] (`configs.py:173`).
- The server can also drop observations. `policy_server.py:268–293` `_obs_sanity_checks` /
  `_enqueue_observation` skips an observation that is "too similar to last obs predicted", measured
  only in joint space with `atol=1` (`helpers.py:281–298`), unless the observation is `must_go`.
- `must_go` is set after each received chunk (`robot_client.py:336`).
- Inference is paced on the server at `inference_latency` (default `1/fps`, `constants.py:18–19`).

**How overlapping chunks are aggregated:** `robot_client.py:224–267` `_aggregate_action_queues`.
- Every action carries an absolute `timestep`.
- Incoming actions with `timestep <= latest_action` (already executed) are skipped (247–248).
- New timesteps are added directly.
- Timesteps that are already queued are blended with `aggregate_fn(old, new)`.
- `configs.py:29–33` `AGGREGATE_FUNCTIONS`: `weighted_average` = `0.3*old + 0.7*new` (the default,
  `configs.py:141–143`), `latest_only`, `average`, `conservative` = `0.7*old + 0.3*new`.
- The server timestamps the chunk with `_time_action_chunk` (`policy_server.py:312–320`):
  `timestep=i_0 + i`. Stale actions predicted from an older observation are therefore dropped by
  timestep, so latency is compensated automatically.

**Does it need gRPC or a server process? Yes, both.**
- `robot_client.py:47` `import grpc`, 110–113 `grpc.insecure_channel(...)` /
  `services_pb2_grpc.AsyncInferenceStub`.
- `policy_server.py:426–434` `grpc.server(futures.ThreadPoolExecutor(max_workers=4))`,
  `add_insecure_port`, `wait_for_termination()`.
- The policy specs travel by `pickle.loads(request.data)` (`policy_server.py:125`).
- **`grpc` is not installed in our `lerobot_env`**: running
  `.venv/bin/python -c "import grpc"` gives `ModuleNotFoundError`. Using it means a new dependency
  (`grpcio`) and running two processes on the same Jetson.

**Does it work with any lerobot Robot, including our plugin? Only structurally.**
- `constants.py:29` `SUPPORTED_ROBOTS = ["so100_follower", "so101_follower", "bi_so_follower",
  "omx_follower"]`. However, the check is commented out (`robot_client.py:488–490`: *"TODO: Assert
  if checking robot support is still needed with the plugin system"*). `register_third_party_plugins()`
  runs at line 516, so `--robot.type=inchworm_follower` (`config_inchworm.py:10`) would construct
  through `make_robot_from_config` (95). ACT is in `SUPPORTED_POLICIES` (`constants.py:26`).
- **It would bypass our runner.** `control_loop_action` (370–399) sends the queued action straight
  to `self.robot.send_action(...)`. None of `PolicyAttempt.run`'s logic runs: the jump guard, the
  per-step `max_deg_s` rate clamp against `last_goal`, the finish rule, the owner `s`/`q` key,
  `max_s`, the trace, or the `MappedPolicy` / `JointOffsetMap` arm transfer. The client also
  `robot.connect()`s by itself (96), outside our handoff and state checks in `policy_handoff.py`.
  Only the protections inside `InchwormFollower.send_action` would still apply.
- The blending (0.3/0.7) changes the commanded trajectory compared with what we evaluated offline.

## (3) Normal record and eval loops

- `$L/scripts/lerobot_record.py` has no policy at all. Line 17: *"tool — no policy inference. For
  deploying trained policies, use"* `lerobot-rollout`. Lines 197 and 431–436 reject policy and
  `eval_` use. Its only threads are dataset `encoder_threads` and `image_writer_threads` (37, 421,
  425, 447, 451). The loop is synchronous with `precise_sleep(max(sleep_time_s, 0.0))` (356). On an
  overrun it just warns: *"Record loop is running slower … 2) Policy inference taking too long"*
  (350–353).
- `$L/rollout/configs.py:233–234`: *"Inference backend (polymorphic: --inference.type=sync|rtc)"*,
  `inference: InferenceEngineConfig = field(default_factory=SyncInferenceConfig)`. **The default
  is sync.**
- `$L/rollout/inference/sync.py:84`: `"SyncInferenceEngine started (inline mode — no background
  thread)"`. `get_action` calls `self._policy.select_action(observation)` (line 115). This is
  the same call as our `LeRobotBackend.step`, so the same 1-in-20 stall would happen.
- `$L/rollout/inference/factory.py:66–74` `RTCInferenceConfig`: *"Real-Time Chunking: async policy
  inference in a background thread"*, `queue_threshold: int = 30`.
  `$L/rollout/inference/rtc.py:17`: *"A background thread produces action chunks asynchronously via
  `policy.predict_action_chunk`"*. It needs `supports_rtc()` and
  `predict_action_chunk(..., inference_delay=, prev_chunk_left_over=)` (rtc.py:66–79).
  `$L/rollout/context.py:259–265` raises *"RTC inference is not supported by policy type … Use
  '--inference.type=sync' instead."* Only pi0, pi05, smolvla, groot, evo1 and molmoact2 implement
  `supports_rtc`. **ACT does not, so RTC cannot be used with our checkpoint.**

## Conclusion for the owner

Computing the next chunk in the background **is a feature we would be adding** for ACT.
- LeRobot's normal paths (`select_action`, the sync rollout engine, and the old record loop) are
  synchronous and stall on refill exactly as ours does.
- LeRobot's background options are either the separate gRPC client/server stack
  (`async_inference`, opt-in, not installed, and it bypasses our runner), or RTC, which ACT does not
  support.

## Recommendation: least new code to reach 30 Hz

| Option | New code | Effect | Concerns |
|---|---|---|---|
| A. `async_inference` as is | ~0 lines of ours, but a new `grpcio` dependency, 2 processes, CLI config | Background chunks with latency compensation | **Bypasses `PolicyAttempt`**: our jump guard, rate cap, finish rule, keys, arm mapping and handoff checks are lost. Getting them back means changing the plugin or forking `RobotClient`. Not the least code once safety is counted. Blending changes behaviour. |
| B. Small background thread in our code | ~40–70 lines (one class) | Every control step only pops; the network runs off-thread | Threading and latency bookkeeping; CPU contention on the Jetson (see below) |
| C. Lower per-chunk cost | 0 lines (config) to a few lines | Smaller stall | Hard to get 80–96 ms under 33 ms without re-export or quantisation. Raising `n_action_steps` (e.g. 20 → 50 or 100) only makes the stall less frequent: 20/0.72 s ≈ 27.9 Hz becomes ≈ 29.2 Hz at 50 and ≈ 29.8 Hz at 100. It also changes policy behaviour (more open-loop), which needs re-evaluation and owner approval. |

**Recommended: B, implemented in `LeRobotBackend`** (`src/inchworm/policy_runner.py`), keeping the
`step(Observation) -> {joint: deg}` contract.
- This keeps `PolicyAttempt.run` and its safety layers byte-for-byte unchanged.
- It mirrors what `async_inference` does with `chunk_size_threshold` and timesteps, without gRPC.
- Sketch:
  - Keep our own queue of postprocessed actions tagged with step indices.
  - When fewer than K actions remain (K ≥ ceil(96 ms / 33 ms) + margin ≈ 5), hand the current
    frame and state to one worker thread. The worker runs `_pre` → `predict_action_chunk` →
    `_post` on the whole chunk.
  - On arrival, drop the actions whose step index has already passed (the same latency rule as
    `robot_client.py:247`). Then either replace the queue tail (`latest_only`, closest to what we
    evaluated) or blend.
  - If the queue ever runs dry, fall back to a blocking call. That is today's behaviour, so it adds
    no new failure mode.
  - Postprocessing the whole chunk at once is what `policy_server.py` and RTC already do.
- Decisions for the owner or orchestrator before building B:
  - The aggregation rule (latest-only vs. weighted).
  - The value of K.
  - Whether the first chunk stays blocking. It should: it happens before any motion.
- Main technical risk: Jetson CPU contention. A background `torch` forward pass competes with the
  control thread (camera read, mesh check). Measure this with `latency_measure.py` or an offline
  replay before the first live run, then do a dry run per rule 5.

If even ~50 lines are too much: option C with `n_action_steps` 100 gives ~29.8 Hz with zero code,
but it is a change to policy behaviour (open-loop for 3.3 s), not a timing fix.

## Safety layers that stay in our code either way (with B or C)

All of these stay outside the learned policy:
- **Jump guard:** `PolicyAttempt.run`, `policy_handoff.py:142–150`, `max_jump_deg` → stop and
  hold.
- **Speed cap:** per-step clamp `goal = last + clip(target - last, ±max_deg_s/fps)` against
  `r.core.last_goal` (`policy_handoff.py:151–156`), plus the core's `speed_ticks_s` cap in the
  plugin.
- **Joint limits:** clamped inside the follower core, soft ∩ EEPROM ∩ hardware.yaml, 10 ticks inside
  (`inchworm_follower.py:19`, 263–265, 314–327). The handoff mesh/collision gate also stays (collision
  hold, ~412–440), subject to the AA-57 `policy_no_mesh_check` setting after catch-up.
- **Stop file:** `robot/out/STOP_TELEOP` raises KeyboardInterrupt (`inchworm_follower.py:20, 51`),
  plus the owner `s`/`q` key and `max_s` in `PolicyAttempt.run`.
- **Gripper:** never in the action space. `PolicyAttempt.run` refuses any action whose keys are not
  joints 1–5 (`policy_handoff.py:138`, `if set(act) != set(tl.J5): raise`). The bus only sync-writes joints 1–5 and rejects
  gripper writes (`inchworm_follower.py:132`). The other arm's gripper stays LOCKED, checked at
  connect.

With option A, the guards in `PolicyAttempt` (jump guard, per-step rate clamp, finish rule, keys,
arm mapping) would **not** run unless they were moved into `InchwormFollower.send_action`. This is
the main reason not to use A as is.
