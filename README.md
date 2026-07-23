# EV Engine Simulator

A tiny, dependency-free web app that gives an electric car a synthesized engine
voice. It reads the phone's **accelerometer** (DeviceMotion) and revs a
procedurally synthesized engine to your real-world acceleration — combustion or
exotic (jet, spaceship, warp). Ships with a **headless simulator** that tests the
hard part — the accelerometer→throttle mapping — plus the audio synthesis, with
no browser or device required.

## The hard part: accelerometer → throttle

A phone is mounted at an unknown orientation, and the car brakes, corners and
hits bumps. Turning that into a believable throttle without a gyro or GPS is the
real problem. `engine-core.js` (`MotionMapper`) does it:

1. **Gravity** — settle fast at startup (car ~at rest), then hold with a long
   time constant. The mount is fixed, so gravity in the device frame is ~constant;
   a slow hold means multi-second accelerations aren't mistaken for gravity and
   subtracted away (the classic low-pass trap).
2. **Horizontal projection** — remove the gravity/vertical component, so road
   bumps don't drive the throttle.
3. **Forward axis via streaming PCA** — accelerating/braking varies more along
   the direction of travel than sideways, so the principal axis of horizontal
   acceleration is "forward." Once locked in confidently it's **frozen** (the
   mount can't move), so a hard slalom can't re-point it and read as throttle.
4. **Sign lock** — the first sustained launch defines +forward = accelerate, so
   braking reads as engine-braking rather than a phantom rev.
5. **Smoothing** — asymmetric (fast attack / slow release) + deadzone.

### GPS speed fusion

Accelerometer-only can't tell you're moving: at constant velocity acceleration
is ~0, so revs sag to idle at a steady 70 mph. `SpeedEstimator` fuses the phone's
**GPS Doppler speed** (`coords.speed`, accurate and absolute but ~1 Hz and laggy)
with accelerometer dead-reckoning (fast but drifts) in a complementary filter:
integrate forward accel between fixes, correct toward each GPS reading. The
gearbox then runs off real speed, so **cruising holds revs**. `node
sim/test-speed.js` proves it — GPS-fused cruise holds ~4600 rpm where accel-only
sags to ~1300, tracks true speed to ~1 m/s, survives a 4 s GPS dropout on dead
reckoning, and stays smooth under noisy fixes. Falls back to the accel-only feel
when GPS is unavailable.

### It's verified, not vibes

`node sim/test-motion.js` synthesizes DeviceMotion streams for scripted drive
cycles at arbitrary mount orientations and asserts the throttle behaves — no
phone needed:

| Scenario | Asserts |
|---|---|
| Idle, flat & 32° tilted mount | no phantom throttle (gravity rejection) |
| Standing launch | throttle rises, sign locks, rpm climbs |
| Accelerate → brake | braking reads as brake, never a rev; rpm falls |
| Launch → hard slalom | corner stays lateral (throttle ≈ 0) |
| Rough road (8 Hz bumps) | throttle stays smooth (low jerk) |
| Full drive cycle | throttle correlates with true forward accel |
| Same launch at 4 mount yaws | orientation-independent (< 0.15 spread) |

`node sim/plot.js` renders those traces to `sim/out/mapping.svg` for eyeballing.

## The engine synthesis

`voicing()` maps rpm/throttle to synth parameters rendered by native Web Audio
nodes. Combustion modes fire a pulse train at `firing = rpm/60 · cylinders/2`
through a harmonic-rich `PeriodicWave`, plus sub-octave rumble, detuned rasp,
throttle-scaled filtered noise, and a low-pass that opens under load. Exotic
modes reuse the same rpm/speed state but synthesize differently.

**Modes:** Inline-4 · V8 · V-Twin · 2-Stroke Kart · Jet Turbine · Spaceship · Warp Drive.

- `node sim/test-audio.js` — parameter-level checks (pitch/loudness monotonic,
  firing freq exact, modes distinct).
- `node sim/render-audio.js` — a headless DSP renderer (band-limited additive
  oscillators + RBJ biquads, mirroring the Web Audio graph) renders a rev sweep
  per mode to `sim/out/*.wav` and asserts each is non-silent, un-clipped, and
  brightens with revs (spectral centroid).

## Run

```bash
npm test          # motion tests + audio tests + render wavs + plot svg
npm run serve     # http://localhost:8080  (open index.html)
```

The app is `index.html` + `engine-core.js` — no build, no dependencies.

For the **accelerometer on a phone**, iOS/Safari needs a secure context (HTTPS
or `localhost`) and a permission tap. Expose it over HTTPS with `npx localtunnel
--port 8080` (or `ngrok`/`cloudflared`), open on the phone, mount it, tap Start,
and drive. Without HTTPS the on-screen **hold-to-rev pedal** drives everything.

## Files

- `index.html` — the app UI + Web Audio layer.
- `engine-core.js` — pure, shared logic (`MotionMapper`, `DriveModel`, modes, `voicing`).
- `sim/` — headless simulator: `motion.js` (sensor generator), `test-motion.js`,
  `test-audio.js`, `render-audio.js`, `plot.js`. Outputs land in `sim/out/`.
