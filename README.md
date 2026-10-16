# gpio4tuner_client

Standalone workshop-side client for the GPIO4 tuner daemon. It measures a
continuous Raspberry Pi carrier in Gqrx RAW I/Q and iteratively sets the
daemon's `txcorr` value. It does not depend on Codex or another assistant.

This is a workshop tool for an authorized, conducted RF setup. It does not
start unless the operator explicitly runs `calibrate`.

## Current calibration loop

1. Connect to the Gqrx Remote Control server and the Pi's `gpio4tuningd`.
2. Verify that the tuner daemon is idle and GPIO4 is free.
3. Set the Gqrx VFO to the requested RF target yourself. The client verifies
   the frequency and refuses a mismatch; it does not retune the receiver. Set
   its demodulator to `RAW / 48000`; check that the UDP stream is arriving at 48 k complex
   samples/s as signed little-endian 16-bit I/Q pairs.
4. Set the daemon base frequency to the target and start a continuous
   uncorrected carrier.
5. Estimate its frequency offset from overlapping 8192-sample FFT windows.
   A small positive `txcorr` probe verifies the IQ sign and the daemon's
   frequency-step response; the probe is then removed.
6. Adjust `txcorr` by the measured signed error and issue `TC`. The daemon
   keeps its TX request active and retunes to `F + txcorr`. Repeat until the
   measured residual is inside tolerance and stable across successive
   windows.
7. Measure the corrected carrier once more. On success, save `txcorr` and
   stop the carrier by default. `--keep-carrier` leaves it on.

The loop does not tune the Gqrx VFO during calibration. Set it to the desired
RF frequency and leave it there as the reference; only the Pi transmitter's
correction changes. A post-retune settling interval is followed by discarding
queued UDP samples, so old IQ from the previous frequency is not mistaken for
the new carrier. The optional USB/1200 Hz check is not part of the current
calibration command.

Each retune asks `librpitx` to recreate its frequency modulator, so a brief
transition can occur at a correction step even though the daemon's TX request
remains active. This is not a phase-continuous DDS sweep.

On failure or Ctrl-C, the client requests `T 0` and restores the daemon's
previous base frequency and stored correction. Gqrx remains at the operator-
selected target in RAW/48k mode so its state is visible and reproducible.

The default search window is ±22 kHz, the default allowed accumulated TX
correction is ±5 kHz, and the default lock tolerance is ±10 Hz. The allowed
TX correction excursion can be raised to ±200 kHz. The Gqrx VFO remains fixed
at the target; the client does not scan or recenter the receiver. The ±200 kHz
option is a safety ceiling, not the normal correction distance.

## Install and run

Requirements: Python 3.10 or later, NumPy, network access to Gqrx and the Pi
daemon, and Gqrx configured to send stereo UDP to the workshop host on port
7355.

```sh
python3 -m pip install --user -r requirements.txt
python3 client.py calibrate --target-hz 433100000
```

Explicit configuration example (replace the TEST-NET addresses with the
addresses of your own Gqrx host and Raspberry Pi):

```sh
python3 client.py calibrate \
  --target-hz 433100000 \
  --gqrx-host 192.0.2.10 --gqrx-port 37356 \
  --tuner-host 192.0.2.20 --tuner-port 37357 \
  --udp-bind 0.0.0.0 --udp-port 7355 \
  --max-excursion-hz 5000
```

Successful calibration stops the carrier. To leave the corrected carrier on:

```sh
python3 client.py calibrate --target-hz 433100000 --keep-carrier
```

The daemon protocol is unauthenticated; use it only on a trusted lab network.
The daemon and its command meanings are documented in the
[`gpio4tuning` project](https://github.com/erixabo/gpio4tuning).
