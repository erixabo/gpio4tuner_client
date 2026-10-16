#!/usr/bin/env python3
"""Closed-loop GPIO4 TX calibration using Gqrx RAW 48 kHz IQ."""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from dataclasses import dataclass

import numpy as np

IQ_RATE = 48_000
FFT_SIZE = 8_192
FFT_HOP = 4_096


class ProtocolError(RuntimeError):
    pass


def tcp_lines(host: str, port: int, command: str, timeout: float = 3.0,
              stop_on_rprt: bool = True, max_lines: int = 0) -> list[str]:
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(command.encode("ascii") + b"\n")
        stream = sock.makefile("r", encoding="ascii", errors="replace", newline="\n")
        lines: list[str] = []
        while True:
            line = stream.readline()
            if not line:
                break
            line = line.strip()
            if line:
                lines.append(line)
            if stop_on_rprt and line.startswith("RPRT "):
                break
            if max_lines and len(lines) >= max_lines:
                break
        return lines


class Tuner:
    def __init__(self, host: str, port: int, timeout: float):
        self.host, self.port, self.timeout = host, port, timeout

    def command(self, command: str) -> list[str]:
        lines = tcp_lines(self.host, self.port, command, self.timeout)
        if not lines:
            raise ProtocolError(f"A tuner daemon nem válaszolt erre: {command}")
        if lines[-1].startswith("RPRT 1"):
            raise ProtocolError(f"Tuner daemon hiba: {' | '.join(lines)}")
        if lines[-1] != "RPRT 0":
            raise ProtocolError(f"Hibás tuner válasz: {' | '.join(lines)}")
        return lines[:-1]

    def get_int(self, command: str) -> int:
        lines = self.command(command)
        if not lines:
            raise ProtocolError(f"Üres válasz: {command}")
        return int(lines[0])

    def set_frequency(self, hz: int) -> None:
        self.command(f"F {hz}")

    def step(self, delta_hz: int) -> int:
        self.command(f"F+ {delta_hz:+d}")
        return self.get_int("f")

    def set_correction(self, hz: int) -> None:
        self.command(f"c={hz:+d}")

    def start_raw(self) -> None:
        self.command("T 1")

    def start_corrected(self) -> None:
        self.command("TC")

    def stop(self) -> None:
        self.command("T 0")


class Gqrx:
    def __init__(self, host: str, port: int, timeout: float):
        self.host, self.port, self.timeout = host, port, timeout

    def command(self, command: str, getter_lines: int = 0) -> list[str]:
        lines = tcp_lines(self.host, self.port, command, self.timeout,
                          stop_on_rprt=(getter_lines == 0), max_lines=getter_lines)
        if not lines:
            raise ProtocolError(f"A Gqrx nem válaszolt erre: {command}")
        if getter_lines == 0 and lines[-1] != "RPRT 0":
            raise ProtocolError(f"Gqrx parancshiba: {' | '.join(lines)}")
        return lines[:-1] if getter_lines == 0 else lines

    def frequency(self) -> int:
        return int(self.command("f", getter_lines=1)[0])

    def mode(self) -> tuple[str, int]:
        lines = self.command("m", getter_lines=2)
        if len(lines) < 2:
            raise ProtocolError(f"Nem értelmezhető Gqrx módválasz: {lines}")
        return lines[0], int(lines[1])

    def set_frequency(self, hz: int) -> None:
        self.command(f"F {hz}")

    def set_raw_iq(self) -> None:
        self.command(f"M RAW {IQ_RATE}")
        mode, bandwidth = self.mode()
        if mode.upper() != "RAW" or bandwidth != IQ_RATE:
            raise ProtocolError(f"Gqrx nem RAW/48k módban van: {mode} {bandwidth}")


class IQStream:
    def __init__(self, bind: str, port: int, source: str | None, timeout: float):
        self.source = source
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        self.sock.bind((bind, port))
        self.sock.settimeout(timeout)

    def close(self) -> None:
        self.sock.close()

    def flush(self) -> int:
        previous_timeout = self.sock.gettimeout()
        self.sock.setblocking(False)
        discarded = 0
        try:
            while True:
                try:
                    packet, _address = self.sock.recvfrom(65536)
                    discarded += len(packet) // 4
                except BlockingIOError:
                    break
        finally:
            self.sock.settimeout(previous_timeout)
        return discarded

    def read(self, count: int) -> np.ndarray:
        parts: list[np.ndarray] = []
        have = 0
        deadline = time.monotonic() + max(4.0, 2.0 * count / IQ_RATE + 1.0)
        while have < count:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"RAW IQ adatfolyam timeout: {have}/{count} minta")
            try:
                packet, address = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            if self.source and address[0] != self.source:
                continue
            if len(packet) < 8:
                continue
            packet = packet[:len(packet) - len(packet) % 4]
            iq = np.frombuffer(packet, dtype="<i2").astype(np.float32).reshape(-1, 2)
            samples = (iq[:, 0] + 1j * iq[:, 1]) / 32768.0
            parts.append(samples)
            have += len(samples)
        joined = np.concatenate(parts)
        return joined[-count:]

    def check_rate(self, seconds: float = 1.5) -> float:
        self.flush()
        started = time.monotonic()
        deadline = started + seconds
        frames = 0
        self.sock.settimeout(0.05)
        while time.monotonic() < deadline:
            try:
                packet, address = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            if self.source and address[0] != self.source:
                continue
            frames += len(packet) // 4
        elapsed = time.monotonic() - started
        self.sock.settimeout(self.timeout)
        measured = frames / max(elapsed, 1e-3)
        if not IQ_RATE * 0.80 <= measured <= IQ_RATE * 1.20:
            raise ProtocolError(f"A Gqrx UDP adatsebessége nem 48 kS/s: {measured:.0f} komplex minta/s")
        return measured


@dataclass
class Estimate:
    offset_hz: float
    snr_db: float
    peak_dbfs: float


class ToneEstimator:
    def __init__(self, sample_rate: int = IQ_RATE, search_hz: float = 8_000.0):
        self.rate = sample_rate
        self.search_hz = search_hz
        self.window = np.hanning(FFT_SIZE).astype(np.float32)

    def candidates(self, samples: np.ndarray, limit: int = 32) -> list[Estimate]:
        if len(samples) < FFT_SIZE + 2 * FFT_HOP:
            raise ValueError("Kevés IQ minta frekvenciabecsléshez")
        spectra = []
        for start in range(0, len(samples) - FFT_SIZE + 1, FFT_HOP):
            frame = samples[start:start + FFT_SIZE]
            spectrum = np.fft.fftshift(np.fft.fft(frame * self.window))
            spectra.append(np.abs(spectrum) ** 2)
        power = np.mean(spectra, axis=0)
        freqs = (np.arange(FFT_SIZE) - FFT_SIZE // 2) * self.rate / FFT_SIZE
        search = np.flatnonzero(np.abs(freqs) <= self.search_hz)
        noise_mask = np.ones(len(power), dtype=bool)
        noise_mask &= np.abs(freqs) <= self.search_hz
        noise = float(np.median(power[noise_mask]))
        coherent_gain = float(np.sum(self.window))
        tiny = np.finfo(np.float64).tiny
        mask = np.zeros(len(power), dtype=bool)
        mask[search] = True
        local = np.flatnonzero(mask[1:-1] & (power[1:-1] > power[:-2]) &
                               (power[1:-1] >= power[2:])) + 1
        local = sorted(local, key=lambda index: power[index], reverse=True)[:limit]
        results = []
        for peak_i in local:
            if peak_i < 1 or peak_i >= FFT_SIZE - 1:
                continue
            side = noise_mask.copy()
            side[max(0, peak_i - 4):min(len(power), peak_i + 5)] = False
            local_noise = float(np.median(power[side]))
            y0, y1, y2 = np.log(np.maximum(power[peak_i - 1:peak_i + 2], tiny))
            denom = y0 - 2.0 * y1 + y2
            fraction = 0.0 if abs(denom) < 1e-15 else 0.5 * (y0 - y2) / denom
            fraction = float(np.clip(fraction, -0.5, 0.5))
            offset = float(freqs[peak_i] + fraction * self.rate / FFT_SIZE)
            snr = 10.0 * math.log10(max(float(power[peak_i]), tiny) / max(local_noise, tiny))
            peak_amplitude = math.sqrt(float(power[peak_i])) / coherent_gain
            peak_dbfs = 20.0 * math.log10(max(peak_amplitude, 1e-12))
            results.append(Estimate(offset, snr, peak_dbfs))
        return results

    def estimate(self, samples: np.ndarray) -> Estimate:
        candidates = self.candidates(samples)
        if not candidates:
            raise ValueError("Nem találtam vivőjelöltet az IQ-ablakban")
        return max(candidates, key=lambda candidate: candidate.snr_db)


def read_estimate(stream: IQStream, estimator: ToneEstimator,
                  samples: int, center_hz: int) -> tuple[Estimate, float]:
    iq = stream.read(samples)
    result = estimator.estimate(iq)
    return result, center_hz + result.offset_hz


def read_candidates(stream: IQStream, estimator: ToneEstimator,
                    samples: int) -> list[Estimate]:
    return estimator.candidates(stream.read(samples))


def nearest_candidate(candidates: list[Estimate], expected_hz: float,
                      min_snr_db: float, max_distance_hz: float) -> Estimate:
    qualified = [candidate for candidate in candidates
                 if candidate.snr_db >= min_snr_db and
                 abs(candidate.offset_hz - expected_hz) <= max_distance_hz]
    if not qualified:
        nearby = ", ".join(f"{item.offset_hz:+.1f}Hz/{item.snr_db:.1f}dB"
                            for item in sorted(candidates, key=lambda c: c.snr_db, reverse=True)[:6])
        raise ProtocolError(f"A követett vivő nem található {expected_hz:+.1f} Hz körül "
                            f"({nearby or 'nincs csúcsjelölt'})")
    return min(qualified, key=lambda candidate: abs(candidate.offset_hz - expected_hz))


def probe_candidates(before_candidates: list[Estimate], after_candidates: list[Estimate],
                     args: argparse.Namespace) -> list[tuple[float, Estimate, Estimate, float]]:
    moving_pairs = []
    for before in before_candidates:
        if before.snr_db < args.min_snr_db:
            continue
        for after in after_candidates:
            movement = after.offset_hz - before.offset_hz
            if abs(abs(movement) - args.probe_hz) <= args.probe_tolerance_hz:
                score = min(before.snr_db, after.snr_db)
                moving_pairs.append((score, before, after, movement))
    return sorted(moving_pairs, key=lambda item: item[0], reverse=True)


def acquire_moving_carrier(stream: IQStream, estimator: ToneEstimator,
                           tuner: Tuner, gqrx: Gqrx, target: int,
                           args: argparse.Namespace) -> tuple[Estimate, int, int, float]:
    center = target  # Gqrx stays fixed on the requested RF frequency throughout.
    before_candidates = read_candidates(stream, estimator, args.window_samples)
    tuner.set_correction(args.probe_hz)
    tuner.start_corrected()
    time.sleep(args.settle_s)
    stream.flush()
    after_candidates = read_candidates(stream, estimator, args.window_samples)
    moving_pairs = probe_candidates(before_candidates, after_candidates, args)
    tuner.set_correction(0)
    tuner.start_corrected()
    time.sleep(args.settle_s)
    stream.flush()
    restored_candidates = read_candidates(stream, estimator, args.window_samples)
    confirmed = []
    for score, before, _after, movement in moving_pairs:
        try:
            restored = nearest_candidate(restored_candidates, before.offset_hz,
                                         args.min_snr_db, args.probe_tolerance_hz)
            confirmed.append((score, restored, movement))
        except ProtocolError:
            continue
    if not confirmed:
        raise ProtocolError(
            f"A célfrekvencia körüli ±{args.search_hz:.0f} Hz-es RAW-IQ ablakban "
            "nem találtam olyan vivőt, amely a TXCORR próbára elmozdult és visszatért")
    _score, restored, movement = max(confirmed, key=lambda item: item[0])
    sign = 1 if movement > 0 else -1
    print(f"Saját mozgó vivő: Gqrx-közép {center} Hz, IQ {restored.offset_hz:+.2f} Hz, "
          f"csúcs/zaj {restored.snr_db:.1f} dB; +{args.probe_hz} Hz TXCORR-próba "
          f"{movement:+.2f} Hz IQ-elmozdulás. RF-eltérés = IQ × {sign:+d}", flush=True)
    iq_slope = movement / args.probe_hz
    return restored, sign, center, iq_slope


def run_calibration(args: argparse.Namespace) -> int:
    tuner = Tuner(args.tuner_host, args.tuner_port, args.timeout)
    gqrx = Gqrx(args.gqrx_host, args.gqrx_port, args.timeout)
    estimator = ToneEstimator(search_hz=args.search_hz)
    iq = IQStream(args.udp_bind, args.udp_port, args.gqrx_host, args.timeout)
    target = args.target_hz
    original_corr = tuner.get_int("c")
    original_txfreq = tuner.get_int("f")
    original_gqrx = gqrx.frequency()
    was_tx = tuner.get_int("t")
    if was_tx:
        iq.close()
        raise ProtocolError("A tuner daemon vivője már aktív; kalibrációt nem indítottam.")
    gpio = tuner.command("g")
    if not gpio or "GPIO4 FREE" not in gpio[0]:
        iq.close()
        raise ProtocolError(f"A GPIO4 nem szabad: {' '.join(gpio)}")

    configured_gqrx = False
    started = False
    success = False
    correction = 0
    max_delta = args.max_excursion_hz
    try:
        if original_gqrx != target:
            raise ProtocolError(
                f"A Gqrx VFO {original_gqrx} Hz-en áll, a kért cél {target} Hz. "
                "A kliens nem hangolja át a vevőt; állítsd a vevőt a célfrekvenciára, majd indítsd újra."
            )
        gqrx.set_raw_iq()
        configured_gqrx = True
        print(f"Gqrx: {target} Hz, RAW IQ {IQ_RATE} komplex minta/s; UDP {args.udp_bind}:{args.udp_port}")
        print("RAW IQ adatfolyam ellenőrzése…", flush=True)
        measured_rate = iq.check_rate()
        print(f"UDP-adatsebesség: {measured_rate:.0f} komplex minta/s", flush=True)
        iq.read(FFT_SIZE + 2 * FFT_HOP)  # drain/confirm before enabling the carrier
        tuner.set_frequency(target)
        tuner.set_correction(0)
        tuner.start_raw()
        started = True
        print(f"Folyamatos hangolóvivő elindult: nyers F={target} Hz", flush=True)
        time.sleep(args.settle_s)
        iq.flush()

        # Identify our own tone by its movement during a small TXCORR probe,
        # scanning overlapping RAW-IQ windows if its initial offset is large.
        restored, iq_sign, rx_center, iq_slope = acquire_moving_carrier(
            iq, estimator, tuner, gqrx, target, args)
        rf_slope = iq_sign * iq_slope
        if not 0.5 <= rf_slope <= 1.5:
            raise ProtocolError(f"A TX léptetési arány irreális: {rf_slope:.4f} RF Hz / parancs-Hz")
        print(f"Mért TX-léptetési arány: {rf_slope:.5f} RF Hz / korrekció-Hz", flush=True)

        stable = 0
        last_values: list[float] = []
        expected_offset = restored.offset_hz
        for iteration in range(1, args.max_iterations + 1):
            candidates = read_candidates(iq, estimator, args.window_samples)
            measured = nearest_candidate(candidates, expected_offset, args.min_snr_db,
                                         args.track_tolerance_hz)
            error_hz = (rx_center - target) + iq_sign * measured.offset_hz
            last_values.append(error_hz)
            last_values = last_values[-3:]
            spread = max(last_values) - min(last_values)
            print(f"{iteration:02d}: eltérés {error_hz:+.2f} Hz; csúcs/zaj {measured.snr_db:.1f} dB; "
                  f"TXCORR {correction:+d} Hz", flush=True)
            if abs(error_hz) <= args.tolerance_hz:
                stable = stable + 1 if len(last_values) >= 3 and spread <= args.stability_hz else 0
                if stable >= 2:
                    success = True
                    break
                continue
            stable = 0
            delta = int(round(-error_hz / rf_slope))
            delta = max(-args.max_step_hz, min(args.max_step_hz, delta))
            if delta == 0:
                delta = -1 if error_hz > 0 else 1
            next_correction = correction + delta
            if abs(next_correction) > max_delta:
                raise ProtocolError(f"A szükséges korrekció túllépné a ±{max_delta} Hz-es határt")
            correction = next_correction
            tuner.set_correction(correction)
            tuner.start_corrected()
            expected_offset += iq_slope * delta
            time.sleep(args.settle_s)
            iq.flush()

        if not success:
            raise ProtocolError(f"{args.max_iterations} mérési lépés alatt nem állt be a ±{args.tolerance_hz} Hz-es tűrés")

        # The daemon's TC updates an already active TX worker to base F + c.
        # Keep it active and do one fresh measurement before declaring lock.
        time.sleep(args.settle_s)
        iq.flush()
        final_candidates = read_candidates(iq, estimator, args.window_samples)
        final = nearest_candidate(final_candidates, expected_offset,
                                  args.min_snr_db, args.track_tolerance_hz)
        final_error = (rx_center - target) + iq_sign * final.offset_hz
        print(f"Végellenőrzés: {target + final_error:.2f} Hz, eltérés {final_error:+.2f} Hz, "
              f"TXCORR {correction:+d} Hz, {final.snr_db:.1f} dB csúcs/zaj", flush=True)
        if abs(final_error) > args.tolerance_hz:
            raise ProtocolError("A korrigált TC vivő végellenőrzése kívül esik a tűrésen")
        print("Kalibráció sikeres.")
        if not args.keep_carrier:
            tuner.stop()
            started = False
            print("A kalibráló vivő leállítva.")
        else:
            print("A korrigált folyamatos vivő bekapcsolva maradt (--keep-carrier).")
        return 0
    except BaseException:
        # Fail closed: stop the tuner output and restore the previous persisted
        # correction and raw base frequency. Never leave a failed calibration TX on.
        if started:
            try:
                tuner.stop()
            except Exception as stop_error:
                print(f"FIGYELEM: a tuner T 0 hibázott: {stop_error}", file=sys.stderr)
        try:
            tuner.set_frequency(original_txfreq)
            tuner.set_correction(original_corr)
        except Exception as restore_error:
            print(f"FIGYELEM: tunerállapot-visszaállítás sikertelen: {restore_error}", file=sys.stderr)
        raise
    finally:
        iq.close()
        if configured_gqrx:
            print(f"Gqrx VFO változatlanul {original_gqrx} Hz-en maradt.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    cal = sub.add_parser("calibrate", help="folyamatos vivős RAW-IQ kalibráció")
    cal.add_argument("--target-hz", type=int, default=433_100_000)
    cal.add_argument("--tuner-host", default="127.0.0.1")
    cal.add_argument("--tuner-port", type=int, default=37357)
    cal.add_argument("--gqrx-host", default="127.0.0.1")
    cal.add_argument("--gqrx-port", type=int, default=37356)
    cal.add_argument("--udp-bind", default="0.0.0.0")
    cal.add_argument("--udp-port", type=int, default=7355)
    cal.add_argument("--timeout", type=float, default=3.0)
    cal.add_argument("--tolerance-hz", type=float, default=10.0)
    cal.add_argument("--stability-hz", type=float, default=5.0)
    cal.add_argument("--min-snr-db", type=float, default=12.0)
    cal.add_argument("--max-step-hz", type=int, default=750)
    cal.add_argument("--max-excursion-hz", type=int, default=5_000)
    cal.add_argument("--max-iterations", type=int, default=300)
    cal.add_argument("--search-hz", type=float, default=22_000.0)
    cal.add_argument("--probe-hz", type=int, default=250)
    cal.add_argument("--probe-tolerance-hz", type=float, default=25.0)
    cal.add_argument("--track-tolerance-hz", type=float, default=300.0)
    # The live Gqrx stream retained old IQ for ~0.3 s after a librpitx retune.
    # Wait longer and flush before measuring, otherwise the loop may lock onto
    # another stationary carrier in the band.
    cal.add_argument("--settle-s", type=float, default=0.5)
    cal.add_argument("--window-samples", type=int, default=16_384)
    cal.add_argument("--keep-carrier", action="store_true", help="sikeres mérés után a korrigált vivő maradjon bekapcsolva")
    cal.set_defaults(func=run_calibration)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.action == "calibrate":
        if args.target_hz < 20_000 or args.target_hz > 1_500_000_000:
            print("A célfrekvencia 20 kHz és 1,5 GHz közé essen.", file=sys.stderr)
            return 2
        if args.max_excursion_hz > 200_000:
            print("A kliens legfeljebb ±200 kHz-es TX-korrekciót enged.", file=sys.stderr)
            return 2
        if args.max_excursion_hz < 1 or args.max_step_hz < 1 or args.max_iterations < 1:
            print("A korrekciós határ, lépésköz és iterációszám legyen pozitív.", file=sys.stderr)
            return 2
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("Megszakítva.", file=sys.stderr)
        return 130
    except (OSError, ValueError, ProtocolError, TimeoutError) as exc:
        print(f"HIBA: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
