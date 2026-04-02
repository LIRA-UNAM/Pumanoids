#!/usr/bin/env python3
import queue
import time
import wave

import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String


#
# Parameters for recording audio
#
CHUNK = 1024
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 10
DEVICE = 25
WAVE_OUTPUT_FILENAME = "/dev/shm/recorder_audio.wav"


def remove_dc(x):
    return x - np.mean(x)


def highpass_filter(x, fs, cutoff=90.0):
    if len(x) == 0:
        return x
    rc = 1.0 / (2.0 * np.pi * cutoff)
    dt = 1.0 / fs
    alpha = rc / (rc + dt)

    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * (y[i - 1] + x[i] - x[i - 1])
    return y


def trim_silence(x, fs, frame_ms=30, hop_ms=10, threshold_db=-38.0, pad_ms=180):
    if len(x) == 0:
        return x

    frame = int(fs * frame_ms / 1000)
    hop = int(fs * hop_ms / 1000)
    pad = int(fs * pad_ms / 1000)

    if len(x) < frame:
        return x

    dbs = []
    starts = []

    for start in range(0, len(x) - frame + 1, hop):
        chunk = x[start:start + frame]
        rms = np.sqrt(np.mean(chunk**2) + 1e-10)
        db = 20.0 * np.log10(rms + 1e-10)
        dbs.append(db)
        starts.append(start)

    dbs = np.array(dbs)
    starts = np.array(starts)

    speech_idx = np.where(dbs > threshold_db)[0]
    if len(speech_idx) == 0:
        return x

    start_sample = max(starts[speech_idx[0]] - pad, 0)
    end_sample = min(starts[speech_idx[-1]] + frame + pad, len(x))
    return x[start_sample:end_sample]


def normalize_rms_peak(x, target_rms=0.10, target_peak=0.85, max_gain=4.0):
    if len(x) == 0:
        return x, 1.0, 0.0, 0.0

    rms = np.sqrt(np.mean(x**2) + 1e-10)
    peak = np.max(np.abs(x)) + 1e-10

    gain_rms = target_rms / rms if rms > 1e-8 else 1.0
    gain_peak = target_peak / peak
    gain = min(gain_rms, gain_peak, max_gain)

    y = np.clip(x * gain, -1.0, 1.0)

    rms_after = np.sqrt(np.mean(y**2) + 1e-10)
    peak_after = np.max(np.abs(y))

    return y, gain, rms_after, peak_after


class FasterWhisperNode(Node):
    def __init__(self):
        super().__init__("faster_whisper_node")
        self.get_logger().info("INITIALIZING FASTER WHISPER NODE")

        self.model_size = "small"
        self.pwr_threshold = 0.01
        self.pub_recognized = self.create_publisher(String, "/sp_rec/recognized", 1)

        self.device = DEVICE
        self.rate = RATE
        self.channels = CHANNELS
        self.chunk = CHUNK
        self.wave_output_filename = WAVE_OUTPUT_FILENAME

        self.audio_queue = queue.Queue()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")
        self.audio_queue.put(indata[:, 0].copy())

    def save_wav(self, audio_f32):
        audio_int16 = (audio_f32 * 32767).astype(np.int16)

        with wave.open(self.wave_output_filename, "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(self.rate)
            wf.writeframes(audio_int16.tobytes())

    def spin(self):
        self.get_logger().info("Creating Faster Whisper model with size " + self.model_size)
        model = WhisperModel(self.model_size, device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model created")
        self.get_logger().info("Creating sounddevice stream...")

        blocksize = self.chunk

        with sd.InputStream(
            samplerate=self.rate,
            channels=self.channels,
            dtype="float32",
            device=self.device,
            blocksize=blocksize,
            callback=self.audio_callback,
        ):
            while rclpy.ok():
                frames = []

                self.get_logger().info("Waiting for audio with enough power")
                pwr = 0.0
                data = None

                while pwr < self.pwr_threshold and rclpy.ok():
                    data = self.audio_queue.get()
                    pwr = float(np.mean(data**2))

                self.get_logger().info("Audio detected. Starting to record...")
                no_audio_counter = 0

                if data is not None:
                    frames.append(data)

                while no_audio_counter < 20 and rclpy.ok():
                    data = self.audio_queue.get()
                    frames.append(data)

                    pwr = float(np.mean(data**2))
                    if pwr < self.pwr_threshold:
                        no_audio_counter += 1
                    else:
                        no_audio_counter = 0

                self.get_logger().info("Stopping audio recording.")

                audio = np.concatenate(frames, axis=0).astype(np.float32)

                peak_original = np.max(np.abs(audio)) if len(audio) > 0 else 0.0
                rms_original = np.sqrt(np.mean(audio**2) + 1e-10) if len(audio) > 0 else 0.0
                self.get_logger().info(f"peak original: {peak_original}")
                self.get_logger().info(f"rms original: {rms_original}")

                audio = remove_dc(audio)

                audio = highpass_filter(audio, self.rate, cutoff=90.0)

                audio = trim_silence(audio, self.rate, threshold_db=-38.0, pad_ms=180)

                audio, gain, rms_after, peak_after = normalize_rms_peak(
                    audio,
                    target_rms=0.10,
                    target_peak=0.85,
                    max_gain=4.0
                )

                self.get_logger().info(f"gain aplicada: {gain}")
                self.get_logger().info(f"rms final: {rms_after}")
                self.get_logger().info(f"peak final: {peak_after}")
                self.get_logger().info(f"duracion final (s): {len(audio) / self.rate if len(audio) > 0 else 0.0}")

                if len(audio) == 0:
                    self.get_logger().warn("Audio vacio despues del preprocesamiento. Saltando.")
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                self.save_wav(audio)

                self.get_logger().info("Transcribing audio...")
                segments, info = model.transcribe(
                    self.wave_output_filename,
                    language="es",
                    beam_size=5,
                    best_of=5,
                    vad_filter=True,
                    vad_parameters=dict(
                        threshold=0.60,
                        min_speech_duration_ms=200,
                        min_silence_duration_ms=500,
                        speech_pad_ms=250,
                    ),
                    condition_on_previous_text=False,
                    temperature=0.0,
                    no_speech_threshold=0.3,
                )

                self.get_logger().info(
                    "Detected language '%s' with probability %f"
                    % (info.language, info.language_probability)
                )

                full_text = " ".join(seg.text.strip() for seg in segments).strip()

                if full_text:
                    self.get_logger().info(f"Recognized: {full_text}")
                    self.pub_recognized.publish(String(data=full_text))
                else:
                    self.get_logger().info("No text recognized.")

                rclpy.spin_once(self, timeout_sec=0.0)
                self.get_clock().sleep_for(Duration(seconds=0.005))


def main(args=None):
    rclpy.init(args=args)
    faster_whisper_node = FasterWhisperNode()
    faster_whisper_node.spin()
    faster_whisper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
