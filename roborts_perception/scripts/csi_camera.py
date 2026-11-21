"""In-process CSI camera capture through a GStreamer ``appsink``.

The Python ``cv2`` on this image is built **without** GStreamer support, and
``/dev/video0`` only exposes raw RG10 Bayer data, so the camera must be read
through Argus.  This module drives the same pipeline used by the design doc
(section 4.2.1) with the GObject bindings:

    nvarguscamerasrc ! video/x-raw(memory:NVMM),... ! nvvidconv !
    video/x-raw,format=BGRx ! appsink max-buffers=1 drop=true

The appsink callback only converts BGRx -> BGR and swaps the "latest frame"
reference.  Consumers call :meth:`CsiCamera.read_latest` at whatever rate they
can sustain; because the sink keeps at most one buffer and drops the oldest on
overflow, a slow consumer skips frames instead of accumulating latency.

No ROS types are used here, which keeps the camera usable from the standalone
calibration tool as well.
"""

import threading
import time

import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402  (after require_version)

_GST_INIT_LOCK = threading.Lock()
_GST_INITIALIZED = False


def _ensure_gst():
    """Call ``Gst.init`` exactly once per process."""
    global _GST_INITIALIZED
    with _GST_INIT_LOCK:
        if not _GST_INITIALIZED:
            Gst.init(None)
            _GST_INITIALIZED = True


class Frame:
    """A captured BGR frame.

    Attributes:
        image: ``numpy.ndarray`` of shape ``(height, width, 3)``, dtype uint8,
            BGR, already copied out of the GStreamer buffer.
        capture_time_ns: wall-clock timestamp (``time.time_ns``) taken when
            the sample was pulled from the sink.
        sequence: monotonically increasing frame counter.
    """

    __slots__ = ('image', 'capture_time_ns', 'sequence')

    def __init__(self, image, capture_time_ns, sequence):
        self.image = image
        self.capture_time_ns = capture_time_ns
        self.sequence = sequence


class CsiCamera:
    """Argus CSI camera with a single-slot latest-frame buffer."""

    def __init__(self, sensor_id=0, width=1920, height=1080, framerate=30):
        _ensure_gst()
        self._sensor_id = int(sensor_id)
        self._width = int(width)
        self._height = int(height)
        self._framerate = int(framerate)

        self._pipeline = None
        self._bus_thread = None
        self._stop_event = threading.Event()

        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._latest_frame = None
        self._error_message = None
        self._frame_size = None
        self._sequence = 0

        self._fps = 0.0
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0

    # ------------------------------------------------------------------ props
    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def framerate(self):
        return self._framerate

    @property
    def measured_fps(self):
        """Frames actually received per second, averaged over ~1 s."""
        return self._fps

    @property
    def frame_count(self):
        """Number of frames delivered by the sink since :meth:`start`."""
        return self._sequence

    @property
    def error_message(self):
        with self._lock:
            return self._error_message

    @property
    def is_running(self):
        return self._pipeline is not None and not self._stop_event.is_set()

    # ---------------------------------------------------------------- lifecycle
    def _pipeline_description(self):
        return (
            f'nvarguscamerasrc sensor-id={self._sensor_id} '
            f'! video/x-raw(memory:NVMM),width={self._width},'
            f'height={self._height},framerate={self._framerate}/1 '
            '! nvvidconv '
            '! video/x-raw,format=BGRx '
            '! appsink name=sink max-buffers=1 drop=true sync=false '
            'emit-signals=true'
        )

    def start(self):
        """Build and start the pipeline.

        Raises:
            RuntimeError: if GStreamer cannot build or start the pipeline.
        """
        if self._pipeline is not None:
            return

        description = self._pipeline_description()
        try:
            pipeline = Gst.parse_launch(description)
        except Exception as exc:  # GLib.Error
            raise RuntimeError(
                f'failed to build CSI pipeline: {exc}; pipeline was: '
                f'{description}') from exc

        sink = pipeline.get_by_name('sink')
        if sink is None:
            raise RuntimeError('CSI pipeline has no "sink" element')
        sink.connect('new-sample', self._on_new_sample)

        self._stop_event.clear()
        self._frame_ready.clear()
        self._latest_frame = None
        self._error_message = None
        self._frame_size = None
        self._sequence = 0
        self._pipeline = pipeline

        result = pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            raise RuntimeError(
                'failed to start CSI pipeline (check that the sensor is not in '
                'use by another process)')

        self._bus_thread = threading.Thread(
            target=self._bus_loop, name='csi-bus', daemon=True)
        self._bus_thread.start()

    def stop(self):
        """Stop the pipeline and join the bus thread (idempotent)."""
        self._stop_event.set()
        pipeline = self._pipeline
        self._pipeline = None
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)
        if self._bus_thread is not None:
            self._bus_thread.join(timeout=2.0)
            self._bus_thread = None
        self._frame_ready.set()

    # -------------------------------------------------------------- bus thread
    def _bus_loop(self):
        pipeline = self._pipeline
        if pipeline is None:
            return
        bus = pipeline.get_bus()
        while not self._stop_event.is_set():
            message = bus.timed_pop_filtered(
                100 * Gst.MSECOND,
                Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                text = f'GStreamer error: {error.message}'
                if debug:
                    text += f' ({debug})'
                self._set_error(text)
            else:
                self._set_error('CSI pipeline reached EOS')
            break

    def _set_error(self, text):
        with self._lock:
            if self._error_message is None:
                self._error_message = text
        # Wake any consumer blocked in read_latest so it can observe the error.
        self._frame_ready.set()

    # ---------------------------------------------------------- appsink thread
    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()
        if buffer is None:
            return Gst.FlowReturn.OK

        ok, map_info = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        try:
            if self._frame_size is None:
                caps = sample.get_caps()
                structure = caps.get_structure(0)
                self._frame_size = (
                    int(structure.get_value('width')),
                    int(structure.get_value('height')))

            width, height = self._frame_size
            expected = width * height * 4
            if map_info.size != expected:
                self._set_error(
                    f'unexpected BGRx buffer size {map_info.size} '
                    f'(expected {expected} for {width}x{height})')
                return Gst.FlowReturn.OK

            raw = np.ndarray(
                (height, width, 4), dtype=np.uint8, buffer=map_info.data)
            # cvtColor allocates a new array, so the frame stays valid after
            # the buffer is unmapped.
            bgr = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
        finally:
            buffer.unmap(map_info)

        frame = Frame(bgr, time.time_ns(), self._sequence)
        self._sequence += 1
        with self._lock:
            self._latest_frame = frame
        self._frame_ready.set()
        self._update_fps()
        return Gst.FlowReturn.OK

    def _update_fps(self):
        now = time.monotonic()
        self._fps_window_count += 1
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._fps = self._fps_window_count / elapsed
            self._fps_window_count = 0
            self._fps_window_start = now

    # -------------------------------------------------------------- consumers
    def read_latest(self, timeout=None):
        """Return the newest frame captured since the previous call.

        Blocks up to ``timeout`` seconds (``None`` = forever).  Returns
        ``None`` on timeout, or when the pipeline failed and no frame arrived.
        """
        if not self._frame_ready.wait(timeout):
            return None
        with self._lock:
            frame = self._latest_frame
            error = self._error_message
            # Hand out the frame only once; a later call waits for a new one.
            self._latest_frame = None
            self._frame_ready.clear()
        if frame is not None:
            return frame
        # No frame: this was an error wake-up.  Keep the event set so repeated
        # calls return immediately instead of blocking.
        if error is not None:
            self._frame_ready.set()
        return None

    def peek_latest(self):
        """Return the newest frame without consuming it (may be ``None``)."""
        with self._lock:
            return self._latest_frame

    # ----------------------------------------------------------------- context
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False
