"""
websocket_client.py - Async WebSocket client for StampFly telemetry

Connects to StampFly's WebSocket endpoint and streams telemetry data.
非同期WebSocketクライアント

Features:
- Async connection with websockets library
- Auto-reconnection on disconnect
- Callback-based packet delivery
- Statistics tracking
"""

import asyncio
import logging
from typing import Callable, List, Optional
from dataclasses import dataclass

try:
    import websockets
    from websockets.exceptions import ConnectionClosed, InvalidHandshake
except ImportError:
    raise ImportError(
        "websockets library required. Install with: pip install websockets"
    )

from .packet_parser import (
    ExtendedSample,
    parse_extended_batch_packet,
    EXTENDED_BATCH_PACKET_SIZE,
)


logger = logging.getLogger(__name__)


@dataclass
class ConnectionStats:
    """WebSocket connection statistics."""
    packets_received: int = 0
    samples_received: int = 0
    checksum_errors: int = 0
    reconnect_count: int = 0
    bytes_received: int = 0


class TelemetryWebSocketClient:
    """Async WebSocket client for StampFly telemetry.

    Usage:
        client = TelemetryWebSocketClient(
            host="192.168.4.1",
            port=80,
            on_samples=my_callback
        )
        await client.connect_and_run()

    Or for integration with ROS2 node:
        client = TelemetryWebSocketClient(...)
        # In ROS2 timer callback or executor thread:
        await client.receive_once()
    """

    DEFAULT_HOST = "192.168.4.1"
    DEFAULT_PORT = 80
    DEFAULT_PATH = "/ws"
    RECONNECT_DELAY = 2.0  # seconds

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        path: str = DEFAULT_PATH,
        on_samples: Optional[Callable[[List[ExtendedSample]], None]] = None,
        auto_reconnect: bool = True,
    ):
        """Initialize WebSocket client.

        Args:
            host: StampFly IP address
            port: WebSocket port
            path: WebSocket path
            on_samples: Callback for received samples
            auto_reconnect: Auto-reconnect on disconnect
        """
        self.host = host
        self.port = port
        self.path = path
        self.on_samples = on_samples
        self.auto_reconnect = auto_reconnect

        self._websocket: Optional[websockets.WebSocketClientProtocol] = None
        self._running = False
        self._connected = False

        self.stats = ConnectionStats()

    @property
    def uri(self) -> str:
        """WebSocket URI."""
        return f"ws://{self.host}:{self.port}{self.path}"

    @property
    def connected(self) -> bool:
        """Check if connected."""
        return self._connected and self._websocket is not None

    async def connect(self) -> bool:
        """Establish WebSocket connection.

        Returns:
            True if connected successfully
        """
        try:
            logger.info(f"Connecting to {self.uri}...")
            self._websocket = await websockets.connect(
                self.uri,
                ping_interval=None,  # Disable ping to reduce latency
                close_timeout=1.0,
            )
            self._connected = True
            logger.info(f"Connected to {self.uri}")
            return True
        except (OSError, InvalidHandshake) as e:
            logger.error(f"Connection failed: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Close WebSocket connection."""
        self._running = False
        if self._websocket:
            try:
                await self._websocket.close()
            except Exception:
                pass
            self._websocket = None
        self._connected = False
        logger.info("Disconnected")

    async def receive_once(self) -> Optional[List[ExtendedSample]]:
        """Receive and parse a single packet.

        Non-blocking if no data available (will raise TimeoutError).
        Use with asyncio.wait_for() to add timeout.

        Returns:
            List of ExtendedSample, or None if not connected/parse error
        """
        if not self._websocket:
            return None

        try:
            data = await self._websocket.recv()
        except ConnectionClosed:
            logger.warning("Connection closed by server")
            self._connected = False
            return None
        except Exception as e:
            logger.error(f"Receive error: {e}")
            return None

        if not isinstance(data, bytes):
            return None

        self.stats.bytes_received += len(data)

        # Parse packet
        if len(data) == EXTENDED_BATCH_PACKET_SIZE:
            samples = parse_extended_batch_packet(data)
            if samples:
                self.stats.packets_received += 1
                self.stats.samples_received += len(samples)
                return samples
            else:
                self.stats.checksum_errors += 1
                return None
        else:
            logger.debug(f"Unexpected packet size: {len(data)}")
            return None

    async def connect_and_run(self) -> None:
        """Connect and run receive loop until stopped.

        Calls on_samples callback for each batch of samples.
        Auto-reconnects if enabled.
        """
        self._running = True

        while self._running:
            # Connect if not connected
            if not self._connected:
                if not await self.connect():
                    if self.auto_reconnect and self._running:
                        logger.info(f"Reconnecting in {self.RECONNECT_DELAY}s...")
                        self.stats.reconnect_count += 1
                        await asyncio.sleep(self.RECONNECT_DELAY)
                        continue
                    else:
                        break

            # Receive loop
            try:
                samples = await self.receive_once()
                if samples and self.on_samples:
                    self.on_samples(samples)
            except Exception as e:
                logger.error(f"Error in receive loop: {e}")
                self._connected = False
                if not self.auto_reconnect:
                    break

    def stop(self) -> None:
        """Signal to stop the receive loop."""
        self._running = False


class ThreadedWebSocketClient:
    """Thread-safe WebSocket client wrapper for ROS2 integration.

    Runs the async WebSocket client in a background thread with its own
    event loop. Provides a thread-safe queue for samples.

    Usage with ROS2:
        client = ThreadedWebSocketClient(host="192.168.4.1")
        client.start()

        # In ROS2 timer callback:
        while True:
            sample = client.get_sample()
            if sample is None:
                break
            publish(sample)

        # On shutdown:
        client.stop()
    """

    def __init__(
        self,
        host: str = TelemetryWebSocketClient.DEFAULT_HOST,
        port: int = TelemetryWebSocketClient.DEFAULT_PORT,
        path: str = TelemetryWebSocketClient.DEFAULT_PATH,
        queue_size: int = 100,
        auto_reconnect: bool = True,
    ):
        """Initialize threaded client.

        Args:
            host: StampFly IP address
            port: WebSocket port
            path: WebSocket path
            queue_size: Max samples in queue before dropping
            auto_reconnect: Auto-reconnect on disconnect
        """
        import threading
        import queue

        self.host = host
        self.port = port
        self.path = path
        self.auto_reconnect = auto_reconnect

        self._queue: queue.Queue[ExtendedSample] = queue.Queue(maxsize=queue_size)
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._client: Optional[TelemetryWebSocketClient] = None
        self._stop_event = threading.Event()

    def _on_samples(self, samples: List[ExtendedSample]) -> None:
        """Callback to enqueue samples."""
        for sample in samples:
            try:
                self._queue.put_nowait(sample)
            except Exception:
                # Queue full, drop oldest and add new
                try:
                    self._queue.get_nowait()
                    self._queue.put_nowait(sample)
                except Exception:
                    pass

    def _run_loop(self) -> None:
        """Run asyncio event loop in background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        self._client = TelemetryWebSocketClient(
            host=self.host,
            port=self.port,
            path=self.path,
            on_samples=self._on_samples,
            auto_reconnect=self.auto_reconnect,
        )

        try:
            self._loop.run_until_complete(self._client.connect_and_run())
        except Exception as e:
            logger.error(f"Event loop error: {e}")
        finally:
            self._loop.close()

    def start(self) -> None:
        """Start background WebSocket thread."""
        import threading

        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("WebSocket thread started")

    def stop(self) -> None:
        """Stop background WebSocket thread."""
        self._stop_event.set()
        if self._client:
            self._client.stop()
        if self._loop:
            # Schedule disconnect in the event loop
            asyncio.run_coroutine_threadsafe(
                self._client.disconnect() if self._client else asyncio.sleep(0),
                self._loop
            )
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("WebSocket thread stopped")

    def get_sample(self, timeout: float = 0.0) -> Optional[ExtendedSample]:
        """Get a sample from the queue.

        Args:
            timeout: Max wait time in seconds (0 = non-blocking)

        Returns:
            ExtendedSample or None if queue empty/timeout
        """
        import queue as queue_module

        try:
            if timeout <= 0:
                return self._queue.get_nowait()
            else:
                return self._queue.get(timeout=timeout)
        except queue_module.Empty:
            return None

    def get_all_samples(self) -> List[ExtendedSample]:
        """Get all available samples from queue (non-blocking).

        Returns:
            List of ExtendedSample (may be empty)
        """
        samples = []
        while True:
            sample = self.get_sample()
            if sample is None:
                break
            samples.append(sample)
        return samples

    @property
    def queue_size(self) -> int:
        """Current number of samples in queue."""
        return self._queue.qsize()

    @property
    def connected(self) -> bool:
        """Check if WebSocket is connected."""
        return self._client.connected if self._client else False

    @property
    def stats(self) -> ConnectionStats:
        """Get connection statistics."""
        return self._client.stats if self._client else ConnectionStats()
