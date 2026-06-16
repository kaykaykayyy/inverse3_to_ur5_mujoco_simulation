"""
import asyncio
import websockets
import orjson
import numpy as np

HAPLY_WS_URI = "ws://localhost:10001"
ZERO_FORCE = {"x": 0.0, "y": 0.0, "z": 0.0}

class HaplyInterface:
    def __init__(self):
        self.ws = None
        self.latest_message = None
        self.inverse3_id = None

    def extract_position(self, message):
        # ""
        This returns the position from the Haply arm itself - Inverse3
        Returns:
            np.array([x, y, z]) or None
        # ""
        devices = message.get("inverse3", [])
        if not devices:
            return None

        device_data = devices[0] if devices else {}
        pos = device_data["state"].get("cursor_position", {})

        if pos is None:
            return None

        return np.array([
            pos.get("x", 0.0),
            pos.get("y", 0.0),
            pos.get("z", 0.0),
        ])

    def extract_orientation(self, message): 
        # ""
        This returns the orientation from the Wireless Verse Grip where 
        -> wireless_verse_grip[0]["state"]["orientation"]
        Returns: xyz orientation 
        # ""
        devices = message.get("wireless_verse_grip", []) 
        if not devices: return None 

        verse_grip_data = devices[0] if devices else {}
        orient = verse_grip_data.get("state", {}).get("orientation", {})

        if orient is None: 
            return None 
        
        return np.array([
            orient.get("x", 0.0),
            orient.get("y", 0.0),
            orient.get("z", 0.0),
            orient.get("w", 1.0),
        ])

    def extract_buttons(self, message):
        devices = message.get("wireless_verse_grip", [])
        verse_grip_data = devices[0] if devices else {}
        buttons = verse_grip_data.get("state", {}).get("buttons", {})

        return buttons
    
    def get_latest_position(self):
        if not self.latest_message:
            return None
        return self.extract_position(self.latest_message)

    async def close(self):
        if self.ws:
            await self.ws.close()
            print("[INFO] Connection closed")

    async def connect(self):
        print("[INFO] Connecting to Haply Inverse Service...")
        self.ws = await websockets.connect(HAPLY_WS_URI)
        msg = orjson.loads(await self.ws.recv())
        print(msg)
        device = msg["inverse3"][0]
        self.inverse3_id = device["device_id"]
        self.latest_message = msg

        print(f"[HAPLY] Connected (device {self.inverse3_id})")
        print("[INFO] Connected to Haply Inverse Service")

    async def read_message(self, timeout: float = 0.2):
        #""
        Try to read a new WS frame, but don't block forever.
        On timeout, return the last cached message (may be None).
        
        if not self.ws:
            return self.latest_message

        try:
            msg = await asyncio.wait_for(self.ws.recv(), timeout)
            self.latest_message = orjson.loads(msg)
            return self.latest_message
        except asyncio.TimeoutError:
            # No new frame — return cached message so main loop can continue
            return self.latest_message
        except Exception as e:
            # Log and return cached message
            print(f"[HAPLY] read_message error: {e}")
            return self.latest_message
        #""
        msg = await self.ws.recv()
        return orjson.loads(msg)

    async def send_force(self, force):
        msg = {
            "inverse3": [{
                "commands": {
                    "set_cursor_force": {
                        "values": {
                            "x": float(force[0]),
                            "y": float(force[1]),
                            "z": float(force[2])
                        }
                    }
                }
            }]
        }
        await self.ws.send(orjson.dumps(msg))
    
    async def _stream_loop(self):
        #""
        Actively drives Haply streaming:
        SEND force -> RECEIVE state
        #""
        while self.running:
            # 1️⃣ Send zero force FIRST
            await self.ws.send(orjson.dumps({
                "inverse3": [{
                    "device_id": self.device_id,
                    "commands": {
                        "set_cursor_force": {
                            "values": ZERO_FORCE
                        }
                    }
                }]
            }))

            # 2️⃣ Receive updated state
            msg = await self.ws.recv()
            self.latest_message = orjson.loads(msg)

            # Small yield to avoid event-loop starvation
            await asyncio.sleep(0)

            """

import asyncio
import websockets
import orjson
import numpy as np

HAPLY_WS_URI = "ws://localhost:10001"
ZERO_FORCE = {"x": 0.0, "y": 0.0, "z": 0.0}

class HaplyInterface:
    def __init__(self):
        self.ws = None
        self.inverse3_id = None
        self._latest_message = None
        self._receive_task = None
        self._running = False
        self._lock = asyncio.Lock()

    def extract_position(self, message):
        """Returns position from the Inverse3 as np.array([x, y, z]) or None."""
        devices = message.get("inverse3", [])
        if not devices:
            return None
        pos = devices[0].get("state", {}).get("cursor_position")
        if pos is None:
            return None
        return np.array([pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)])

    def extract_orientation(self, message):
        """Returns orientation from Wireless Verse Grip as [x, y, z, w]."""
        devices = message.get("wireless_verse_grip", [])
        if not devices:
            return None
        orient = devices[0].get("state", {}).get("orientation", {})
        if orient is None:
            return None
        return np.array([orient.get("x", 0.0), orient.get("y", 0.0),
                         orient.get("z", 0.0), orient.get("w", 1.0)])

    def extract_buttons(self, message):
        devices = message.get("wireless_verse_grip", [])
        if not devices:
            return {}
        return devices[0].get("state", {}).get("buttons", {})

    def get_latest_position(self):
        """Thread‑safe access to the most recent position."""
        if self._latest_message is None:
            return None
        return self.extract_position(self._latest_message)

    def get_button_state(self, button_name="a"):
        """
        Returns True if the specified button on the Wireless Verse Grip is pressed.
        Button names: 'a', 'b', 'x', 'y', 'menu', 'home' (or use index).
        """
        if self._latest_message is None:
            return False
        buttons = self.extract_buttons(self._latest_message)
        # buttons is a dict like {"a": True, "b": False, ...}
        return buttons.get(button_name, False)
    
    async def connect(self):
        print("[INFO] Connecting to Haply Inverse Service...")
        self.ws = await websockets.connect(HAPLY_WS_URI)
        # First message is the initial device state
        msg = orjson.loads(await self.ws.recv())
        device = msg["inverse3"][0]
        self.inverse3_id = device["device_id"]
        self._latest_message = msg
        print(f"[HAPLY] Connected (device {self.inverse3_id})")
        self._running = True
        # Start background receiver loop
        self._receive_task = asyncio.create_task(self._receive_loop())

    async def _receive_loop(self):
        """Continuously receive state updates after each force command."""
        while self._running:
            try:
                msg = await self.ws.recv()
                parsed = orjson.loads(msg)
                async with self._lock:
                    self._latest_message = parsed
            except websockets.exceptions.ConnectionClosed:
                print("[ERROR] WebSocket closed")
                break
            except Exception as e:
                print(f"[ERROR] in receive loop: {e}")
                await asyncio.sleep(0.01)

    async def send_force(self, force):
        """Send a force command (this triggers the next state update)."""
        if not self.ws:
            return
        msg = {
            "inverse3": [{
                "device_id": self.inverse3_id,
                "commands": {
                    "set_cursor_force": {
                        "values": {
                            "x": float(force[0]),
                            "y": float(force[1]),
                            "z": float(force[2])
                        }
                    }
                }
            }]
        }
        await self.ws.send(orjson.dumps(msg))
        # The receive loop will pick up the response automatically

    async def close(self):
        self._running = False
        if self._receive_task:
            self._receive_task.cancel()
        if self.ws:
            await self.ws.close()
        print("[INFO] Connection closed")