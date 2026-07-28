"""Desktop stand-in for the micro:bit `radio` module.

Acts as the map transmitter: whatever string is loaded with `load_map()` is
delivered in response to the firmware's 'Request' broadcast.
"""

_queue = []
_map_string = None
_on = False


def load_map(map_string):
    """Arm the simulated transmitter with the map it should reply with."""
    global _map_string
    _map_string = map_string


def config(**kwargs):
    pass


def on():
    global _on
    _on = True


def off():
    global _on
    _on = False
    _queue.clear()


def send(message):
    if message == "Request" and _map_string is not None:
        _queue.append(_map_string)


def receive():
    if _queue:
        return _queue.pop(0)
    return None
