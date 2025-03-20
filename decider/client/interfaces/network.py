import asyncio

# interfaces/network.py
#   @description:   network utilities

class Network:
    # class Network
    #
    #   @public variants
    #
    #   @public methods
    #
    #   @private variants
    #       _config     a dict of configuration
    #       _server     a tuple of 2, (ip, port)
    #       _reader
    #       _write

    def __init__(self, config):
        self._config = config
        self._server = ('127.0.0.1', 50140)
        self._reader = None
        self._writer = None

    async def initialize(self):
        self._reader, self._writer = await asyncio.open_connection(*self._server)

    async def receive(self):
        try:
            data = await asyncio.wait_for(self._reader.read(self._config["max_buffer_size"]),
                                          timeout=self._config["fake_aync_time_out"])
            return data
        except asyncio.TimeoutError:
            return "NO_MESSAGE"
        except ConnectionResetError:
            return "DISCONNECTED"
