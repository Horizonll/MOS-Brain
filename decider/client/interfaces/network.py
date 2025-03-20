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
        return
        self._config = config
        self._server = (127.0.0.1, 50140)
        reader, write = await asyncio.open_connection(self._server)
        self._reader, self._writer = reader, writer

    def receive():
        return "NO_MESSAGE"
        try:
            data = await asyncio.wait_for(reader.read(config["max_buffer_size"]), 
                                            time_out = config["fake_aync_time_out"])
            return data
        except asyncio.TimeoutError:
            return "NO_MESSAGE"
        except ConnectionResetError:
            return "DISCONNECTED"
            
