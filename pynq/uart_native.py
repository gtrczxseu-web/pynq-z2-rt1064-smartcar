"""Bounded UARTLite native receiver; no odometry or gyro integration here."""
import ctypes
import hashlib
from pathlib import Path

class NativeUartReceiver:
    def __init__(self, mmio):
        self.mmio = mmio  # keep the MMIO mapping alive until the native join
        array = mmio.array
        if array.dtype.itemsize != 4 or array.nbytes < 12:
            raise RuntimeError('UART MMIO must expose native 32-bit registers')
        source=Path(__file__).with_name('libuart_receiver_rt.so')
        data=source.read_bytes()
        digest=hashlib.sha256(data).hexdigest()
        # Loaded ELF mappings must never be overwritten by a later upload.
        cache=source.parent/'.native_cache'
        cache.mkdir(exist_ok=True)
        target=cache/('uart_'+digest+'.so')
        try:
            with target.open('xb') as stream:
                stream.write(data)
        except FileExistsError:
            if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Native cache integrity failure; restart with a fresh cache directory')
        lib = ctypes.CDLL(str(target))
        lib.uart_rx_start.argtypes = [ctypes.c_size_t]
        lib.uart_rx_start.restype = ctypes.c_void_p
        lib.uart_rx_read.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_uint16),ctypes.c_uint32]
        lib.uart_rx_read.restype = ctypes.c_uint32
        lib.uart_rx_dropped.argtypes = [ctypes.c_void_p]
        lib.uart_rx_dropped.restype = ctypes.c_uint64
        lib.uart_rx_stop.argtypes = [ctypes.c_void_p]
        lib.uart_rx_stop.restype = None
        lib.uart_rx_metrics.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_uint32)]
        lib.uart_rx_metrics.restype = None
        self.lib = lib
        self.buffer = (ctypes.c_uint16 * 4096)()
        self.handle = lib.uart_rx_start(int(array.ctypes.data))
        if not self.handle:
            raise RuntimeError('Cannot start native UART receiver')

    def read(self, limit=4096):
        if self.handle is None:
            raise RuntimeError('Native UART receiver is closed')
        count = self.lib.uart_rx_read(self.handle,self.buffer,min(limit,len(self.buffer)))
        dropped = int(self.lib.uart_rx_dropped(self.handle))
        return list(self.buffer[:count]), dropped

    def close(self):
        if self.handle is not None:
            self.lib.uart_rx_stop(self.handle)
            self.handle = None

    def metrics(self):
        if self.handle is None:
            raise RuntimeError('Native UART receiver is closed')
        values=(ctypes.c_uint32*6)()
        self.lib.uart_rx_metrics(self.handle,values)
        return dict(zip(('maximum_poll_gap_us','poll_gaps_over_1ms','affinity_error',
                         'realtime_error','scheduling_policy','priority'),map(int,values)))
