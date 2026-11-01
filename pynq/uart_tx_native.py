"""Immutable native UART TX helper; importing this module accesses no hardware."""
import ctypes,hashlib
from pathlib import Path

class NativeUartTx:
    def __init__(self,mmio):
        self.mmio=mmio
        array=mmio.array
        if array.dtype.itemsize!=4 or array.nbytes<8:raise RuntimeError('Native UART TX requires a uint32 MMIO array')
        self.address=int(array.ctypes.data)
        if self.address&3:raise RuntimeError('UART MMIO address is not word aligned')
        source=Path(__file__).with_name('libuart_tx_v1.so');data=source.read_bytes();digest=hashlib.sha256(data).hexdigest()
        cache=source.parent/'.native_tx_cache';cache.mkdir(exist_ok=True);target=cache/('tx_'+digest+'.so')
        try:
            with target.open('xb') as f:f.write(data)
        except FileExistsError:
            if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:raise RuntimeError('Native TX cache checksum failure')
        self.lib=ctypes.CDLL(str(target))
        self.lib.uart_tx_abi.argtypes=[];self.lib.uart_tx_abi.restype=ctypes.c_uint32
        if self.lib.uart_tx_abi()!=0x54583132:raise RuntimeError('Native UART TX ABI mismatch')
        self.lib.uart_tx_write12.argtypes=[ctypes.c_size_t,ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint32]
        self.lib.uart_tx_write12.restype=ctypes.c_int
        self.buffer=(ctypes.c_uint8*12)()
    def write(self,packet):
        if len(packet)!=12:raise ValueError('Native TX requires exactly 12 bytes')
        self.buffer[:]=packet
        if self.lib.uart_tx_write12(self.address,self.buffer,12)!=0:raise RuntimeError('Native UART TX rejected input')
