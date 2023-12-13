# sdrdevilsrv
Modification of hackrf_tcp to connect directly to sdrangel. Modified from the [HackRF repository](https://github.com/avian2/hackrf/blob/master/host/hackrf-tools/src/hackrf_tcp.c). Implements sdrangel's [SDRA protocol](https://github.com/f4exb/sdrangel/blob/master/plugins/channelrx/remotetcpsink/remotetcpprotocol.h).

## Dependencies / Compile
```
apt-get install hackrf
gcc -pthread -I /usr/local/include/libhackrf -o sdrdevilsrv main.c -lhackrf
./sdrdevilsrv
```

## Description
The SDR workbench application sdrangel needs no introduction. It comes in a locally executed form with GUI, but it can also be run in headless mode for streaming IQ over a LAN. The server application, sdrangelsrv, is stable and no doubt extremely capable, but it uses a lot of CPU when serving IQ to a client. On a Raspberry Pi 4, this limits it to about 2.3MS/s. sdrdevilsrv eliminates all background processing and exclusively moves IQ samples from USB into the LAN. On the Pi 4, it can serve up to 17MS/s before the ethernet controller becomes a bottleneck at 285Mbps. Parameters such as center freq, sample rate, amplifier stages and baseband BW can be changed from the sdrangel client at runtime.

## Known issues
The signal degradation at high amplification has been resolved. Turns out sdrangel expects unsigned samples while hackrf_tcp sends signed samples. Somehow this produced _almost_ usable IQ.

**Only compatible with HackRF One for now! Working on implementations for RTL-SDR and nuand BladeRF 2.0.**
