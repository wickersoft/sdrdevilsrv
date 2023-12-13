# sdrdevilsrv
Modification of hackrf_tcp to connect directly to sdrangel

## Dependencies / Compile
apt-get install hackrf
gcc -pthread -I /usr/local/include/libhackrf -o sdrdevilsrv main.c -lhackrf
./sdrdevilsrv

## Description
The SDR workbench application sdrangel needs no introduction. It comes in a locally executed form with GUI, but it can also be run in headless mode for streaming IQ over a LAN. The server application, sdrangelsrv, is stable and no doubt extremely capable, but I noticed it uses a lot of CPU when serving IQ to a client. The application must be doing some processing that is not necessary at all, possibly due to its complex modular architecture. sdrdevilsrv has a simple mission: It takes IQ samples from a HackRF One and shovels them into the LAN. On a Raspberry Pi 4, it can serve up to 17MS/s before not the CPU, but the ethernet controller becomes a bottleneck.

Only compatible with HackRF One for now! Working on implementations for RTL-SDR and nuand BladeRF 2.0.

Modified from the HackRF repository: https://github.com/avian2/hackrf/blob/master/host/hackrf-tools/src/hackrf_tcp.c
sdrangel's SDRA protocol: https://github.com/f4exb/sdrangel/blob/master/plugins/channelrx/remotetcpsink/remotetcpprotocol.h