# AD9361 IIO context profiles

Vendored libiio context XMLs captured from real hardware. Useful as:

- Reference fixtures for tests that need a realistic 4-channel layout
  without an SDR plugged in.
- Provenance reference when bringing up a new tuning preset (each
  channel's attribute set is hardware/firmware-specific — this is the
  ground truth for the device the file was captured on).
- Future input to a fully synthetic libiio mock context (libiio v0.26
  has `iio_create_xml_context()` for loading these directly, but DMA
  buffers cannot be exercised through it; capture-only).

The profiles are *raw* `iio_context_get_xml()` output — no
normalisation. URIs are intentionally left as `local:` (the value the
device reported) so the file is reproducible when re-captured.

## Files

### `ad9361_2r2t.xml`

- **Source**: FISH Ball PlutoSky (Z7010-AD9361) running tezuka_fw v0.3.5
- **Captured**: 2026-05-09 from `ip:10.0.23.149`
- **Kernel**: Linux 6.12.77 #1 SMP PREEMPT armv7l
- **libiio**: v0.26 (server side)
- **Mode**: 2R2T (full 4-channel topology — `voltage0..voltage3` on
  both `cf-ad9361-lpc` (RX) and `cf-ad9361-dds-core-lpc` (TX) DDS-core)

Capture procedure (no `iio_info` CLI required):

```sh
( printf 'PRINT\r\n'; sleep 1 ) | timeout 5 nc <host> 30431 > raw.bin
python3 -c '
import sys
data = open("raw.bin", "rb").read()
start = data.find(b"<?xml")
end = data.find(b"</context>", start) + len(b"</context>")
sys.stdout.buffer.write(data[start:end] + b"\n")
' > ad9361_2r2t.xml
```

The IIOD text protocol on port 30431 responds to `PRINT` with the
context XML directly (preceded by a length prefix). The Python step
strips the prefix and any trailing protocol echoes.
