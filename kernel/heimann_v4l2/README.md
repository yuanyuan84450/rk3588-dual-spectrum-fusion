# Heimann HTPA V4L2 driver

This directory is an independently buildable, rollback-safe V4L2 version of
the existing Heimann I2C misc driver. It does not replace the old source or
module.

Build on the RK3588S board:

```bash
make clean
make
```

Outputs:

- `heimann_v4l2_drv.ko`: V4L2/videobuf2 kernel module.
- `test_heimann_v4l2`: MMAP/QBUF/DQBUF streaming smoke test.
- `99-heimann-v4l2.rules`: grants the `video` group access to the EEPROM/
  legacy misc node.

The capture format is the private `HTPA` FourCC, fixed at 32×32 with a
2580-byte payload. `/dev/heimann0` remains the EEPROM/legacy control node;
the V4L2 capture node is allocated dynamically as `/dev/videoX`.

Current checkpoint: compilation against Linux 5.10.160, user test compilation,
DTS compilation, and full application compilation pass. Hardware streaming is
not yet tested because the active device tree currently has no `heimann,htpa`
node. Do not load the old and new modules at the same time because both bind to
the same compatible string.

See `docs/phase3_heimann_v4l2_driver_and_interview.md` in `project_clean` for
the deployment checklist, remaining tests, design explanation and interview
questions.
