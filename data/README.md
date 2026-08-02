# data/

Generated from `web/` — **never hand-edit anything under here.**

```bash
python scripts/sync_web.py            # web/ -> data/web/
pio run -e esp32s3 -t uploadfs        # flash it to the device's LittleFS partition
```

If you edited a file under `data/web/` directly, your changes will be
silently overwritten the next time someone runs `sync_web.py`. Edit the
matching file under `web/` instead.
