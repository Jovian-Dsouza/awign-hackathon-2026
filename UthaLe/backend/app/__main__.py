"""`python -m app` runs the API on 127.0.0.1:8000.

timeout_graceful_shutdown matters: open MJPEG streams never finish on their own,
so without it Ctrl+C would wait forever and the camera would not be released.
"""

import os

import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",
        host=os.environ.get("HOST", "127.0.0.1"),
        port=int(os.environ.get("PORT", "8000")),
        timeout_graceful_shutdown=2,
        reload=os.environ.get("RELOAD", "0") == "1",
    )
