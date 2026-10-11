import urllib.request
import hashlib
import os
import sys

# Approved ONNX Runtime v1.19.0 (Linux x64)
URL = "https://github.com/microsoft/onnxruntime/releases/download/v1.19.0/onnxruntime-linux-x64-1.19.0.tgz"
EXPECTED_SHA256 = "8627cae49b51544ed15bb70c8ecf25b187f22a301c99c1ced5f4f3af5c36b44c"
OUT_FILE = "onnxruntime.tgz"

def fetch_and_verify():
    print(f"Downloading approved ONNX Runtime from {URL}...")
    urllib.request.urlretrieve(URL, OUT_FILE)

    print("Verifying cryptographic hash...")
    sha256_hash = hashlib.sha256()
    with open(OUT_FILE, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)

    actual_hash = sha256_hash.hexdigest()
    if actual_hash != EXPECTED_SHA256:
        os.remove(OUT_FILE)
        print(f"CRITICAL: Hash mismatch!\nExpected: {EXPECTED_SHA256}\nGot:      {actual_hash}")
        sys.exit(1)

    print("Hash verified successfully. Supply chain secure.")
    # Extract the archive (Linux/CI environment)
    os.system(f"tar -xzf {OUT_FILE}")
    os.remove(OUT_FILE)

if __name__ == "__main__":
    fetch_and_verify()
