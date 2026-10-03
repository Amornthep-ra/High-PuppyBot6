"""Build the separate board release from an installed Arduino-Pico 6.0.0 core.

Usage: python build_package.py PATH_TO_ARDUINO_PICO_6.0.0
The upstream installation is read only. Output is written to dist/.
"""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("core", type=Path)
    args = parser.parse_args()
    core = args.core.resolve()
    root = Path(__file__).resolve().parent
    index_file = root / "package_highpuppybot6_index.json"
    index = json.loads(index_file.read_text(encoding="utf-8"))
    platform = index["packages"][0]["platforms"][0]
    if not (core / "cores" / "rp2040").is_dir():
        parser.error("core must be the installed Arduino-Pico 6.0.0 directory")
    upstream = (core / "platform.txt").read_text(encoding="utf-8")
    if "version=6.0.0" not in upstream.splitlines():
        parser.error("this release requires Arduino-Pico 6.0.0")
    output = root / "dist" / platform["archiveFileName"]
    output.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for source in sorted(core.rglob("*")):
            if not source.is_file():
                continue
            rel = source.relative_to(core)
            if rel.parts[0] in {"docs", "tests", ".github", "variants"}:
                if rel.parts[:2] != ("variants", "generic"):
                    continue
            if rel.as_posix() in {"installed.json", "boards.txt", "platform.txt"}:
                continue
            archive.write(source, "highpuppybot6/" + rel.as_posix())
        updated = "\n".join(
            "name=High-PuppyBot6" if line.startswith("name=") else
            "version=" + platform["version"] if line.startswith("version=") else line
            for line in upstream.splitlines()
        ) + "\n"
        archive.writestr("highpuppybot6/platform.txt", updated)
        for folder, prefix in [(root / "platform", ""), (root / "libraries", "libraries/")]:
            for source in sorted(folder.rglob("*")):
                if source.is_file():
                    archive.write(source, "highpuppybot6/" + prefix + source.relative_to(folder).as_posix())
    platform["size"] = str(output.stat().st_size)
    with output.open("rb") as stream:
        platform["checksum"] = "SHA-256:" + hashlib.file_digest(stream, "sha256").hexdigest()
    index_file.write_text(json.dumps(index, indent=2) + "\n", encoding="utf-8")
    print(f"Built {output.name}: {output.stat().st_size / 1024**2:.1f} MB")


if __name__ == "__main__":
    main()
