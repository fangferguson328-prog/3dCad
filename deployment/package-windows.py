#!/usr/bin/env python3
"""Package current sources and the bundled x64 Release SDKs; never reuse old EXEs.

Preview: python3 deployment/package-windows.py --dry-run
Build:   python3 deployment/package-windows.py --output /path/3dCad-windows-source.zip
The archive preserves source bytes. Windows build converts only its staging copy.
"""
import argparse
import datetime
import fnmatch
import hashlib
import json
from pathlib import Path
import zipfile


def selected_files(root, manifest):
    selected = {}
    for pattern in manifest["include"]:
        matches = [p for p in root.glob(pattern) if p.is_file()]
        if not matches:
            raise ValueError("No files match required include pattern: " + pattern)
        for path in matches:
            relative = path.relative_to(root).as_posix()
            # Mesh OBJ fixtures are source assets, unlike MSVC object files.
            if relative.startswith("pcd/") or not any(
                fnmatch.fnmatch(relative, pattern) for pattern in manifest["exclude"]
            ):
                if path.is_symlink():
                    raise ValueError("Refusing symbolic link: " + relative)
                selected[relative] = path
    for required in manifest["required"]:
        if required not in selected:
            raise ValueError("Required file not selected: " + required)
    return sorted(selected.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    root = args.root.resolve()
    manifest = json.loads((root / "deployment/package-manifest.json").read_text(encoding="utf-8"))
    files = selected_files(root, manifest)
    total = sum(path.stat().st_size for _, path in files)
    print(f"Selected {len(files)} files, {total / (1024 ** 2):.1f} MiB uncompressed")
    if args.dry_run:
        return
    if args.output is None:
        parser.error("--output is required unless --dry-run is supplied")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError("Output already exists: " + str(output))
    output.parent.mkdir(parents=True, exist_ok=True)
    integrity = {
        "schemaVersion": 1,
        "target": manifest["target"],
        "createdAtUtc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "files": [],
    }
    temporary = output.with_suffix(output.suffix + ".partial")
    try:
        with zipfile.ZipFile(temporary, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for name, path in files:
                data = path.read_bytes()
                integrity["files"].append({"path": name, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()})
                archive.writestr(name, data)
            archive.writestr("deployment/package-integrity.json", json.dumps(integrity, ensure_ascii=False, indent=2))
        temporary.replace(output)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise
    print("Archive: " + str(output))
    with output.open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    print("SHA256: " + digest.hexdigest())


if __name__ == "__main__":
    main()
