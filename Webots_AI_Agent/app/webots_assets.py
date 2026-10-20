"""Verified Webots asset references (EXTERNPROTO lines).

Why this exists
---------------
Generated worlds must reference Webots' built-in PROTO files in a form Webots can resolve:

  * ``webots://projects/<path>``  - the form Webots' own R2025a PROTO files use internally
                                    (checked against the R2025a source tree). Resolved against the installation.
  * ``https://raw.githubusercontent.com/cyberbotics/webots/<VERSION>/projects/<path>``  - the pinned online form.
  * absolute Windows paths (``G:\\...``)  - NOT used: backslashes / drive letters are not valid PROTO URLs.

The resolver (1) detects the installed Webots version, (2) checks that every PROTO file we are going to
reference really exists under ``<WEBOTS_HOME>/projects`` and (3) refuses to hand out a reference it could not
verify. It only READS the installation; nothing in the Webots folder is ever modified.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

# key -> (PROTO node name, path relative to <webots>/projects)
CATALOG: dict[str, tuple[str, str]] = {
    "TexturedBackground": ("TexturedBackground", "objects/backgrounds/protos/TexturedBackground.proto"),
    "TexturedBackgroundLight": ("TexturedBackgroundLight", "objects/backgrounds/protos/TexturedBackgroundLight.proto"),
    "RectangleArena": ("RectangleArena", "objects/floors/protos/RectangleArena.proto"),
    "WoodenBox": ("WoodenBox", "objects/factory/containers/protos/WoodenBox.proto"),
    "CardboardBox": ("CardboardBox", "objects/factory/containers/protos/CardboardBox.proto"),
    "E-puck": ("E-puck", "robots/gctronic/e-puck/protos/E-puck.proto"),
    "Mavic2Pro": ("Mavic2Pro", "robots/dji/mavic/protos/Mavic2Pro.proto"),
}
GITHUB_RAW = "https://raw.githubusercontent.com/cyberbotics/webots/{version}/projects/{rel}"
VERSION_RE = re.compile(r"R\d{4}[a-z]")


class AssetError(Exception):
    """Raised when a required Webots asset cannot be verified."""


@dataclass(frozen=True)
class Asset:
    key: str
    node: str
    rel: str
    url: str
    local_path: Path | None
    verified: bool                 # file exists in the local installation
    online: bool = False           # reference is the pinned GitHub URL (needs internet on first load; Webots caches it)

    @property
    def externproto_line(self) -> str:
        return f'EXTERNPROTO "{self.url}"'


class AssetResolver:
    def __init__(self, webots_home: str | Path | None, version_hint: str = "R2025a", url_mode: str = "webots"):
        self.home = Path(webots_home) if webots_home else None
        self.version_hint = version_hint
        self.url_mode = url_mode if url_mode in ("webots", "github", "local") else "webots"

    # ------------------------------------------------------------ installation facts
    @property
    def projects_dir(self) -> Path | None:
        return self.home / "projects" if self.home else None

    def detect_version(self) -> tuple[str, str]:
        """Return (version, how). Reads <home>/resources/version.txt when present; else the .env hint."""
        if self.home:
            for rel in ("resources/version.txt", "resources/webots_version.txt"):
                f = self.home / rel
                if f.is_file():
                    m = VERSION_RE.search(f.read_text(encoding="utf-8", errors="ignore"))
                    if m:
                        return m.group(0), f"detected from {f}"
        return self.version_hint, "WEBOTS_VERSION from .env (installation version file not found)"

    @property
    def version(self) -> str:
        return self.detect_version()[0]

    def installation_problems(self) -> list[str]:
        out: list[str] = []
        if self.home is None:
            out.append("WEBOTS_HOME is not set in .env, so PROTO files cannot be verified.")
        elif not self.home.is_dir():
            out.append(f"WEBOTS_HOME does not exist: {self.home}")
        elif not (self.home / "projects").is_dir():
            out.append(f"No 'projects' folder inside WEBOTS_HOME ({self.home}). Is this the Webots root folder?")
        v, how = self.detect_version()
        if how.startswith("WEBOTS_VERSION") and self.home and self.home.is_dir():
            out.append(f"Could not read the installed Webots version ({how}); using {v}.")
        return out

    # ------------------------------------------------------------ resolution
    def resolve(self, key: str) -> Asset:
        node, rel = CATALOG[key]
        local = (self.projects_dir / rel) if self.projects_dir else None
        verified = bool(local and local.is_file())
        online = False
        if self.url_mode == "github" or (self.url_mode == "webots" and not verified):
            # Not shipped locally (e.g. this Windows install): use the official pinned URL, which Webots downloads.
            url = GITHUB_RAW.format(version=self.version, rel=rel)
            online = True
        elif self.url_mode == "local" and local is not None:
            url = local.as_posix()            # forward slashes, e.g. G:/InstallSoftWare/Webots/projects/...
        else:
            url = f"webots://projects/{rel}"
        return Asset(key, node, rel, url, local, verified, online)

    @staticmethod
    def online_exists(url: str) -> bool:
        try:
            import httpx
            return httpx.head(url, timeout=10, follow_redirects=True).status_code == 200
        except Exception:
            return False

    def require(self, keys: list[str]) -> dict[str, Asset]:
        """Resolve all keys; raise AssetError for every asset found neither locally nor at its pinned GitHub URL."""
        assets = {k: self.resolve(k) for k in keys}
        missing = [a for a in assets.values() if not a.verified and not (a.online and self.online_exists(a.url))]
        if missing:
            where = self.projects_dir or "<WEBOTS_HOME not set>"
            lines = "\n".join(f"  - {a.node}: not in {a.local_path or a.rel} and not reachable at {a.url}" for a in missing)
            raise AssetError(
                f"{len(missing)} Webots asset(s) not found under {where}:\n{lines}\n"
                "Check WEBOTS_HOME / WEBOTS_VERSION in .env and your internet connection. Nothing was generated, so Webots will not skip any nodes.")
        return assets
