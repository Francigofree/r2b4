"""Sign in with ChatGPT OAuth for the host-side R2B4 LLM path.

This module owns only authentication material.  It has no RobotInterface,
runtime, motor, GPIO or safety authority.

The implementation follows OpenAI's open-source Sign in with ChatGPT flow:
- dynamic public-client registration with PKCE and loopback callback
- ID-token verification against the OpenAI JWKS
- owner-only atomic credential storage
- lazy access-token renewal with rotating refresh-token serialization
- optional protected credential import for a headless/self-hosted R2B4 host
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import os
import secrets
import shutil
import stat
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import webbrowser
from collections.abc import Mapping
from contextlib import contextmanager
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Iterator

try:  # Linux/Pi runtime; optional so the desktop transfer helper also works on Windows.
    import fcntl  # type: ignore
except ImportError:  # pragma: no cover - Windows helper path
    fcntl = None  # type: ignore


ISSUER = "https://auth.openai.com"
AUTH_ENDPOINT = "https://auth.openai.com/api/accounts/authorize"
TOKEN_ENDPOINT = "https://auth.openai.com/api/accounts/oauth/token"
JWKS_URI = "https://auth.openai.com/.well-known/jwks.json"
DISCOVERY_URI = "https://auth.openai.com/.well-known/openid-configuration"
RESOURCE = "https://api.openai.com/v1"
DYNAMIC_CLIENT_ID = "dynamic_agent_client"
AGENT_NAME = "R2B4"
DIRECT_SCOPE = "chatgpt.tokens.use.direct"
REQUESTED_SCOPES = (
    "openid",
    "profile",
    "email",
    "offline_access",
    "resource.invoke",
    DIRECT_SCOPE,
)
CALLBACK_PATH = "/auth/callback"
CREDENTIAL_REL = Path("conf") / ".chatgpt_oauth.json"
HOST_ID_REL = Path("conf") / ".chatgpt_host_id"
LOCK_REL = Path("conf") / ".chatgpt_oauth.lock"
REFRESH_SKEW_S = 120.0
LOGIN_TIMEOUT_S = 300.0

UrlOpen = Callable[..., object]


class OAuthError(RuntimeError):
    pass


class OAuthLoginError(OAuthError):
    pass


class OAuthRefreshError(OAuthError):
    pass


class OAuthReauthRequired(OAuthRefreshError):
    pass


def _root(project_root: Path | str | None) -> Path:
    if project_root is not None:
        return Path(project_root).expanduser().resolve()
    raw = os.environ.get("R2B4_ROOT", "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def credential_path(project_root: Path | str | None = None) -> Path:
    return _root(project_root) / CREDENTIAL_REL


def host_id_path(project_root: Path | str | None = None) -> Path:
    return _root(project_root) / HOST_ID_REL


def _secure_mode(path: Path) -> None:
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + f".tmp.{os.getpid()}.{secrets.token_hex(4)}")
    try:
        with temp.open("w", encoding="utf-8") as handle:
            try:
                os.chmod(temp, 0o600)
            except OSError:
                pass
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp, path)
        _secure_mode(path)
    finally:
        try:
            temp.unlink()
        except FileNotFoundError:
            pass


def _atomic_write_json(path: Path, value: Mapping[str, object]) -> None:
    _atomic_write_text(path, json.dumps(dict(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n")


def ensure_host_id(project_root: Path | str | None = None) -> str:
    path = host_id_path(project_root)
    if path.is_file():
        value = path.read_text(encoding="utf-8").strip()
        if _valid_host_id(value):
            _secure_mode(path)
            return value
        raise OAuthError(f"invalid ChatGPT host ID in {path}")
    value = "urn:uuid:" + str(uuid.uuid4())
    _atomic_write_text(path, value + "\n")
    return value


def _valid_host_id(value: object) -> bool:
    if not isinstance(value, str) or not value.strip():
        return False
    value = value.strip()
    if value.startswith("urn:uuid:"):
        try:
            uuid.UUID(value[len("urn:uuid:"):])
            return True
        except ValueError:
            return False
    return value.startswith("urn:ietf:params:oauth:jwk-thumbprint:") or value.startswith("did:key:")


def _parse_saved_at(value: object) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if isinstance(value, str) and value.strip():
        raw = value.strip().replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(raw).timestamp()
        except ValueError:
            pass
    return 0.0


def _now_iso(now: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if now is None else now, timezone.utc).isoformat().replace("+00:00", "Z")


def _scopes(value: object) -> tuple[str, ...]:
    if isinstance(value, str):
        parts = value.split()
    elif isinstance(value, list):
        parts = [item for item in value if isinstance(item, str)]
    else:
        parts = []
    return tuple(sorted({item.strip() for item in parts if item.strip()}))


def load_credentials(project_root: Path | str | None = None) -> dict[str, object] | None:
    path = credential_path(project_root)
    if not path.is_file():
        return None
    try:
        mode = stat.S_IMODE(path.stat().st_mode)
        if os.name != "nt" and mode & 0o077:
            raise OAuthError(f"ChatGPT OAuth credential permissions are too open: {oct(mode)}; expected 0o600")
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise OAuthError(f"cannot read ChatGPT OAuth credentials: {type(exc).__name__}") from exc
    if not isinstance(data, dict):
        raise OAuthError("ChatGPT OAuth credential file is not a JSON object")
    return data


def _has_direct_scope(creds: Mapping[str, object]) -> bool:
    return DIRECT_SCOPE in _scopes(creds.get("scopes") or creds.get("scope"))


def _expires_at(creds: Mapping[str, object]) -> float:
    explicit = creds.get("expires_at")
    if isinstance(explicit, (int, float)) and not isinstance(explicit, bool):
        return float(explicit)
    saved = creds.get("saved_at_epoch")
    base = float(saved) if isinstance(saved, (int, float)) and not isinstance(saved, bool) else _parse_saved_at(creds.get("saved_at"))
    ttl = creds.get("expires_in")
    if not isinstance(ttl, (int, float)) or isinstance(ttl, bool):
        return 0.0
    return base + float(ttl)


def credential_status(project_root: Path | str | None = None, *, now: float | None = None) -> dict[str, object]:
    current = time.time() if now is None else float(now)
    host = ensure_host_id(project_root)
    creds = load_credentials(project_root)
    if creds is None:
        return {
            "status": "MISSING",
            "host_id": host,
            "credential_file": str(credential_path(project_root)),
            "plan_scope": False,
            "access_valid": False,
            "refresh_present": False,
            "client_id_present": False,
            "email": None,
        }
    expires = _expires_at(creds)
    access = isinstance(creds.get("access_token"), str) and bool(str(creds.get("access_token", "")).strip())
    refresh = isinstance(creds.get("refresh_token"), str) and bool(str(creds.get("refresh_token", "")).strip())
    client = isinstance(creds.get("client_id"), str) and bool(str(creds.get("client_id", "")).strip())
    direct = _has_direct_scope(creds)
    access_valid = access and expires > current
    if direct and client and (access_valid or refresh):
        status = "READY" if access_valid else "REFRESHABLE"
    elif client:
        status = "REAUTH_REQUIRED"
    else:
        status = "INVALID"
    return {
        "status": status,
        "host_id": host,
        "credential_file": str(credential_path(project_root)),
        "plan_scope": direct,
        "access_valid": access_valid,
        "access_expires_in_s": max(0, int(expires - current)) if expires else 0,
        "refresh_present": refresh,
        "client_id_present": client,
        "email": creds.get("email") if isinstance(creds.get("email"), str) else None,
    }


def _b64url_decode(text: str) -> bytes:
    if not isinstance(text, str):
        raise ValueError("base64url value must be text")
    return base64.urlsafe_b64decode(text + "=" * ((4 - len(text) % 4) % 4))


def _json_response(urlopen: UrlOpen, request: urllib.request.Request, *, timeout_s: float = 30.0) -> dict[str, object]:
    try:
        response = urlopen(request, timeout=timeout_s)
        raw = response.read()  # type: ignore[attr-defined]
    except urllib.error.HTTPError as exc:
        raw = exc.read()
        detail = raw.decode("utf-8", errors="replace")[:1000]
        try:
            obj = json.loads(detail)
            if isinstance(obj, Mapping):
                error = obj.get("error")
                if isinstance(error, Mapping):
                    code = error.get("code") or error.get("error")
                    message = error.get("message") or error.get("description")
                    detail = f"{code}: {message}" if code else str(message or obj)
                elif obj.get("detail"):
                    detail = str(obj.get("detail"))
        except Exception:
            pass
        err = OAuthError(f"OAuth HTTP {exc.code}: {detail[:500]}")
        setattr(err, "status_code", exc.code)
        setattr(err, "response_text", detail[:1000])
        raise err from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise OAuthError(f"OAuth network error: {type(exc).__name__}") from exc
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise OAuthError("OAuth endpoint returned invalid JSON") from exc
    if not isinstance(value, dict):
        raise OAuthError("OAuth endpoint returned a non-object JSON value")
    return value


def _fetch_jwks(urlopen: UrlOpen = urllib.request.urlopen) -> dict[str, object]:
    request = urllib.request.Request(JWKS_URI, headers={"Accept": "application/json", "User-Agent": "r2b4-chatgpt-oauth/1"})
    return _json_response(urlopen, request)


def _verify_rs256(signing_input: bytes, signature: bytes, jwk: Mapping[str, object]) -> None:
    if jwk.get("kty") != "RSA":
        raise OAuthLoginError("ID token JWKS key is not RSA")
    n_raw, e_raw = jwk.get("n"), jwk.get("e")
    if not isinstance(n_raw, str) or not isinstance(e_raw, str):
        raise OAuthLoginError("ID token JWKS RSA key is incomplete")
    n = int.from_bytes(_b64url_decode(n_raw), "big")
    e = int.from_bytes(_b64url_decode(e_raw), "big")
    if n <= 0 or e <= 1:
        raise OAuthLoginError("ID token JWKS RSA key is invalid")
    k = (n.bit_length() + 7) // 8
    if len(signature) != k:
        raise OAuthLoginError("ID token signature length is invalid")
    em = pow(int.from_bytes(signature, "big"), e, n).to_bytes(k, "big")
    digest_info = bytes.fromhex("3031300d060960864801650304020105000420") + hashlib.sha256(signing_input).digest()
    if len(em) < len(digest_info) + 11 or not em.startswith(b"\x00\x01"):
        raise OAuthLoginError("ID token signature padding is invalid")
    separator = em.find(b"\x00", 2)
    if separator < 10 or any(byte != 0xFF for byte in em[2:separator]):
        raise OAuthLoginError("ID token signature padding is invalid")
    if not hmac.compare_digest(em[separator + 1 :], digest_info):
        raise OAuthLoginError("ID token signature verification failed")


def validate_id_token(
    token: str,
    *,
    client_id: str,
    nonce: str,
    urlopen: UrlOpen = urllib.request.urlopen,
    now: float | None = None,
    jwks: Mapping[str, object] | None = None,
) -> dict[str, object]:
    try:
        parts = token.split(".")
        if len(parts) != 3:
            raise ValueError("JWT must have three parts")
        header = json.loads(_b64url_decode(parts[0]).decode("utf-8"))
        claims = json.loads(_b64url_decode(parts[1]).decode("utf-8"))
        signature = _b64url_decode(parts[2])
    except Exception as exc:
        raise OAuthLoginError("ID token is not a valid JWT") from exc
    if not isinstance(header, Mapping) or not isinstance(claims, dict):
        raise OAuthLoginError("ID token header/claims are invalid")
    if header.get("alg") != "RS256" or not isinstance(header.get("kid"), str):
        raise OAuthLoginError("ID token must use RS256 with a kid")
    keyset = dict(jwks) if isinstance(jwks, Mapping) else _fetch_jwks(urlopen)
    keys = keyset.get("keys")
    if not isinstance(keys, list):
        raise OAuthLoginError("OpenAI JWKS contains no keys")
    key = next((item for item in keys if isinstance(item, Mapping) and item.get("kid") == header.get("kid")), None)
    if key is None:
        raise OAuthLoginError("ID token signing key was not found in OpenAI JWKS")
    _verify_rs256(f"{parts[0]}.{parts[1]}".encode("ascii"), signature, key)

    current = time.time() if now is None else float(now)
    if claims.get("iss") != ISSUER:
        raise OAuthLoginError("ID token issuer mismatch")
    aud = claims.get("aud")
    audiences = {aud} if isinstance(aud, str) else set(aud) if isinstance(aud, list) and all(isinstance(x, str) for x in aud) else set()
    if client_id not in audiences:
        raise OAuthLoginError("ID token audience mismatch")
    exp = claims.get("exp")
    if not isinstance(exp, (int, float)) or isinstance(exp, bool) or float(exp) <= current - 5:
        raise OAuthLoginError("ID token is expired")
    iat = claims.get("iat")
    if isinstance(iat, (int, float)) and not isinstance(iat, bool) and float(iat) > current + 60:
        raise OAuthLoginError("ID token issued-at time is in the future")
    if claims.get("nonce") != nonce:
        raise OAuthLoginError("ID token nonce mismatch")
    if not isinstance(claims.get("sub"), str) or not str(claims.get("sub")).strip():
        raise OAuthLoginError("ID token subject is missing")
    return claims


def _token_request(form: Mapping[str, str], *, urlopen: UrlOpen = urllib.request.urlopen) -> dict[str, object]:
    request = urllib.request.Request(
        TOKEN_ENDPOINT,
        data=urllib.parse.urlencode(dict(form)).encode("utf-8"),
        method="POST",
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "Accept": "application/json",
            "User-Agent": "r2b4-chatgpt-oauth/1",
        },
    )
    return _json_response(urlopen, request)


def _credential_record(
    token_response: Mapping[str, object],
    *,
    client_id: str,
    host_id: str,
    claims: Mapping[str, object],
    now: float | None = None,
) -> dict[str, object]:
    access = token_response.get("access_token")
    refresh = token_response.get("refresh_token")
    id_token = token_response.get("id_token")
    expires_in = token_response.get("expires_in")
    scopes = _scopes(token_response.get("scope") or token_response.get("scopes"))
    if not isinstance(access, str) or not access.strip():
        raise OAuthLoginError("OAuth token response has no access_token")
    if not isinstance(refresh, str) or not refresh.strip():
        raise OAuthLoginError("OAuth token response has no refresh_token; offline_access is required")
    if not isinstance(id_token, str) or not id_token.strip():
        raise OAuthLoginError("OAuth token response has no id_token")
    if not isinstance(expires_in, (int, float)) or isinstance(expires_in, bool) or float(expires_in) <= 0:
        raise OAuthLoginError("OAuth token response has invalid expires_in")
    if DIRECT_SCOPE not in scopes:
        raise OAuthLoginError(f"OAuth grant does not include required scope {DIRECT_SCOPE}")
    saved = time.time() if now is None else float(now)
    out: dict[str, object] = {
        "email": claims.get("email") if isinstance(claims.get("email"), str) else None,
        "issuer": ISSUER,
        "subject": claims["sub"],
        "client_id": client_id,
        "ext_agent_host_id": host_id,
        "id_token": id_token.strip(),
        "access_token": access.strip(),
        "refresh_token": refresh.strip(),
        "token_type": str(token_response.get("token_type") or "Bearer"),
        "expires_in": int(expires_in),
        "saved_at": _now_iso(saved),
        "saved_at_epoch": saved,
        "expires_at": saved + float(expires_in),
        "scopes": list(scopes),
    }
    earliest = token_response.get("earliest_refresh_at")
    if isinstance(earliest, (int, float)) and not isinstance(earliest, bool):
        out["earliest_refresh_at"] = float(earliest)
    return out


def _pkce_verifier() -> str:
    # RFC 7636 allows 43..128 URL-safe characters.
    return secrets.token_urlsafe(64)[:96]


def _code_challenge(verifier: str) -> str:
    return base64.urlsafe_b64encode(hashlib.sha256(verifier.encode("ascii")).digest()).rstrip(b"=").decode("ascii")


class _CallbackHandler(BaseHTTPRequestHandler):
    server_version = "R2B4OAuth/1"

    def do_GET(self) -> None:  # noqa: N802 - stdlib callback name
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path != CALLBACK_PATH:
            self.send_response(404)
            self.end_headers()
            return
        query = urllib.parse.parse_qs(parsed.query, keep_blank_values=True)
        self.server.oauth_result = {key: values[-1] if values else "" for key, values in query.items()}  # type: ignore[attr-defined]
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write("<html><body><h2>R2B4 ChatGPT sign-in complete.</h2>You may close this tab.</body></html>".encode("utf-8"))

    def log_message(self, _format: str, *_args: object) -> None:
        return


def perform_login(
    *,
    host_id: str,
    output_path: Path | str,
    existing: Mapping[str, object] | None = None,
    open_browser: bool = True,
    timeout_s: float = LOGIN_TIMEOUT_S,
    urlopen: UrlOpen = urllib.request.urlopen,
    stdout=None,
) -> dict[str, object]:
    if not _valid_host_id(host_id):
        raise OAuthLoginError("invalid ext_agent_host_id")
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")
    saved_client = existing.get("client_id") if isinstance(existing, Mapping) else None
    client_id = saved_client.strip() if isinstance(saved_client, str) and saved_client.strip() else DYNAMIC_CLIENT_ID
    state = secrets.token_urlsafe(32)
    nonce = secrets.token_urlsafe(32)
    verifier = _pkce_verifier()

    server = ThreadingHTTPServer(("127.0.0.1", 0), _CallbackHandler)
    server.timeout = 0.5
    server.oauth_result = None  # type: ignore[attr-defined]
    redirect_uri = f"http://127.0.0.1:{server.server_address[1]}{CALLBACK_PATH}"
    params: dict[str, str] = {
        "client_id": client_id,
        "ext_agent_host_id": host_id,
        "response_type": "code",
        "redirect_uri": redirect_uri,
        "scope": " ".join(REQUESTED_SCOPES),
        "resource": RESOURCE,
        "state": state,
        "nonce": nonce,
        "code_challenge_method": "S256",
        "code_challenge": _code_challenge(verifier),
    }
    if client_id == DYNAMIC_CLIENT_ID:
        params["agent_name_hint"] = AGENT_NAME
    elif isinstance(existing, Mapping):
        if isinstance(existing.get("id_token"), str) and str(existing.get("id_token")).strip():
            params["id_token_hint"] = str(existing["id_token"]).strip()
        if isinstance(existing.get("email"), str) and str(existing.get("email")).strip():
            params["login_hint"] = str(existing["email"]).strip()
    auth_url = AUTH_ENDPOINT + "?" + urllib.parse.urlencode(params)
    stream = stdout if stdout is not None else __import__("sys").stdout
    print("Open this URL in a browser on THIS computer:", file=stream)
    print(auth_url, file=stream, flush=True)
    if open_browser:
        try:
            webbrowser.open(auth_url, new=2)
        except Exception:
            pass

    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and server.oauth_result is None:  # type: ignore[attr-defined]
            server.handle_request()
        result = server.oauth_result  # type: ignore[attr-defined]
    finally:
        server.server_close()
    if not isinstance(result, Mapping):
        raise OAuthLoginError("ChatGPT sign-in callback timed out")
    if result.get("state") != state:
        raise OAuthLoginError("OAuth state mismatch")
    if result.get("error"):
        raise OAuthLoginError(f"ChatGPT authorization failed: {result.get('error')}")
    code = result.get("code")
    if not isinstance(code, str) or not code:
        raise OAuthLoginError("OAuth callback contains no authorization code")
    callback_client = result.get("client_id")
    if client_id == DYNAMIC_CLIENT_ID:
        if not isinstance(callback_client, str) or not callback_client or callback_client == DYNAMIC_CLIENT_ID:
            raise OAuthLoginError("dynamic registration returned no issued client_id")
        issued_client = callback_client
    else:
        if isinstance(callback_client, str) and callback_client and callback_client != client_id:
            raise OAuthLoginError("OAuth callback client_id does not match saved registration")
        issued_client = client_id

    try:
        token_response = _token_request(
            {
                "grant_type": "authorization_code",
                "client_id": issued_client,
                "code": code,
                "code_verifier": verifier,
                "redirect_uri": redirect_uri,
                "resource": RESOURCE,
            },
            urlopen=urlopen,
        )
    except OAuthError as exc:
        raise OAuthLoginError(str(exc)) from exc
    id_token = token_response.get("id_token")
    if not isinstance(id_token, str):
        raise OAuthLoginError("OAuth token response has no id_token")
    claims = validate_id_token(id_token, client_id=issued_client, nonce=nonce, urlopen=urlopen)
    if isinstance(existing, Mapping) and isinstance(existing.get("subject"), str):
        if existing.get("subject") != claims.get("sub"):
            raise OAuthLoginError("reauthorized ChatGPT account identity does not match saved registration")
    record = _credential_record(token_response, client_id=issued_client, host_id=host_id, claims=claims)
    _atomic_write_json(Path(output_path).expanduser().resolve(), record)
    return record


def login(project_root: Path | str | None = None, *, open_browser: bool = True) -> dict[str, object]:
    root = _root(project_root)
    host = ensure_host_id(root)
    existing = load_credentials(root)
    return perform_login(host_id=host, output_path=credential_path(root), existing=existing, open_browser=open_browser)


@contextmanager
def _refresh_file_lock(root: Path) -> Iterator[None]:
    path = root / LOCK_REL
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    _secure_mode(path)
    try:
        if fcntl is not None:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        yield
    finally:
        if fcntl is not None:
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
        handle.close()


class ChatGPTOAuthTokenProvider:
    """Load, renew and return one ChatGPT-plan access token."""

    def __init__(
        self,
        project_root: Path | str | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
        now: Callable[[], float] = time.time,
        refresh_skew_s: float = REFRESH_SKEW_S,
    ) -> None:
        if refresh_skew_s < 0:
            raise ValueError("refresh_skew_s must be non-negative")
        self._root = _root(project_root)
        self._urlopen = urlopen
        self._now = now
        self._refresh_skew_s = float(refresh_skew_s)
        self._thread_lock = threading.Lock()

    @property
    def auth_mode(self) -> str:
        return "oauth"

    def available(self) -> bool:
        try:
            status = credential_status(self._root, now=self._now())
        except OAuthError:
            return False
        return status["status"] in {"READY", "REFRESHABLE"}

    def get_access_token(self, *, force_refresh: bool = False) -> str:
        creds = load_credentials(self._root)
        if creds is None:
            raise OAuthReauthRequired("ChatGPT OAuth credentials are not configured; run: ./r chatgpt login")
        if not _has_direct_scope(creds):
            raise OAuthReauthRequired(f"ChatGPT OAuth grant lacks {DIRECT_SCOPE}; sign in again")
        return self._token_or_refresh(creds, force_refresh=force_refresh)

    def _token_or_refresh(self, creds: Mapping[str, object], *, force_refresh: bool) -> str:
        now = self._now()
        access = creds.get("access_token")
        expires = _expires_at(creds)
        needs_refresh = force_refresh or not isinstance(access, str) or not access.strip() or expires <= now + self._refresh_skew_s
        if not needs_refresh:
            return access.strip()
        refresh = creds.get("refresh_token")
        if not isinstance(refresh, str) or not refresh.strip():
            raise OAuthReauthRequired("ChatGPT OAuth refresh token is unavailable; sign in again")
        earliest = creds.get("earliest_refresh_at")
        if isinstance(earliest, (int, float)) and not isinstance(earliest, bool) and now < float(earliest):
            if isinstance(access, str) and access.strip() and expires > now:
                return access.strip()
            raise OAuthRefreshError("ChatGPT OAuth token cannot be refreshed yet")

        with self._thread_lock:
            with _refresh_file_lock(self._root):
                # Another short-lived R2B4 process may have rotated the refresh token.
                latest = load_credentials(self._root)
                if latest is None:
                    raise OAuthReauthRequired("ChatGPT OAuth credentials disappeared during refresh")
                now = self._now()
                latest_access = latest.get("access_token")
                latest_exp = _expires_at(latest)
                if not force_refresh and isinstance(latest_access, str) and latest_access.strip() and latest_exp > now + self._refresh_skew_s:
                    return latest_access.strip()
                return self._refresh(latest)

    def _refresh(self, creds: Mapping[str, object]) -> str:
        client_id = creds.get("client_id")
        refresh_token = creds.get("refresh_token")
        if not isinstance(client_id, str) or not client_id.strip() or client_id == DYNAMIC_CLIENT_ID:
            raise OAuthReauthRequired("ChatGPT OAuth issued client_id is missing")
        if not isinstance(refresh_token, str) or not refresh_token.strip():
            raise OAuthReauthRequired("ChatGPT OAuth refresh token is missing")
        try:
            response = _token_request(
                {
                    "grant_type": "refresh_token",
                    "client_id": client_id.strip(),
                    "refresh_token": refresh_token.strip(),
                    "resource": RESOURCE,
                },
                urlopen=self._urlopen,
            )
        except OAuthError as exc:
            text = str(exc).lower()
            status = getattr(exc, "status_code", None)
            if status in {400, 401, 403} or "invalid_grant" in text or "invalid token" in text or "invalid_token" in text:
                raise OAuthReauthRequired(f"ChatGPT OAuth refresh requires sign-in: {exc}") from exc
            raise OAuthRefreshError(f"ChatGPT OAuth refresh failed: {exc}") from exc

        access = response.get("access_token")
        replacement = response.get("refresh_token")
        expires_in = response.get("expires_in")
        scopes = _scopes(response.get("scope") or response.get("scopes") or creds.get("scopes"))
        if not isinstance(access, str) or not access.strip():
            raise OAuthRefreshError("ChatGPT OAuth refresh returned no access_token")
        if not isinstance(replacement, str) or not replacement.strip():
            raise OAuthRefreshError("ChatGPT OAuth refresh returned no replacement refresh_token")
        if not isinstance(expires_in, (int, float)) or isinstance(expires_in, bool) or float(expires_in) <= 0:
            raise OAuthRefreshError("ChatGPT OAuth refresh returned invalid expires_in")
        if DIRECT_SCOPE not in scopes:
            raise OAuthReauthRequired(f"ChatGPT OAuth refreshed grant lacks {DIRECT_SCOPE}")
        saved = self._now()
        updated = dict(creds)
        updated.update(
            {
                "access_token": access.strip(),
                "refresh_token": replacement.strip(),
                "token_type": str(response.get("token_type") or creds.get("token_type") or "Bearer"),
                "expires_in": int(expires_in),
                "saved_at": _now_iso(saved),
                "saved_at_epoch": saved,
                "expires_at": saved + float(expires_in),
                "scopes": list(scopes),
            }
        )
        earliest = response.get("earliest_refresh_at")
        if isinstance(earliest, (int, float)) and not isinstance(earliest, bool):
            updated["earliest_refresh_at"] = float(earliest)
        else:
            updated.pop("earliest_refresh_at", None)
        # Do not replace the retained ID token on refresh unless a future flow
        # explicitly validates a newly returned one with the correct nonce.
        _atomic_write_json(credential_path(self._root), updated)
        return access.strip()


def import_credentials(source: Path | str, project_root: Path | str | None = None) -> dict[str, object]:
    root = _root(project_root)
    host = ensure_host_id(root)
    src = Path(source).expanduser().resolve()
    try:
        value = json.loads(src.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise OAuthError(f"cannot import ChatGPT OAuth credential file: {type(exc).__name__}") from exc
    if not isinstance(value, dict):
        raise OAuthError("imported ChatGPT credential is not a JSON object")
    imported_host = value.get("ext_agent_host_id")
    if imported_host != host:
        raise OAuthError(
            "imported credential host ID does not match this R2B4 host; "
            "run './r chatgpt host-id' on the Pi and use that exact ID during desktop sign-in"
        )
    for field in ("client_id", "access_token", "refresh_token", "subject"):
        if not isinstance(value.get(field), str) or not str(value[field]).strip():
            raise OAuthError(f"imported ChatGPT credential is missing {field}")
    if value.get("client_id") == DYNAMIC_CLIENT_ID:
        raise OAuthError("imported credential contains dynamic_agent_client instead of an issued client ID")
    if not _has_direct_scope(value):
        raise OAuthError(f"imported ChatGPT credential lacks {DIRECT_SCOPE}")
    _atomic_write_json(credential_path(root), value)
    return credential_status(root)


def _discover(urlopen: UrlOpen = urllib.request.urlopen) -> dict[str, object]:
    request = urllib.request.Request(DISCOVERY_URI, headers={"Accept": "application/json", "User-Agent": "r2b4-chatgpt-oauth/1"})
    return _json_response(urlopen, request)


def logout(project_root: Path | str | None = None, *, urlopen: UrlOpen = urllib.request.urlopen) -> bool:
    root = _root(project_root)
    creds = load_credentials(root)
    if creds is None:
        return True
    refresh = creds.get("refresh_token")
    client_id = creds.get("client_id")
    revoked = True
    if isinstance(refresh, str) and refresh.strip() and isinstance(client_id, str) and client_id.strip():
        try:
            discovery = _discover(urlopen)
            endpoint = discovery.get("revocation_endpoint")
            if not isinstance(endpoint, str) or not endpoint.startswith("https://"):
                raise OAuthError("OIDC discovery returned no revocation_endpoint")
            request = urllib.request.Request(
                endpoint,
                data=urllib.parse.urlencode(
                    {
                        "token": refresh.strip(),
                        "token_type_hint": "refresh_token",
                        "client_id": client_id.strip(),
                    }
                ).encode("utf-8"),
                method="POST",
                headers={"Content-Type": "application/x-www-form-urlencoded", "User-Agent": "r2b4-chatgpt-oauth/1"},
            )
            response = urlopen(request, timeout=30.0)
            response.read()  # type: ignore[attr-defined]
        except Exception:
            revoked = False
    # Retain the client/account mapping and host ID, but clear usable tokens.
    retained = {
        key: creds[key]
        for key in ("email", "issuer", "subject", "client_id", "ext_agent_host_id")
        if key in creds
    }
    retained["signed_out_at"] = _now_iso()
    _atomic_write_json(credential_path(root), retained)
    return revoked


def cli_main(argv: list[str] | None = None, *, project_root: Path | str | None = None) -> int:
    parser = argparse.ArgumentParser(prog="r chatgpt", description="R2B4 Sign in with ChatGPT OAuth")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="show local OAuth state without a network request")
    sub.add_parser("host-id", help="show the persistent R2B4 host ID")
    login_p = sub.add_parser("login", help="sign in using a browser on this same computer")
    login_p.add_argument("--no-open", action="store_true", help="print the URL without opening the browser")
    import_p = sub.add_parser("import", help="import a credential file created on another computer for this host ID")
    import_p.add_argument("file")
    sub.add_parser("logout", help="revoke the renewable session when possible and clear local tokens")
    args = parser.parse_args(argv)
    root = _root(project_root)
    try:
        if args.command == "host-id":
            print(ensure_host_id(root))
            return 0
        if args.command == "status":
            print(json.dumps(credential_status(root), ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        if args.command == "login":
            record = login(root, open_browser=not args.no_open)
            print(json.dumps({"status": "READY", "email": record.get("email"), "client_id": record.get("client_id")}, ensure_ascii=False, indent=2))
            return 0
        if args.command == "import":
            print(json.dumps(import_credentials(args.file, root), ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        if args.command == "logout":
            revoked = logout(root)
            print("ChatGPT OAuth: signed out" + ("" if revoked else " (local tokens cleared; remote revocation not confirmed)"))
            return 0
    except OAuthError as exc:
        print(f"ERROR: {exc}", file=__import__("sys").stderr)
        return 2
    return 2


__all__ = [
    "AGENT_NAME",
    "ChatGPTOAuthTokenProvider",
    "DIRECT_SCOPE",
    "OAuthError",
    "OAuthLoginError",
    "OAuthReauthRequired",
    "OAuthRefreshError",
    "credential_path",
    "credential_status",
    "ensure_host_id",
    "host_id_path",
    "import_credentials",
    "load_credentials",
    "login",
    "logout",
    "perform_login",
    "validate_id_token",
    "cli_main",
]
