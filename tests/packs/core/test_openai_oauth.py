from __future__ import annotations

import json
import urllib.parse
from pathlib import Path

from r2b4_voice.openai_oauth import (
    ChatGPTOAuthTokenProvider,
    DIRECT_SCOPE,
    ensure_host_id,
    validate_id_token,
)

JWT = "eyJhbGciOiJSUzI1NiIsImtpZCI6InRlc3Qta2V5IiwidHlwIjoiSldUIn0.eyJpc3MiOiJodHRwczovL2F1dGgub3BlbmFpLmNvbSIsImF1ZCI6Im9haWFwcF90ZXN0IiwiZXhwIjoyMDAwMDAwMDAwLCJpYXQiOjE4OTk5OTk5MDAsIm5vbmNlIjoibm9uY2UtdGVzdCIsInN1YiI6InN1YmplY3QtdGVzdCIsImVtYWlsIjoidUBleGFtcGxlLmNvbSJ9.vuFfUqZrAsZRZHvx0_9S1Je_VG1ZCYT9U8VEkBlwiojy3PxFovem0xTjL4imG6-rApEIkrk6MksphqdBmjXNrMd-kMqXiAk611SWfFaYbADczRUqUBertfSkMJUwp9NeIBXekFt73M7WAD_4iWEFagJTyaLpo9UEUMA4sNppFMgKbHnkWOwZgWetTdqolA-0o9q7u9GIPIDKtz96Gv9GEZr6vI1cNLHjqxczgnY4lwCoiyCgM-cKlEVQFbA52TujGppHUHcoKWqB9GIs-ImlpEI4jfy_MxxA1rYIOer6xD1b8JWN2u8X2R2aXNFOggX3b9-SAICgpnjMvWHpTv-LWA"
JWKS = {
    "keys": [
        {
            "kty": "RSA",
            "kid": "test-key",
            "n": "5Hh3VLTCECJqjDhyx8sJ2ax0ZR2IH4Y_HLx4J7Dd7IK2amL8AJthmdNL0Fu1iDWjGtJ2RSfhhyJTzGb0y0GhBRtaP5zZI7k80VtQvo58gK6nvI9sOJ7X0mAt2Csr84A3ps5OimMS3JPb7AqybhxDFOLZDbuRqtcgSrUBqMC7PUBSVXGyx5ky3PD7EBuzTG_4-qA8NS-8nciOAQgyAzs4Gz5RHY-pcZIiyqGEXQJ_EeuB8rW4vpgfY_Se75S4egji9K6Zu7HwV0ats5dUaF8amkfzbRJpEl1a2PESuiO8zE8TjCZBY01R8D5zNPdc4B5StIrhNPiQI_TZdE4SRNqUuQ",
            "e": "AQAB",
        }
    ]
}


class JsonResponse:
    def __init__(self, payload):
        self.payload = json.dumps(payload).encode()

    def read(self):
        return self.payload


def test_id_token_rs256_validation_without_external_jwt_library() -> None:
    claims = validate_id_token(
        JWT,
        client_id="oaiapp_test",
        nonce="nonce-test",
        now=1_900_000_000,
        jwks=JWKS,
    )
    assert claims["sub"] == "subject-test"
    assert claims["email"] == "u@example.com"


def test_expired_access_token_rotates_refresh_token(tmp_path: Path) -> None:
    host = ensure_host_id(tmp_path)
    conf = tmp_path / "conf"
    cred = conf / ".chatgpt_oauth.json"
    cred.write_text(
        json.dumps(
            {
                "client_id": "oaiapp_test",
                "subject": "subject-test",
                "issuer": "https://auth.openai.com",
                "ext_agent_host_id": host,
                "access_token": "old-access",
                "refresh_token": "old-refresh",
                "expires_in": 1,
                "saved_at_epoch": 0,
                "scopes": [DIRECT_SCOPE, "offline_access", "resource.invoke"],
            }
        )
    )
    cred.chmod(0o600)
    seen = {}

    def fake_urlopen(request, timeout):
        seen["timeout"] = timeout
        body = urllib.parse.parse_qs(request.data.decode())
        seen["body"] = body
        return JsonResponse(
            {
                "access_token": "new-access",
                "refresh_token": "new-refresh",
                "token_type": "Bearer",
                "expires_in": 3600,
                "scope": f"{DIRECT_SCOPE} offline_access resource.invoke",
            }
        )

    provider = ChatGPTOAuthTokenProvider(tmp_path, urlopen=fake_urlopen, now=lambda: 1000.0)
    assert provider.get_access_token() == "new-access"
    saved = json.loads(cred.read_text())
    assert saved["refresh_token"] == "new-refresh"
    assert saved["access_token"] == "new-access"
    assert seen["body"]["grant_type"] == ["refresh_token"]
    assert seen["body"]["client_id"] == ["oaiapp_test"]
    assert "scope" not in seen["body"]
