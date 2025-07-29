# web/utils.py  (경로: proj_root/web/utils.py)

from flask import Request

def get_client_ip(req: Request) -> str:
    """
    프록시/로드밸런서를 거친 경우를 포함해
    실제 클라이언트 IP를 반환.
    """
    if (xff := req.environ.get("HTTP_X_FORWARDED_FOR")):
        # 'A, B, C' 형태일 때 첫 번째 IP가 원본
        return xff.split(",")[0].strip()
    return req.remote_addr or "0.0.0.0"
