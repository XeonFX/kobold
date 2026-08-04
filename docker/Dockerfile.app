# Kobold web app — FastAPI + a no-build-step PWA.
#
# The static files are NOT copied in. They are bind-mounted from the git
# checkout (see compose.yaml), so a UI change deploys with `git checkout` plus a
# browser refresh — no rebuild, no registry pull. That is the concrete payoff
# for choosing vanilla ES modules over a bundled framework.
FROM python:3.12-slim AS base

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

# Dependencies in their own layer so app code changes do not re-resolve pip.
COPY app/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app/server.py .
# A placeholder so the container still starts if the bind mount is missing —
# an app that fails to boot is harder to diagnose than one serving an error page.
RUN mkdir -p /app/static

EXPOSE 8000

# curl kept deliberately: this is what deploy.sh's health check calls, and a
# container that cannot be health-checked is a container you cannot deploy safely.
RUN apt-get update && apt-get install -y --no-install-recommends curl \
    && rm -rf /var/lib/apt/lists/*

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
  CMD curl -fsS http://localhost:8000/healthz || exit 1

CMD ["python", "-m", "uvicorn", "server:app", "--host", "0.0.0.0", "--port", "8000"]
