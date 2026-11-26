# PITWALL on Railway (or any Docker host). Mount a volume at /data so accounts, scripts and laps survive deploys.
FROM python:3.13-slim
RUN pip install --no-cache-dir numpy==2.2.5 \
 && useradd --system --no-create-home --shell /usr/sbin/nologin bot
WORKDIR /app
COPY server.py runner.py botview.py track.py manage.py ./
COPY examples ./examples
COPY static ./static
ENV HOST=0.0.0.0 PITWALL_DATA=/data PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
CMD ["python3", "server.py"]
