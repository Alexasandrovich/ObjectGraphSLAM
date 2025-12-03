xhost +local:root
docker compose -f docker/docker-compose.yml up --build --force-recreate