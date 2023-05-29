## Docker reminders

You could have been using docker for ages and still haven't explored these, so here's a summary of docker stuff you'll find in this repo:

**Compose**: Docker compose allows you to predefine a series of variables in a `docker-compose.yaml` file, pertaining to context-related docker images. Think of it as a more formal version of a docker run in which you would write a 10-line `docker run` plus arguments. The commands you'll probably use the most are:
- `docker compose build IMAGE_NAME`
- `docker compose run IMAGE_NAME`

**Entrypoint**: Entrypoints in docker are typically shell scripts that will be executed when a new container is created. This is handy when you need some shell-level modifications before you do anything, or when casting custom commands from the outside to the docker container. In the context of ROS, they'll help with sourcing our environments, optionally compiling our code, and update system-level configuration based on what instanced volumes have. The latter is particularly useful in this context. Entrypoints become especially tricky if we want to call ros actions from outside a running container (`docker run image rosls`).
