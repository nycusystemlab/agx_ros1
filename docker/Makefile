# ==============================================================================
#  AGX ROS 1 Control Interface (ros1_ws) - Simplified Makefile
# ==============================================================================

# --- [Configuration] ---
PROJECT_NAME   := agx_ros_1
TASK_CONTAINER := control

# Docker Buildkit
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# --- [Auto-Detection] ---
CURRENT_CONTEXT := $(shell docker context show)
MODE := pc
ENV_FILE := ../.env
COMPOSE_FILE :=

ifneq (,$(findstring agx,$(CURRENT_CONTEXT)))
    MODE := agx
    ENV_FILE := ../.env.agx
    COMPOSE_FILE := -f docker-compose.agx.yaml
else ifeq ($(shell uname -m), aarch64)
    MODE := agx
    ENV_FILE := ../.env.agx
    COMPOSE_FILE := -f docker-compose.agx.yaml
endif

# --- [Compose Commands] ---
COMPOSE_CMD := docker compose --env-file $(ENV_FILE) -f docker-compose.yaml $(COMPOSE_FILE) -p $(PROJECT_NAME)
TASK_EXEC   := docker exec -it $(TASK_CONTAINER) bash -ic

# --- [Make Settings] ---
.DEFAULT_GOAL := help
.SILENT:
.PHONY: help build up down rebuild logs ps clean join run stop

# ==============================================================================
#  Targets
# ==============================================================================

help: ## Show available commands
	echo "🚀 AGX ROS 1 Workspace ($(MODE) mode)"
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	echo ""
	echo "--- Docker Services ---"
	echo "  make up              Start all services"
	echo "  make down            Stop all services"
	echo "  make build           Build images"
	echo "  make rebuild         Force rebuild all services"
	echo ""
	echo "--- Status & Logs ---"
	echo "  make logs            Follow service logs"
	echo "  make ps              Show running containers"
	echo "  make clean           Remove all containers & images"
	echo ""
	echo "--- Container Access ---"
	echo "  make join            Enter container shell"
	echo "  make run             Launch ROS tasks"
	echo ""

build: ## Build all images
	$(COMPOSE_CMD) build

up: ## Start all services
	$(COMPOSE_CMD) up -d
	echo "✅ Services started ($(MODE) mode)"

down: ## Stop all services
	$(COMPOSE_CMD) down --remove-orphans
	echo "✅ Services stopped"

rebuild: ## Force rebuild and recreate all services
	$(COMPOSE_CMD) up -d --build --force-recreate
	echo "✅ Services rebuilt"

logs: ## Follow service logs
	$(COMPOSE_CMD) logs -f

ps: ## Show container status
	$(COMPOSE_CMD) ps

clean: ## Remove all containers and images
	$(COMPOSE_CMD) down --rmi local -v --remove-orphans
	echo "✅ Cleanup complete"

join: ## Enter container shell (interactive)
	@echo "Available containers:"
	@docker ps --filter "label=com.docker.compose.project=$(PROJECT_NAME)" --format "table {{.Names}}\t{{.Status}}"
	@read -p "Enter container name: " CONTAINER; \
	if [ -n "$$CONTAINER" ]; then \
		docker exec -it $$CONTAINER bash; \
	fi

run: ## Launch ROS tasks (in control container)
	@echo "[*] Launching tasks in $(TASK_CONTAINER) container..."
	$(TASK_EXEC) 'echo "roscore status:" && rostopic list | head -5'

stop: ## Stop all background tasks
	@echo "[*] Stopping background tmux sessions..."
	@tmux list-sessions -F "#{session_name}" 2>/dev/null | grep -E "^agx_" | xargs -r tmux kill-session -t || echo "No tasks running"
