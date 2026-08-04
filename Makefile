# Kobold — top-level developer commands.
#
#   make help          list targets
#   make check         everything CI runs
#
# Firmware builds need PlatformIO (pip install platformio).
# ROS 2 targets need a sourced ROS 2 Jazzy environment.

.DEFAULT_GOAL := help
SHELL := /bin/bash

PYTHON  ?= python3
PIO     ?= pio
BRIDGE  := ros2_ws/src/kobold_bridge

.PHONY: help
help: ## Show this help
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN{FS=":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'

# ---------------------------------------------------------------- protocol --

.PHONY: protocol
protocol: ## Regenerate C++/Python bindings from protocol/protocol.yaml
	$(PYTHON) tools/gen_protocol.py

.PHONY: protocol-check
protocol-check: ## Fail if the generated bindings are stale (CI gate)
	@$(PYTHON) tools/gen_protocol.py >/dev/null
	@if ! git diff --quiet -- firmware/lib/kobold_protocol/protocol_generated.h \
	       $(BRIDGE)/kobold_bridge/protocol_generated.py; then \
	  echo "ERROR: generated protocol bindings are stale."; \
	  echo "Run 'make protocol' and commit the result."; \
	  git diff --stat -- firmware/lib/kobold_protocol/protocol_generated.h \
	       $(BRIDGE)/kobold_bridge/protocol_generated.py; \
	  exit 1; \
	fi
	@echo "protocol bindings up to date"

# ---------------------------------------------------------------- firmware --

.PHONY: firmware
firmware: ## Build drive + sense firmware
	cd firmware && $(PIO) run -e drive -e sense

.PHONY: firmware-test
firmware-test: ## Run native codec tests (no hardware needed)
	cd firmware && $(PIO) test -e native

.PHONY: flash-drive
flash-drive: ## Flash the drive board over USB
	cd firmware && $(PIO) run -e drive -t upload

.PHONY: flash-sense
flash-sense: ## Flash the sense board over USB
	cd firmware && $(PIO) run -e sense -t upload

.PHONY: flash
flash: flash-drive flash-sense ## Flash both boards

# --------------------------------------------------------------------- ROS --

.PHONY: build
build: ## colcon build the ROS 2 workspace
	cd ros2_ws && colcon build --symlink-install

.PHONY: test
test: ## Run Python tests (framing codec, bridge)
	cd $(BRIDGE) && $(PYTHON) -m pytest test/ -v

.PHONY: sim
sim: ## Run the firmware simulator (fake boards on PTYs, no hardware)
	cd $(BRIDGE) && $(PYTHON) -m kobold_bridge.sim

.PHONY: bringup
bringup: ## Launch the robot base
	ros2 launch kobold_bringup bringup.launch.py

# ------------------------------------------------------------------ checks --

.PHONY: lint
lint: ## Lint Python
	$(PYTHON) -m ruff check tools/ $(BRIDGE) || true
	$(PYTHON) -m ruff format --check tools/ $(BRIDGE) || true

.PHONY: format
format: ## Auto-format Python
	$(PYTHON) -m ruff format tools/ $(BRIDGE)

.PHONY: check
check: protocol-check test firmware-test ## Everything CI runs

.PHONY: clean
clean: ## Remove build artefacts
	rm -rf firmware/.pio ros2_ws/build ros2_ws/install ros2_ws/log
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
	find . -name '*.pyc' -delete

# ------------------------------------------------------------------ deploy --
# One git SHA is one complete robot state: images, firmware and config all come
# from the same commit. Deploys are always explicit — never automatic.

ROBOT       ?= kobold
ROBOT_ROOT  ?= /opt/kobold
SHA         ?= $(shell git rev-parse --short=8 HEAD)

.PHONY: udev
udev: ## Install udev rules for stable device names (Linux, needs sudo)
	sudo ./scripts/setup-udev.sh

.PHONY: robot-setup
robot-setup: ## One-time bootstrap on the robot (/opt/kobold + systemd units)
	ssh $(ROBOT) 'bash -s' < scripts/robot-setup.sh

.PHONY: deploy
deploy: ## Deploy a commit to the robot   (make deploy SHA=abc12345)
	@echo "deploying $(SHA) to $(ROBOT)"
	ssh -t $(ROBOT) '$(ROBOT_ROOT)/scripts/deploy.sh $(SHA)'

.PHONY: deploy-latest
deploy-latest: ## Deploy the newest commit on origin/main
	ssh -t $(ROBOT) '$(ROBOT_ROOT)/scripts/deploy.sh --latest'

.PHONY: rollback
rollback: ## Roll back the robot to the previous deployment
	ssh -t $(ROBOT) '$(ROBOT_ROOT)/scripts/rollback.sh'

.PHONY: robot-status
robot-status: ## What is running on the robot right now
	@ssh $(ROBOT) 'echo "sha:      $$(cat /data/deploy/current 2>/dev/null || echo none)"; \
	  echo "previous: $$(cat /data/deploy/previous 2>/dev/null || echo none)"; \
	  echo "update:   $$(cat /data/deploy/available 2>/dev/null || echo "up to date")"; \
	  echo; docker ps --format "  {{.Names}}\t{{.Status}}"; \
	  echo; free -h | head -2; df -h /data | tail -1'

.PHONY: robot-logs
robot-logs: ## Tail robot container logs   (make robot-logs SVC=bridge)
	ssh -t $(ROBOT) 'docker logs -f --tail 100 kobold-$(or $(SVC),bridge)'

.PHONY: robot-shell
robot-shell: ## Shell inside a robot container   (make robot-shell SVC=bridge)
	ssh -t $(ROBOT) 'docker exec -it kobold-$(or $(SVC),bridge) bash'

# Fast local loop: Apple Silicon builds arm64 natively, so this is ~30 s versus
# waiting on CI. Use it while iterating; use CI for anything you deploy.
.PHONY: push-dev
push-dev: ## Build one image locally and ship it straight to the robot
	docker buildx build --platform linux/arm64 --load \
	  -f docker/Dockerfile.$(or $(SVC),ros) -t kobold-$(or $(SVC),ros):dev .
	docker save kobold-$(or $(SVC),ros):dev | ssh $(ROBOT) 'docker load'

# ------------------------------------------------------------------ models --

.PHONY: fetch-models
fetch-models: ## Download + checksum models into /data/models (on the robot)
	ssh -t $(ROBOT) '$(ROBOT_ROOT)/scripts/fetch-models.sh'

.PHONY: check-models
check-models: ## Verify model checksums without downloading
	ssh $(ROBOT) '$(ROBOT_ROOT)/scripts/fetch-models.sh --verify'

# --------------------------------------------------------------------- app --

.PHONY: app-dev
app-dev: ## Run the web app locally against the simulator
	cd app && $(PYTHON) -m uvicorn server:app --reload --host 0.0.0.0 --port 8000
