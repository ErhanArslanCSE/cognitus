# COGNITUS - Simple Commands (like NextJS)

.PHONY: help dev sim start stop clean

help: ## Show available commands
	@echo "COGNITUS Commands:"
	@echo ""
	@echo "  make dev      - Start development environment (Docker)"
	@echo "  make sim      - Start Gazebo simulation (Docker)"
	@echo "  make start    - Start on LIMO Pro (production)"
	@echo "  make stop     - Stop Docker containers"
	@echo "  make clean    - Clean everything"
	@echo ""

dev: ## Start development environment (like 'npm run dev')
	@echo "Starting COGNITUS development environment..."
	@xhost +local:docker 2>/dev/null || true
	docker-compose up cognitus

sim: ## Start Gazebo simulation
	@echo "Starting Gazebo simulation..."
	@xhost +local:docker 2>/dev/null || true
	docker-compose up simulation

start: ## Start on LIMO Pro (production)
	@echo "Starting COGNITUS on LIMO Pro..."
	./start.sh

stop: ## Stop Docker containers
	docker-compose down

clean: ## Clean Docker artifacts
	docker-compose down -v
	docker system prune -f

.DEFAULT_GOAL := help
