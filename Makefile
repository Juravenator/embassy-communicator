SHELL:=/usr/bin/env bash -o pipefail
# Run `make` to see some helpful output
.DEFAULT_GOAL:=help

include .make/vars.mk

##
### main targets
##

build: ## build code
	cargo build

run: ## flash to device and get logs
	cargo run --release

probe: ## attach to device and get logs
	probe-run --chip STM32H723ZGTx --no-flash target/thumbv7em-none-eabihf/release/${PROJECT_NAME}

test: ## run tests
	cargo test --test integration

##
### helper targets
##

include .make/dependencies.mk
include .make/help.mk